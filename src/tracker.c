/* tracker.c — α-β 估计 + 二阶临界阻尼 + 状态机
 *
 * scaler OFF 平移模式
 * offset 像素空间 [0,672]×[0,492]
 */
#include "tracker.h"
#include "hal_ov5640.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ═══════════════════════════════════════
 * 常量
 * ═══════════════════════════════════════ */

#define OX_MIN  0
#define OX_MAX  (SUBSAMPLED_1X_W - IMG_W)   /* 672 */
#define OY_MIN  0
#define OY_MAX  (SUBSAMPLED_1X_H - IMG_H)   /* 492 */
#define STARTUP_SKIP_FRAMES  3   /* 切换后跳过的帧数 */

/* ═══════════════════════════════════════
 * 辅助
 * ═══════════════════════════════════════ */

static float clampf(float v, float lo, float hi)
{ return v < lo ? lo : (v > hi ? hi : v); }

static int clampi(int v, int lo, int hi)
{ return v < lo ? lo : (v > hi ? hi : v); }

static int scaler_disabled = 0;

/* ═══════════════════════════════════════
 * 坐标转换
 * ═══════════════════════════════════════ */

/* scaler ON: screen 是 1312×972 缩放到 640×480 */
static void screen_to_sub_scaled(float scr_cx, float scr_cy,
                                 float *sub_x, float *sub_y)
{
    *sub_x = (1.0f - scr_cx) * (float)SUBSAMPLED_1X_W;
    *sub_y = (1.0f - scr_cy) * (float)SUBSAMPLED_1X_H;
}

/* scaler OFF: screen 是 offset 处开始的 640×480 窗口 */
static void screen_to_sub(float scr_cx, float scr_cy,
                          int snap_ox, int snap_oy,
                          float *sub_x, float *sub_y)
{
    *sub_x = (1.0f - scr_cx) * (float)IMG_W + (float)snap_ox;
    *sub_y = (1.0f - scr_cy) * (float)IMG_H + (float)snap_oy;
}

static void sub_to_offset(float sub_x, float sub_y,
                          int *ox, int *oy)
{
    *ox = clampi((int)(sub_x - (float)(IMG_W / 2)), OX_MIN, OX_MAX);
    *oy = clampi((int)(sub_y - (float)(IMG_H / 2)), OY_MIN, OY_MAX);
}

static void det_to_cxcy(const detection_t *d, float *cx, float *cy)
{
    *cx = d->bbox[0] + d->bbox[2] / 2.0f;
    *cy = d->bbox[1] + d->bbox[3] / 2.0f;
}

/* ═══════════════════════════════════════
 * α-β 估计层
 * ═══════════════════════════════════════ */

static void ab_init(ab_estimator_t *e)
{
    memset(e, 0, sizeof(*e));
    e->initialized = 0;
}

static void ab_start(ab_estimator_t *e, float ox, float oy)
{
    e->ox = ox;
    e->oy = oy;
    e->vx = 0;
    e->vy = 0;
    e->initialized = 1;
}

static void ab_predict(ab_estimator_t *e, float dt) {
    if (!e->initialized) return;
    e->ox += e->vx * dt;
    e->oy += e->vy * dt;
    
    if (e->ox < (float)OX_MIN) { e->ox = (float)OX_MIN; if (e->vx < 0) e->vx = 0; }
    if (e->ox > (float)OX_MAX) { e->ox = (float)OX_MAX; if (e->vx > 0) e->vx = 0; }
    if (e->oy < (float)OY_MIN) { e->oy = (float)OY_MIN; if (e->vy < 0) e->vy = 0; }
    if (e->oy > (float)OY_MAX) { e->oy = (float)OY_MAX; if (e->vy > 0) e->vy = 0; }
}

static void ab_update(ab_estimator_t *e, float obs_ox, float obs_oy)
{
    if (!e->initialized) {
        ab_start(e, obs_ox, obs_oy);
        return;
    }

    float rx = obs_ox - e->ox;
    float ry = obs_oy - e->oy;

    e->ox += AB_ALPHA * rx;
    e->oy += AB_ALPHA * ry;
    e->vx += AB_BETA_RATE * rx;
    e->vy += AB_BETA_RATE * ry;
}

/* ═══════════════════════════════════════
 * 二阶临界阻尼执行层
 * ═══════════════════════════════════════ */

static void damp2_init(damp2_t *d)
{
    memset(d, 0, sizeof(*d));
    d->initialized = 0;
}

static void damp2_start(damp2_t *d, float x0, float y0)
{
    d->x.pos = x0; d->x.vel = 0;
    d->y.pos = y0; d->y.vel = 0;
    d->initialized = 1;
}

static void damp2_step_channel(damp_channel_t *ch, float target, float omega, float dt)
{
    float w2 = omega * omega;
    float acc = w2 * (target - ch->pos) - 2.0f * omega * ch->vel;
    ch->vel += acc * dt;
    ch->pos += ch->vel * dt;
}

static void damp2_step(damp2_t *d, float tx, float ty, float dt)
{
    if (!d->initialized) {
        damp2_start(d, tx, ty);
        return;
    }
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.1f) dt = 0.1f;
    damp2_step_channel(&d->x, tx, DAMP_OMEGA, dt);
    damp2_step_channel(&d->y, ty, DAMP_OMEGA, dt);
}

/* ═══════════════════════════════════════
 * 状态机
 * ═══════════════════════════════════════ */

static float sm_confidence(const tracker_t *trk)
{
    if (trk->history_count == 0) return 0;
    int hits = 0;
    int n = trk->history_count < SM_WINDOW_SIZE ? trk->history_count : SM_WINDOW_SIZE;
    for (int i = 0; i < n; i++)
        hits += trk->match_history[i];
    return (float)hits / (float)n;
}

static void sm_push(tracker_t *trk, int hit)
{
    trk->match_history[trk->history_idx] = hit;
    trk->history_idx = (trk->history_idx + 1) % SM_WINDOW_SIZE;
    if (trk->history_count < SM_WINDOW_SIZE) trk->history_count++;
}

/* ═══════════════════════════════════════
 * ISP 控制
 * ═══════════════════════════════════════ */

static int current_xoff = ISP_DEFAULT_XOFF;
static int current_yoff = ISP_DEFAULT_YOFF;
static int last_written_xoff = -1;
static int last_written_yoff = -1;
static int isp_written_this_frame = 0;
static uint8_t saved_5001 = 0xA3;

void tracker_write_isp_offset(int x, int y)
{
    current_xoff = x;
    current_yoff = y;

    if (isp_written_this_frame) return;
    if (x == last_written_xoff && y == last_written_yoff) return;

    ov5640_write_reg(0x3212, 0x03);
    ov5640_write_reg(0x3810, (x >> 8) & 0x0F);
    ov5640_write_reg(0x3811, x & 0xFF);
    ov5640_write_reg(0x3812, (y >> 8) & 0x07);
    ov5640_write_reg(0x3813, y & 0xFF);
    ov5640_write_reg(0x3212, 0x13);
    ov5640_write_reg(0x3212, 0xA3);

    last_written_xoff = x;
    last_written_yoff = y;
    isp_written_this_frame = 1;
}

void tracker_reset_frame_flag(void)
{
    isp_written_this_frame = 0;
}

void tracker_get_isp_offset(int *xoff, int *yoff)
{
    *xoff = current_xoff;
    *yoff = current_yoff;
}

static void scaler_restore(void) {
    if (!scaler_disabled) return;
    ov5640_write_reg(0x5001, saved_5001);
    scaler_disabled = 0;
}

static void scaler_disable(void) {
    if (scaler_disabled) return;
    ov5640_read_reg(0x5001, &saved_5001);
    ov5640_write_reg(0x5001, saved_5001 & ~0x20);
    scaler_disabled = 1;
}

/* ═══════════════════════════════════════
 * 公开接口
 * ═══════════════════════════════════════ */

void tracker_init(tracker_t *trk)
{
    memset(trk, 0, sizeof(*trk));
    trk->state = TRACK_STATE_IDLE;
    trk->target_id = -1;
    trk->target_class = -1;
    trk->select_valid = 0;
    ab_init(&trk->est);
    damp2_init(&trk->damp);
    g_state.pred_box.valid = 0;
    current_xoff = ISP_DEFAULT_XOFF;
    current_yoff = ISP_DEFAULT_YOFF;
    last_written_xoff = -1;
    last_written_yoff = -1;
    printf("[tracker] Init (α-β + damp2)\n");
}

void tracker_handle_cmd(tracker_t *trk)
{
    pthread_mutex_lock(&g_state.cmd_mutex);
    track_cmd_t cmd = g_state.track_cmd;
    int tid = g_state.track_target_id;
    g_state.track_cmd = TRACK_CMD_NONE;
    pthread_mutex_unlock(&g_state.cmd_mutex);
    if (cmd == TRACK_CMD_NONE) return;

    switch (cmd) {
    case TRACK_CMD_SELECT: {
        trk->target_id = tid;
        trk->select_valid = 0;

        printf("[tracker] SELECT id=%d\n", tid);

        pthread_mutex_lock(&g_state.det_mutex);
        for (int i = 0; i < g_state.det_count; i++) {
            if (g_state.detections[i].track_id == tid) {
                trk->target_class = g_state.detections[i].class_id;

                float cx, cy;
                det_to_cxcy(&g_state.detections[i], &cx, &cy);

                /* scaler ON: screen 坐标 → sub 坐标 */
                screen_to_sub_scaled(cx, cy,
                                     &trk->select_sub_x, &trk->select_sub_y);

                trk->select_bbox_w = g_state.detections[i].bbox[2];
                trk->select_bbox_h = g_state.detections[i].bbox[3];
                trk->select_valid = 1;

                /* 同步到 last_sub 用于匹配 */
                trk->last_sub_x = trk->select_sub_x;
                trk->last_sub_y = trk->select_sub_y;

                printf("[tracker] Cached: class=%d screen=(%.3f,%.3f) "
                       "sub=(%.1f,%.1f) bbox=(%.3f,%.3f)\n",
                       trk->target_class, cx, cy,
                       trk->select_sub_x, trk->select_sub_y,
                       trk->select_bbox_w, trk->select_bbox_h);
                break;
            }
        }
        pthread_mutex_unlock(&g_state.det_mutex);

        if (!trk->select_valid) {
            printf("[tracker] SELECT failed: id=%d not found in detections\n", tid);
            trk->target_id = -1;
        }
        break;
    }

    case TRACK_CMD_START: {
        if (!trk->select_valid || trk->target_id < 0) {
            printf("[tracker] Cannot start: no valid selection\n");
            break;
        }

        /* 用缓存的坐标计算 offset */
        int init_ox, init_oy;
        sub_to_offset(trk->select_sub_x, trk->select_sub_y, &init_ox, &init_oy);

        printf("[tracker] START: cached_sub=(%.1f,%.1f) → off=(%d,%d)\n",
               trk->select_sub_x, trk->select_sub_y, init_ox, init_oy);

        /* 禁用 scaler */
        scaler_disable();

        /* 写入 offset */
        last_written_xoff = -1;
        last_written_yoff = -1;
        isp_written_this_frame = 0;
        tracker_write_isp_offset(init_ox, init_oy);

        /* 初始化估计层 */
        ab_start(&trk->est, (float)init_ox, (float)init_oy);

        /* 初始化执行层 */
        damp2_start(&trk->damp, (float)init_ox, (float)init_oy);

        /* 状态 */
        trk->state = TRACK_STATE_TRACKING;
        trk->last_predict_time = 0;
        trk->startup_frames = STARTUP_SKIP_FRAMES;
        trk->history_idx = 0;
        trk->history_count = 0;
        memset(trk->match_history, 0, sizeof(trk->match_history));

        /* last_sub 已在 SELECT 时设置 */

        g_state.tracking_active = 1;
        g_state.tracking_target_id = trk->target_id;

        printf("[tracker] Started: class=%d off=(%d,%d)\n",
               trk->target_class, init_ox, init_oy);
        break;
    }

    case TRACK_CMD_STOP:
        tracker_stop(trk);
        printf("[tracker] Stopped by user\n");
        break;

    default:
        break;
    }
}

void tracker_update_detections(tracker_t *trk,
                               detection_t *dets, int det_count,
                               int snap_xoff, int snap_yoff,
                               float time_sec)
{
    for (int i = 0; i < det_count; i++) dets[i].is_target = 0;
    if (trk->state == TRACK_STATE_IDLE) return;
    if (trk->target_id < 0) return;

    /* 启动阶段跳过: scaler 切换后前几帧数据不可靠 */
    if (trk->startup_frames > 0) {
        printf("[trk] startup skip (remaining=%d)\n", trk->startup_frames);
        return;
    }

    /* 匹配 */
    int best = -1;
    float best_d = 999999.0f;

    for (int i = 0; i < det_count; i++) {
        if (trk->target_class >= 0 && dets[i].class_id != trk->target_class)
            continue;

        float cx, cy;
        det_to_cxcy(&dets[i], &cx, &cy);

        float sub_x, sub_y;
        screen_to_sub(cx, cy, snap_xoff, snap_yoff, &sub_x, &sub_y);

        float dx = sub_x - trk->last_sub_x;
        float dy = sub_y - trk->last_sub_y;
        float d = sqrtf(dx*dx + dy*dy);

        if (d < best_d && d < MATCH_DIST_PX) {
            best_d = d;
            best = i;
        }
    }

    if (best >= 0) {
        dets[best].is_target = 1;
        trk->target_id = dets[best].track_id;

        float cx, cy;
        det_to_cxcy(&dets[best], &cx, &cy);

        float sub_x, sub_y;
        screen_to_sub(cx, cy, snap_xoff, snap_yoff, &sub_x, &sub_y);

        trk->last_sub_x = sub_x;
        trk->last_sub_y = sub_y;

        int obs_ox, obs_oy;
        sub_to_offset(sub_x, sub_y, &obs_ox, &obs_oy);

        ab_update(&trk->est, (float)obs_ox, (float)obs_oy);

        trk->last_seen_time = time_sec;
        sm_push(trk, 1);

        printf("[trk] match sub=(%.0f,%.0f) off=(%d,%d) "
               "est=(%.0f,%.0f) vel=(%.1f,%.1f)\n",
               sub_x, sub_y, obs_ox, obs_oy,
               trk->est.ox, trk->est.oy,
               trk->est.vx, trk->est.vy);

    } else {
        sm_push(trk, 0);
        if (det_count > 0) {
            printf("[trk] no match: %d dets, best_d=%.0f (thresh=%.0f)\n",
                   det_count, best_d, MATCH_DIST_PX);
        }
    }

    /* 状态机 */
    float conf = sm_confidence(trk);

    switch (trk->state) {
    case TRACK_STATE_TRACKING:
        if (conf < SM_CONF_COAST) {
            trk->state = TRACK_STATE_COASTING;
            trk->lost_time = time_sec;
            printf("[tracker] → COASTING conf=%.2f\n", conf);
        }
        break;

    case TRACK_STATE_COASTING:
        if (conf >= SM_CONF_TRACK) {
            trk->state = TRACK_STATE_TRACKING;
            printf("[tracker] → TRACKING conf=%.2f\n", conf);
        } else if (time_sec - trk->lost_time > SM_LOST_TIMEOUT) {
            trk->state = TRACK_STATE_LOST;
            trk->lost_time = time_sec;
            printf("[tracker] → LOST\n");
        }
        break;

    case TRACK_STATE_LOST:
        if (conf >= SM_CONF_TRACK) {
            trk->state = TRACK_STATE_TRACKING;
            printf("[tracker] → TRACKING (recovered)\n");
        } else if (time_sec - trk->lost_time > SM_LOST_TIMEOUT) {
            printf("[tracker] Lost timeout\n");
            tracker_stop(trk);
        }
        break;

    default:
        break;
    }
}

void tracker_predict(tracker_t *trk, float time_sec)
{
    if (trk->state == TRACK_STATE_IDLE) {
        g_state.pred_box.valid = 0;
        return;
    }

    if (trk->startup_frames > 0) {
        trk->startup_frames--;
        trk->last_predict_time = time_sec;
        g_state.pred_box.valid = 0;
        return;
    }

    float dt;
    if (trk->last_predict_time <= 0) {
        dt = 1.0f / 30.0f;
    } else {
        dt = time_sec - trk->last_predict_time;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.1f) dt = 0.1f;
    }
    trk->last_predict_time = time_sec;

    /* 估计层预测 */
    ab_predict(&trk->est, dt);

    /* 执行层平滑 */
    damp2_step(&trk->damp, trk->est.ox, trk->est.oy, dt);

    /* 写入 ISP */
    int ox = clampi((int)(trk->damp.x.pos + 0.5f), OX_MIN, OX_MAX);
    int oy = clampi((int)(trk->damp.y.pos + 0.5f), OY_MIN, OY_MAX);
    tracker_write_isp_offset(ox, oy);

    /* ══ 计算预测框在当前画面中的位置 ══
     *
     * 估计器认为"使目标居中"的 offset = (est.ox, est.oy)
     * 即目标在 sub 空间的位置 = (est.ox + 320, est.oy + 240)
     *
     * 当前画面显示的 sub 区域起点 = (smooth_ox, smooth_oy)
     *
     * 目标在画面中的位置（翻转前）:
     *   raw_cx = (target_sub_x - view_ox) / 640
     *   raw_cy = (target_sub_y - view_oy) / 480
     *
     * 翻转回前端坐标（与 screen_to_sub 中的翻转对应）:
     *   scr_cx = 1 - raw_cx
     *   scr_cy = 1 - raw_cy
     */
    {
        float target_sub_x = trk->est.ox + (float)(IMG_W / 2);
        float target_sub_y = trk->est.oy + (float)(IMG_H / 2);

        float view_ox = trk->damp.x.pos;
        float view_oy = trk->damp.y.pos;

        float raw_cx = (target_sub_x - view_ox) / (float)IMG_W;
        float raw_cy = (target_sub_y - view_oy) / (float)IMG_H;

        g_state.pred_box.cx = 1.0f - raw_cx;
        g_state.pred_box.cy = 1.0f - raw_cy;
        g_state.pred_box.w  = trk->select_bbox_w;
        g_state.pred_box.h  = trk->select_bbox_h;
        g_state.pred_box.class_id = trk->target_class;
        g_state.pred_box.track_id = trk->target_id;
        g_state.pred_box.valid = 1;
    }

    /* 每 2 秒诊断 */
    static float last_diag = 0;
    if (time_sec - last_diag >= 2.0f) {
        last_diag = time_sec;
        printf("[tracker] st=%d est=(%.0f,%.0f) vel=(%.1f,%.1f) "
               "smooth=(%.0f,%.0f) off=(%d,%d) pred=(%.2f,%.2f) conf=%.2f\n",
               trk->state,
               trk->est.ox, trk->est.oy,
               trk->est.vx, trk->est.vy,
               trk->damp.x.pos, trk->damp.y.pos,
               ox, oy,
               g_state.pred_box.cx, g_state.pred_box.cy,
               sm_confidence(trk));
    }
}

int tracker_is_active(const tracker_t *trk)
{
    return trk->state != TRACK_STATE_IDLE;
}

int tracker_get_target_id(const tracker_t *trk)
{
    return trk->target_id;
}

void tracker_stop(tracker_t *trk)
{
    trk->state = TRACK_STATE_IDLE;
    trk->target_id = -1;
    trk->target_class = -1;
    trk->select_valid = 0;
    ab_init(&trk->est);
    damp2_init(&trk->damp);

    scaler_restore();

    current_xoff = ISP_DEFAULT_XOFF;
    current_yoff = ISP_DEFAULT_YOFF;
    last_written_xoff = -1;
    isp_written_this_frame = 0;
    tracker_write_isp_offset(ISP_DEFAULT_XOFF, ISP_DEFAULT_YOFF);

    g_state.pred_box.valid = 0;

    g_state.tracking_active = 0;
    g_state.tracking_target_id = -1;

    printf("[tracker] Stopped, full view restored\n");
}

void tracker_tune(tracker_t *trk, const char *key, float val)
{
    (void)trk;
    printf("[tune] %s=%.4f (fixed params)\n", key, val);
}
