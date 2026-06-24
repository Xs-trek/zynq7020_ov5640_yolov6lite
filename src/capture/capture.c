#include "capture/capture.h"
#include "platform/hal_frmbuf.h"
#include "platform/hal_ov5640.h"
#include "media/jpeg_enc.h"
#include "vision/optical_flow.h"
#include "cam_system/tracker.h"
#include "cam_system/app_state.h"
#include "cam_system/config.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

extern tracker_t g_tracker;

static float get_time_sec(void)
{
    static double t0 = 0;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double now = (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
    if (t0 == 0) t0 = now;
    return (float)(now - t0);  /* 相对时间，float 精度足够表示数天内的时间差 */
}

static void notify_status_changed(void)
{
    app_state_inc_u32(&g_state.status_seq);
}

static int capture_pop_control_cmd(track_cmd_t *cmd, int *target_id)
{
    pthread_mutex_lock(&g_state.cmd_mutex);

    if (g_state.cmd_count > 0) {
        int idx = g_state.cmd_head;
        *cmd = g_state.cmd_queue[idx];
        *target_id = g_state.cmd_target_queue[idx];
        g_state.cmd_head = (idx + 1) % CONTROL_CMD_QUEUE_LEN;
        g_state.cmd_count--;
    } else {
        *cmd = g_state.track_cmd;
        *target_id = g_state.track_target_id;
        g_state.track_cmd = TRACK_CMD_NONE;
    }

    pthread_mutex_unlock(&g_state.cmd_mutex);
    return *cmd != TRACK_CMD_NONE;
}

static void capture_handle_control_cmd(void)
{
    track_cmd_t cmd = TRACK_CMD_NONE;
    int tid = -1;

    if (!capture_pop_control_cmd(&cmd, &tid)) return;

    tracker_apply_cmd(&g_tracker, cmd, tid);
}

void *capture_thread(void *arg)
{
    (void)arg;
    printf("[capture] Thread started\n");

    int write_idx = 0;
    int warmup = 0;
    int was_enabled = 0;
    unsigned int last_processed_det_seq = 0;

    while (!app_state_load_int(&g_state.quit)) {

        if (!app_state_load_int(&g_state.cam_enabled)) {
            capture_handle_control_cmd();
            was_enabled = 0;
            usleep(100000);
            continue;
        }

        if (!was_enabled) {
            was_enabled = 1;
            warmup = 5;
            write_idx = 0;
            printf("[capture] Camera enabled, warming up\n");
        }

        /* 记录本帧采集前的 ISP offset 快照 */
        int snap_x, snap_y;
        tracker_get_isp_offset(&snap_x, &snap_y);

        int completed_idx = frmbuf_capture_frame(write_idx, 2000);
        if (completed_idx < 0) {
            fprintf(stderr, "[capture] Frame timeout\n");
            continue;
        }

        /* ══ 帧间隙操作 ══ */
        float now = get_time_sec();

        tracker_reset_frame_flag();  /* 每帧重置写入标志 */
        capture_handle_control_cmd();

        /* warmup 期间保持原有“跳过后续处理”的语义 */
        if (warmup > 0) {
            warmup--;
            if (frmbuf_release_frame(completed_idx) < 0) {
                fprintf(stderr, "[capture] Failed to release warmup frame %d\n",
                        completed_idx);
                app_state_store_int(&g_state.quit, 1);
                break;
            }
            write_idx = (completed_idx + 1) % FRAME_BUF_CNT;
            continue;
        }

        /* 提前获取并发布本帧图像，供光流和 YOLO 使用 */
        pthread_mutex_lock(&g_state.frame_mutex);
        g_state.frame_offsets[completed_idx].xoff = snap_x;
        g_state.frame_offsets[completed_idx].yoff = snap_y;
        memcpy(g_cached_pool[completed_idx], g_frame_pool[completed_idx], FRAME_SIZE);
        app_state_store_int(&g_state.latest_frame_idx, completed_idx);
        app_state_inc_u32(&g_state.frame_seq);
        pthread_mutex_unlock(&g_state.frame_mutex);

        if (frmbuf_release_frame(completed_idx) < 0) {
            fprintf(stderr, "[capture] Failed to release frame %d\n",
                    completed_idx);
            app_state_store_int(&g_state.quit, 1);
            break;
        }

        unsigned int current_det_seq = 0;
        int have_new_det = 0;
        int det_count = 0;
        int det_xoff = 0;
        int det_yoff = 0;
        detection_t det_snapshot[YOLO_MAX_DET];
        int has_new_yolo = 0;
        float yolo_abs_x = 0.0f, yolo_abs_y = 0.0f;
        float yolo_w = 0.0f, yolo_h = 0.0f;

        pthread_mutex_lock(&g_state.det_mutex);
        current_det_seq = app_state_load_u32(&g_state.det_seq);

        if (current_det_seq != 0 && current_det_seq != last_processed_det_seq) {
            last_processed_det_seq = current_det_seq;
            have_new_det = 1;
            det_count = g_state.det_count;
            if (det_count < 0) det_count = 0;
            if (det_count > YOLO_MAX_DET) det_count = YOLO_MAX_DET;
            det_xoff = g_state.det_frame_xoff;
            det_yoff = g_state.det_frame_yoff;
            memcpy(det_snapshot, g_state.detections,
                   (size_t)det_count * sizeof(detection_t));
        }

        pthread_mutex_unlock(&g_state.det_mutex);

        if (have_new_det) {
            tracker_update_detections(&g_tracker,
                                      det_snapshot,
                                      det_count,
                                      det_xoff,
                                      det_yoff,
                                      now);
            int tracking_active = tracker_is_active(&g_tracker);
            int tracking_target_id = tracker_get_target_id(&g_tracker);

            if (tracking_active && tracking_target_id >= 0) {
                for (int i = 0; i < det_count; i++) {
                    if (det_snapshot[i].is_target == 1) {
                        /* last_sub_x/y 在当前架构下仍保持 YOLO 真值语义 */
                        yolo_abs_x = g_tracker.last_sub_x;
                        yolo_abs_y = g_tracker.last_sub_y;
                        yolo_w = det_snapshot[i].bbox[2];
                        yolo_h = det_snapshot[i].bbox[3];
                        has_new_yolo = 1;
                        break;
                    }
                }
            }

            pthread_mutex_lock(&g_state.det_mutex);
            int prev_tracking_active = app_state_load_int(&g_state.tracking_active);
            int prev_tracking_target_id = app_state_load_int(&g_state.tracking_target_id);
            app_state_store_int(&g_state.tracking_active, tracking_active);
            app_state_store_int(&g_state.tracking_target_id, tracking_target_id);
            if (app_state_load_u32(&g_state.det_seq) == current_det_seq) {
                memcpy(g_state.detections, det_snapshot,
                       (size_t)det_count * sizeof(detection_t));
                g_state.det_count = det_count;
            }
            pthread_mutex_unlock(&g_state.det_mutex);

            if (prev_tracking_active != tracking_active ||
                prev_tracking_target_id != tracking_target_id)
                notify_status_changed();
        }

        int tracking_active_now = app_state_load_int(&g_state.tracking_active);
        int tracking_target_id_now = app_state_load_int(&g_state.tracking_target_id);

        if (tracking_active_now && tracking_target_id_now >= 0) {
            /* YOLO 新结果到来时，用它校准/重建光流 */
            if (has_new_yolo) {
                of_calibrate(current_det_seq,
                             yolo_abs_x, yolo_abs_y,
                             yolo_w, yolo_h,
                             g_cached_pool[completed_idx],
                             snap_x, snap_y);
            }

            /* 如果本帧刚做过 YOLO 校准，则不再继续 of_track，
             * 避免“同帧校准 + 同帧高频观测”叠加引发抖动 */
            if (!has_new_yolo && of_is_tracking()) {
                float abs_x, abs_y;
                unsigned int current_frame_seq =
                    app_state_load_u32(&g_state.frame_seq);

                if (of_track(g_cached_pool[completed_idx],
                             current_frame_seq,
                             snap_x, snap_y,
                             &abs_x, &abs_y)) {
                    now = get_time_sec();
                    tracker_update_optical_flow(&g_tracker, abs_x, abs_y, now);
                }
            }
        } else {
            of_init();
        }

        if (app_state_load_int(&g_state.tracking_active)) {
            tracker_predict(&g_tracker, now);
        }

        jpeg_encode_frame(g_cached_pool[completed_idx]);

        write_idx = (completed_idx + 1) % FRAME_BUF_CNT;
        if (write_idx == app_state_load_int(&g_state.latest_frame_idx))
            write_idx = (write_idx + 1) % FRAME_BUF_CNT;
    }

    printf("[capture] Thread exiting\n");
    return NULL;
}
