/* test.c — ISP 平移测试 v3
 *
 * 核心发现：
 *   scaler 启用 (0x5001[5]=1): offset = 对称裁剪 → 变焦
 *   scaler 禁用 (0x5001[5]=0): offset = 单侧起始 → 平移
 *
 * 测试方法：
 *   Phase 1 (0-3s): 禁用 scaler + 静态偏移，验证 pipeline
 *   Phase 2 (3s+):  连续平移轨迹
 */
#include "test.h"
#include "tracker.h"
#include "hal_ov5640.h"
#include "config.h"
#include <stdio.h>
#include <string.h>

/* ═══════════════════════════════════════
 * 平移范围 (scaler 禁用, subsample 2x 后)
 * ISP input = 1312×972, output = 640×480
 * ═══════════════════════════════════════ */

#define PAN_MAX_X   (SUBSAMPLED_1X_W - IMG_W)   /* 672 */
#define PAN_MAX_Y   (SUBSAMPLED_1X_H - IMG_H)   /* 492 */

/* 留安全边距 */
#define PAN_MARGIN  16

#define PAN_LEFT    PAN_MARGIN                          /* 16 */
#define PAN_RIGHT   (PAN_MAX_X - PAN_MARGIN)            /* 656 */
#define PAN_TOP     PAN_MARGIN                          /* 16 */
#define PAN_BOTTOM  (PAN_MAX_Y - PAN_MARGIN)            /* 476 */
#define PAN_CX      (PAN_MAX_X / 2)                     /* 336 */
#define PAN_CY      (PAN_MAX_Y / 2)                     /* 246 */

/* 5 个航点 (offset_x, offset_y) */
static const int waypoints[][2] = {
    { PAN_CX,    PAN_CY     },   /* 0: 中心 */
    { PAN_RIGHT, PAN_TOP    },   /* 1: 右上 */
    { PAN_LEFT,  PAN_TOP    },   /* 2: 左上 */
    { PAN_RIGHT, PAN_BOTTOM },   /* 3: 右下 */
    { PAN_LEFT,  PAN_BOTTOM },   /* 4: 左下 */
};

/* ═══════════════════════════════════════
 * 状态
 * ═══════════════════════════════════════ */

static test_mode_t g_test_mode = TEST_MODE_NONE;
static int g_test_active = 0;
static float g_test_start_time = 0.0f;
static float g_last_print_time = 0.0f;
static float g_last_call_time = 0.0f;

static int g_diag_writes = 0;
static int g_diag_cycles = 0;
static int g_diag_calls = 0;
static float g_diag_max_gap = 0.0f;
static float g_diag_min_gap = 999.0f;

static int g_phase1_done = 0;
static uint8_t g_saved_5001 = 0xA3;  /* 原始 0x5001 值 */

/* 上次写入值 */
static int g_last_ox = -1;
static int g_last_oy = -1;

/* ═══════════════════════════════════════
 * 辅助
 * ═══════════════════════════════════════ */

static int clampi(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 写 offset（scaler 禁用时 = 单侧平移起始位置） */
static void write_pan_offset(int ox, int oy)
{
    if (ox == g_last_ox && oy == g_last_oy) return;

    ov5640_write_reg(0x3212, 0x03);
    ov5640_write_reg(0x3810, (ox >> 8) & 0x0F);
    ov5640_write_reg(0x3811, ox & 0xFF);
    ov5640_write_reg(0x3812, (oy >> 8) & 0x07);
    ov5640_write_reg(0x3813, oy & 0xFF);
    ov5640_write_reg(0x3212, 0x13);
    ov5640_write_reg(0x3212, 0xA3);

    g_last_ox = ox;
    g_last_oy = oy;
    g_diag_writes++;
}

/* ═══════════════════════════════════════
 * 公开接口
 * ═══════════════════════════════════════ */

void test_init(void)
{
    g_test_mode = TEST_MODE_NONE;
    g_test_active = 0;
    g_test_start_time = 0.0f;
    g_last_print_time = 0.0f;
    g_last_call_time = 0.0f;
    g_diag_writes = 0;
    g_diag_cycles = 0;
    g_diag_calls = 0;
    g_diag_max_gap = 0.0f;
    g_diag_min_gap = 999.0f;
    g_phase1_done = 0;
    g_last_ox = -1;
    g_last_oy = -1;
    printf("[test] Initialized\n");
}

void test_start(test_mode_t mode)
{
    if (mode == TEST_MODE_NONE) {
        test_stop();
        return;
    }

    g_test_mode = mode;
    g_test_active = 1;
    g_test_start_time = 0.0f;
    g_last_print_time = 0.0f;
    g_last_call_time = 0.0f;
    g_diag_writes = 0;
    g_diag_cycles = 0;
    g_diag_calls = 0;
    g_diag_max_gap = 0.0f;
    g_diag_min_gap = 999.0f;
    g_phase1_done = 0;
    g_last_ox = -1;
    g_last_oy = -1;

    /* 保存并禁用 scaler */
    ov5640_read_reg(0x5001, &g_saved_5001);
    uint8_t new_5001 = g_saved_5001 & ~0x20;
    ov5640_write_reg(0x5001, new_5001);

    uint8_t readback;
    ov5640_read_reg(0x5001, &readback);

    printf("[test] === SCALER-OFF PAN TEST ===\n");
    printf("[test] 0x5001: 0x%02X → 0x%02X (readback: 0x%02X)\n",
           g_saved_5001, new_5001, readback);
    printf("[test] Scaler %s\n", (readback & 0x20) ? "STILL ON!" : "disabled OK");
    printf("[test] Mode: offset = single-side start position (per fig 4-2)\n");
    printf("[test] Pan range: X=[0,%d] Y=[0,%d]\n", PAN_MAX_X, PAN_MAX_Y);
    printf("[test] Phase 1 (0-3s): static offset (656,476), verify image\n");
    printf("[test] Phase 2 (3s+):  10s cycle trajectory\n");
}

void test_stop(void)
{
    if (g_test_active) {
        printf("[test] Stopped. calls=%d writes=%d cycles=%d\n",
               g_diag_calls, g_diag_writes, g_diag_cycles);
        if (g_diag_min_gap < 900.0f)
            printf("[test] Frame gap: min=%.1fms max=%.1fms\n",
                   g_diag_min_gap * 1000.0f, g_diag_max_gap * 1000.0f);
    }

    g_test_active = 0;
    g_test_mode = TEST_MODE_NONE;

    /* 恢复 scaler */
    ov5640_write_reg(0x5001, g_saved_5001);
    printf("[test] 0x5001 restored to 0x%02X\n", g_saved_5001);

    /* 恢复默认 offset */
    g_last_ox = -1;
    g_last_oy = -1;
    write_pan_offset(ISP_DEFAULT_XOFF, ISP_DEFAULT_YOFF);

    /* 同步 tracker 状态 */
    tracker_write_isp_offset(ISP_DEFAULT_XOFF, ISP_DEFAULT_YOFF);
    printf("[test] Restored to default\n");
}

int test_is_active(void)
{
    return g_test_active;
}

test_mode_t test_get_mode(void)
{
    return g_test_mode;
}

void test_get_current_offset(int *xoff, int *yoff)
{
    if (xoff) *xoff = (g_last_ox >= 0) ? g_last_ox : ISP_DEFAULT_XOFF;
    if (yoff) *yoff = (g_last_oy >= 0) ? g_last_oy : ISP_DEFAULT_YOFF;
}

void test_predict(float time_sec)
{
    if (!g_test_active) return;
    if (g_test_mode != TEST_MODE_ISP_PATH) return;

    g_diag_calls++;

    if (g_last_call_time > 0.0f) {
        float gap = time_sec - g_last_call_time;
        if (gap > g_diag_max_gap) g_diag_max_gap = gap;
        if (gap < g_diag_min_gap) g_diag_min_gap = gap;
    }
    g_last_call_time = time_sec;

    if (g_test_start_time <= 0.0f)
        g_test_start_time = time_sec;

    float elapsed = time_sec - g_test_start_time;

    /* ════════════════════════════════
     * Phase 1: 0-3s 静态验证
     * ════════════════════════════════ */
    if (elapsed < 3.0f) {
        if (g_last_ox < 0) {
            /* 设置到右下角 */
            int ox = PAN_RIGHT;  /* 656 */
            int oy = PAN_BOTTOM; /* 476 */
            write_pan_offset(ox, oy);
            printf("[test] Phase 1: offset=(%d,%d)\n", ox, oy);
            printf("[test]   Shows pixels (%d,%d)-(%d,%d) of 1312x972\n",
                   ox, oy, ox + IMG_W - 1, oy + IMG_H - 1);
            printf("[test]   Expected: bottom-right area, ~2x zoom, no scaling\n");
        }

        if (time_sec - g_last_print_time >= 1.0f) {
            g_last_print_time = time_sec;
            printf("[test] Phase 1: t=%.1fs writes=%d\n", elapsed, g_diag_writes);
        }
        return;
    }

    if (!g_phase1_done) {
        g_phase1_done = 1;
        printf("[test] Phase 2 starting: continuous pan\n");
    }

    /* ════════════════════════════════
     * Phase 2: 连续平移
     * ════════════════════════════════ */

    float pan_t = elapsed - 3.0f;
    const float cycle = 10.0f;
    const float seg_t = 2.0f;

    int cycle_num = (int)(pan_t / cycle);
    if (cycle_num > g_diag_cycles) {
        g_diag_cycles = cycle_num;
        printf("[test] === Cycle %d done. gap: min=%.1fms max=%.1fms ===\n",
               cycle_num,
               g_diag_min_gap * 1000.0f, g_diag_max_gap * 1000.0f);
        g_diag_max_gap = 0.0f;
        g_diag_min_gap = 999.0f;
    }

    while (pan_t >= cycle) pan_t -= cycle;
    if (pan_t < 0) pan_t = 0;

    int seg = (int)(pan_t / seg_t);
    if (seg > 4) seg = 4;

    float frac = (pan_t - seg * seg_t) / seg_t;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;

    int from = seg;
    int to = (seg + 1) % 5;

    int ox = waypoints[from][0] + (int)((waypoints[to][0] - waypoints[from][0]) * frac);
    int oy = waypoints[from][1] + (int)((waypoints[to][1] - waypoints[from][1]) * frac);

    ox = clampi(ox, 0, PAN_MAX_X);
    oy = clampi(oy, 0, PAN_MAX_Y);

    write_pan_offset(ox, oy);

    if (time_sec - g_last_print_time >= 0.5f) {
        g_last_print_time = time_sec;

        static const char *names[] = {
            "center->TR", "TR->TL", "TL->BR", "BR->BL", "BL->center"
        };

        printf("[pan] t=%.1fs seg=%d(%s) frac=%.0f%% off=(%d,%d) "
               "view=(%d,%d)-(%d,%d) wr=%d\n",
               pan_t, seg, names[seg], frac * 100.0f,
               ox, oy,
               ox, oy, ox + IMG_W - 1, oy + IMG_H - 1,
               g_diag_writes);
    }
}
