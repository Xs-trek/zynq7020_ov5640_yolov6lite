#include "capture.h"
#include "hal_frmbuf.h"
#include "hal_ov5640.h"
#include "jpeg_enc.h"
#include "tracker.h"
#include "app_state.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include "test.h"

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

void *capture_thread(void *arg)
{
    (void)arg;
    printf("[capture] Thread started\n");

    int write_idx = 0;
    int warmup = 0;
    int was_enabled = 0;

    while (!g_state.quit) {

        if (!g_state.cam_enabled) {
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

        if (frmbuf_start_frame(write_idx) < 0) {
            usleep(10000);
            continue;
        }

        if (frmbuf_wait_done(2000) < 0) {
            fprintf(stderr, "[capture] Frame timeout\n");
            continue;
        }

        /* 保存快照到该帧缓冲对应的槽位 */
        g_state.frame_offsets[write_idx].xoff = snap_x;
        g_state.frame_offsets[write_idx].yoff = snap_y;

        /* ══ 帧间隙操作 ══ */
        float now = get_time_sec();
        
        tracker_reset_frame_flag();  /* 每帧重置写入标志 */
        tracker_handle_cmd(&g_tracker);

        if (test_is_active()) {
            test_predict(now);
        } else if (g_state.tracking_active) {
            tracker_predict(&g_tracker, now);
        }

        if (warmup > 0) {
            warmup--;
            write_idx = (write_idx + 1) % FRAME_BUF_CNT;
            continue;
        }

        int completed_idx = write_idx;
        g_state.latest_frame_idx = completed_idx;
        g_state.frame_seq++;

        jpeg_encode_frame(g_frame_pool[completed_idx]);

        write_idx = (write_idx + 1) % FRAME_BUF_CNT;
        if (write_idx == g_state.latest_frame_idx)
            write_idx = (write_idx + 1) % FRAME_BUF_CNT;
    }

    printf("[capture] Thread exiting\n");
    return NULL;
}
