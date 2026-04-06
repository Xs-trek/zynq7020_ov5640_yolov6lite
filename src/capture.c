#include "capture.h"
#include "hal_frmbuf.h"
#include "hal_ov5640.h"
#include "jpeg_enc.h"
#include "optical_flow.h"
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

        /* warmup 期间保持原有“跳过后续处理”的语义 */
        if (warmup > 0) {
            warmup--;
            write_idx = (write_idx + 1) % FRAME_BUF_CNT;
            continue;
        }

        /* 提前获取并发布本帧图像，供光流和 YOLO 使用 */
        int completed_idx = write_idx;
        memcpy(g_cached_pool[completed_idx], g_frame_pool[completed_idx], FRAME_SIZE);
        __atomic_store_n(&g_state.latest_frame_idx, completed_idx, __ATOMIC_RELEASE);
        __atomic_add_fetch(&g_state.frame_seq, 1, __ATOMIC_RELEASE);

        if (g_state.tracking_active && g_state.tracking_target_id >= 0) {
            static unsigned int last_processed_det_seq = 0;

            unsigned int current_det_seq = 0;
            int has_new_yolo = 0;
            float yolo_abs_x = 0.0f, yolo_abs_y = 0.0f;
            float yolo_w = 0.0f, yolo_h = 0.0f;

            pthread_mutex_lock(&g_state.det_mutex);
            current_det_seq = g_state.det_seq;

            if (current_det_seq != last_processed_det_seq) {
                last_processed_det_seq = current_det_seq;

                for (int i = 0; i < g_state.det_count; i++) {
                    if (g_state.detections[i].is_target == 1) {
                        /* last_sub_x/y 在当前架构下仍保持 YOLO 真值语义 */
                        yolo_abs_x = g_tracker.last_sub_x;
                        yolo_abs_y = g_tracker.last_sub_y;
                        yolo_w = g_state.detections[i].bbox[2];
                        yolo_h = g_state.detections[i].bbox[3];
                        has_new_yolo = 1;
                        break;
                    }
                }
            }

            pthread_mutex_unlock(&g_state.det_mutex);

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
                    __atomic_load_n(&g_state.frame_seq, __ATOMIC_ACQUIRE);

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

        if (test_is_active()) {
            test_predict(now);
        } else if (g_state.tracking_active) {
            tracker_predict(&g_tracker, now);
        }

        jpeg_encode_frame(g_cached_pool[completed_idx]);

        write_idx = (write_idx + 1) % FRAME_BUF_CNT;
        if (write_idx == g_state.latest_frame_idx)
            write_idx = (write_idx + 1) % FRAME_BUF_CNT;
    }

    printf("[capture] Thread exiting\n");
    return NULL;
}
