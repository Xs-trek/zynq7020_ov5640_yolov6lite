/* main.cpp — cam_system 入口 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>

extern "C" {
#include "config.h"
#include "app_state.h"
#include "hal_mmio.h"
#include "hal_ov5640.h"
#include "hal_frmbuf.h"
#include "capture.h"
#include "jpeg_enc.h"
#include "net_server.h"
#include "tracker.h"
#include "test.h"
}
#include "yolo_detect.h"

/* ── 全局状态 ── */
app_state_t g_state;
uint8_t *g_frame_pool[FRAME_BUF_CNT];
uint8_t *g_cached_pool[FRAME_BUF_CNT];

/* 全局 tracker 实例 */
tracker_t g_tracker;

void app_state_init(app_state_t *s)
{
    memset(s, 0, sizeof(*s));
    s->cam_enabled = 0;
    s->yolo_enabled = 0;
    s->latest_frame_idx = -1;
    s->frame_seq = 0;
    s->zoom_level = 100;
    s->zoom_pending = 0;
    for (int i = 0; i < FRAME_BUF_CNT; i++) {
        s->frame_offsets[i].xoff = 16;
        s->frame_offsets[i].yoff = 4;
    }
    s->pred_box.valid = 0;
    s->tracking_active = 0;
    s->tracking_target_id = -1;
    s->quit = 0;

    pthread_mutex_init(&s->jpeg_mutex, NULL);
    pthread_mutex_init(&s->det_mutex, NULL);
    pthread_mutex_init(&s->cmd_mutex, NULL);
    pthread_mutex_init(&s->settings_mutex, NULL);
    pthread_mutex_init(&s->pred_mutex, NULL);

    s->jpeg_buf = NULL;
    s->jpeg_size = 0;
    s->det_count = 0;
    s->max_det = YOLO_MAX_DET;

    for (int i = 0; i < YOLO_NUM_CLASSES; i++)
        s->enabled_classes[i] = i;
    s->enabled_class_count = YOLO_NUM_CLASSES;
}

/* ── 信号处理 ── */
static void signal_handler(int sig)
{
    (void)sig;
    printf("\n[main] Signal %d, shutting down...\n", sig);
    g_state.quit = 1;
    net_server_stop();
}

/* ── CPU 亲和性 ── */
static void bind_to_core(int core)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
}

/* ── 主函数 ── */
int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    printf("============================================\n");
    printf("               cam_system\n");
    printf("============================================\n\n");

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGPIPE, SIG_IGN);

    /* 初始化全局状态 */
    app_state_init(&g_state);

    /* 初始化 /dev/mem */
    if (mmio_init() < 0) return 1;

    /* 初始化 OV5640 */
    if (ov5640_init() < 0) {
        fprintf(stderr, "[main] OV5640 init failed\n");
        return 1;
    }

    /* 初始化 frmbuf_wr */
    if (frmbuf_init() < 0) {
        fprintf(stderr, "[main] frmbuf init failed\n");
        return 1;
    }

    /* 映射帧缓冲到用户空间 */
    uint32_t fb_phys[FRAME_BUF_CNT] = {
        FRAME_BUF_0, FRAME_BUF_1, FRAME_BUF_2
    };
    for (int i = 0; i < FRAME_BUF_CNT; i++) {
        g_frame_pool[i] = (uint8_t *)mmio_map(fb_phys[i], FRAME_SIZE);
        if (!g_frame_pool[i]) {
            fprintf(stderr, "[main] Failed to map frame buffer %d\n", i);
            return 1;
        }
        g_cached_pool[i] = (uint8_t *)malloc(FRAME_SIZE);
        if (!g_cached_pool[i]) {
            fprintf(stderr, "[main] Failed to alloc cached pool %d\n", i);
            return 1;
        }
        printf("[main] Frame pool[%d] mapped: phys=0x%08X\n", i, fb_phys[i]);
    }

    /* 初始化 JPEG 编码器 */
    if (jpeg_init() < 0) {
        fprintf(stderr, "[main] JPEG init failed\n");
        return 1;
    }

    /* 初始化 tracker */
    tracker_init(&g_tracker);

    test_init();

    /* 启动采集线程 (Core 1) */
    pthread_t cap_tid;
    pthread_create(&cap_tid, NULL, capture_thread, NULL);
    {
        cpu_set_t cs;
        CPU_ZERO(&cs);
        CPU_SET(1, &cs);
        pthread_setaffinity_np(cap_tid, sizeof(cs), &cs);
    }

    /* 初始化并启动 YOLO 线程 (Core 0) */
    if (yolo_init() < 0) {
        fprintf(stderr, "[main] YOLO init failed, continuing without detection\n");
    }
    pthread_t yolo_tid;
    pthread_create(&yolo_tid, NULL, yolo_thread, NULL);
    {
        cpu_set_t cs;
        CPU_ZERO(&cs);
        CPU_SET(0, &cs);
        pthread_setaffinity_np(yolo_tid, sizeof(cs), &cs);
    }

    /* 主线程绑定 Core 1，运行 HTTP 事件循环 */
    bind_to_core(1);
    printf("[main] Starting HTTP server on port %d...\n\n", HTTP_PORT);
    net_server_run();  /* 阻塞直到 quit */

    /* ── 清理 ── */
    printf("[main] Shutting down...\n");

    pthread_join(cap_tid, NULL);
    pthread_join(yolo_tid, NULL);

    ov5640_standby(1);

    jpeg_close();
    frmbuf_close();
    ov5640_close();

    for (int i = 0; i < FRAME_BUF_CNT; i++) {
        if (g_frame_pool[i])
            mmio_unmap((volatile void *)g_frame_pool[i], FRAME_SIZE);
        if (g_cached_pool[i]) 
            free(g_cached_pool[i]);
    }
    mmio_close();

    pthread_mutex_destroy(&g_state.jpeg_mutex);
    pthread_mutex_destroy(&g_state.det_mutex);
    pthread_mutex_destroy(&g_state.cmd_mutex);
    pthread_mutex_destroy(&g_state.settings_mutex);
    pthread_mutex_destroy(&g_state.pred_mutex);
    if (g_state.jpeg_buf) free(g_state.jpeg_buf);

    printf("[main] Clean exit\n");
    return 0;
}
