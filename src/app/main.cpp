/* main.cpp — cam_system 入口 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>

extern "C" {
#include "cam_system/config.h"
#include "cam_system/app_state.h"
#include "platform/hal_ov5640.h"
#include "platform/hal_frmbuf.h"
#include "capture/capture.h"
#include "media/jpeg_enc.h"
#include "network/net_server.h"
#include "cam_system/tracker.h"
}
#include "vision/yolo_detect.h"

/* ── 全局状态 ── */
app_state_t g_state;
uint8_t *g_frame_pool[FRAME_BUF_CNT];
uint8_t *g_cached_pool[FRAME_BUF_CNT];
volatile sig_atomic_t g_app_signal_exit_requested = 0;

/* 全局 tracker 实例 */
tracker_t g_tracker;

void app_state_init(app_state_t *s)
{
    s->cam_enabled = 0;
    s->yolo_enabled = 0;
    s->quit = 0;
    s->tracking_active = 0;
    s->tracking_target_id = -1;
    s->latest_frame_idx = -1;
    s->frame_seq = 0;
    s->status_seq = 0;

    for (int i = 0; i < FRAME_BUF_CNT; i++) {
        s->frame_offsets[i].xoff = 16;
        s->frame_offsets[i].yoff = 4;
    }
    s->pred_box = predict_box_t{};
    for (int i = 0; i < YOLO_MAX_DET; i++)
        s->detections[i] = detection_t{};

    pthread_mutex_init(&s->jpeg_mutex, NULL);
    pthread_mutex_init(&s->det_mutex, NULL);
    pthread_mutex_init(&s->cmd_mutex, NULL);
    pthread_mutex_init(&s->settings_mutex, NULL);
    pthread_mutex_init(&s->pred_mutex, NULL);
    pthread_mutex_init(&s->frame_mutex, NULL);

    s->jpeg_buf = NULL;
    s->jpeg_size = 0;
    s->jpeg_seq = 0;
    s->det_count = 0;
    s->det_seq = 0;
    s->det_frame_seq = 0;
    s->det_frame_xoff = ISP_DEFAULT_XOFF;
    s->det_frame_yoff = ISP_DEFAULT_YOFF;
    s->track_cmd = TRACK_CMD_NONE;
    s->track_target_id = -1;
    s->cmd_head = 0;
    s->cmd_tail = 0;
    s->cmd_count = 0;
    for (int i = 0; i < CONTROL_CMD_QUEUE_LEN; i++) {
        s->cmd_queue[i] = TRACK_CMD_NONE;
        s->cmd_target_queue[i] = -1;
    }
    s->max_det = 10;

    memset(s->enabled_classes, 0, sizeof(s->enabled_classes));
    s->enabled_classes[0] = 0;
    s->enabled_class_count = 1;
}

/* ── 信号处理 ── */
static void signal_handler(int sig)
{
    (void)sig;
    g_app_signal_exit_requested = 1;
}

static int install_signal_handlers(void)
{
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);

    if (sigaction(SIGINT, &sa, NULL) < 0) {
        perror("[main] sigaction SIGINT");
        return -1;
    }
    if (sigaction(SIGTERM, &sa, NULL) < 0) {
        perror("[main] sigaction SIGTERM");
        return -1;
    }
    if (signal(SIGPIPE, SIG_IGN) == SIG_ERR) {
        perror("[main] signal SIGPIPE");
        return -1;
    }
    return 0;
}

/* ── CPU 亲和性 ── */
static int bind_current_to_core(const char *role, int core)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
    if (rc != 0) {
        fprintf(stderr, "[main] Failed to bind %s to Core %d: %s\n",
                role, core, strerror(rc));
        return -1;
    }
    printf("[main] %s bound to Core %d\n", role, core);
    return 0;
}

static int create_thread_on_core(pthread_t *thread, const char *role, int core,
                                 void *(*start_routine)(void *), void *arg)
{
    pthread_attr_t attr;
    int rc = pthread_attr_init(&attr);
    if (rc != 0) {
        fprintf(stderr, "[main] Failed to init %s attr: %s\n",
                role, strerror(rc));
        return -1;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    rc = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
    if (rc != 0) {
        fprintf(stderr, "[main] Failed to set %s attr Core %d: %s\n",
                role, core, strerror(rc));
        pthread_attr_destroy(&attr);
        return -1;
    }

    rc = pthread_create(thread, &attr, start_routine, arg);
    pthread_attr_destroy(&attr);
    if (rc != 0) {
        fprintf(stderr, "[main] Failed to create %s: %s\n",
                role, strerror(rc));
        return -1;
    }

    printf("[main] %s created on Core %d\n", role, core);
    return 0;
}

/* ── 主函数 ── */
int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    printf("============================================\n");
    printf("               cam_system\n");
    printf("============================================\n\n");

    if (install_signal_handlers() < 0)
        return 1;

    if (bind_current_to_core("main/network thread", CAM_CPU_NETWORK_CORE) < 0)
        return 1;

    /* 初始化全局状态 */
    app_state_init(&g_state);

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

    for (int i = 0; i < FRAME_BUF_CNT; i++) {
        g_frame_pool[i] = frmbuf_get_buffer(i);
        if (!g_frame_pool[i]) {
            fprintf(stderr, "[main] Failed to get frame buffer %d\n", i);
            return 1;
        }
        g_cached_pool[i] = (uint8_t *)malloc(FRAME_SIZE);
        if (!g_cached_pool[i]) {
            fprintf(stderr, "[main] Failed to alloc cached pool %d\n", i);
            return 1;
        }
        printf("[main] Frame pool[%d] ready\n", i);
    }

    /* 初始化 JPEG 编码器 */
    if (jpeg_init() < 0) {
        fprintf(stderr, "[main] JPEG init failed\n");
        return 1;
    }

    /* 初始化 tracker */
    tracker_init(&g_tracker);

    /* 启动采集/控制线程 */
    pthread_t cap_tid;
    if (create_thread_on_core(&cap_tid, "capture/control thread",
                              CAM_CPU_CAPTURE_CORE,
                              capture_thread, NULL) < 0)
        return 1;

    /* 初始化并启动 YOLO 线程 */
    if (yolo_init() < 0) {
        fprintf(stderr, "[main] YOLO init failed, continuing without detection\n");
    }
    pthread_t yolo_tid;
    if (create_thread_on_core(&yolo_tid, "YOLO thread", CAM_CPU_YOLO_CORE,
                              yolo_thread, NULL) < 0)
        return 1;

    printf("[main] Starting HTTP server on port %d...\n\n", HTTP_PORT);
    net_server_run();  /* 阻塞直到 quit */
    app_state_store_int(&g_state.quit, 1);

    /* ── 清理 ── */
    printf("[main] Shutting down...\n");

    pthread_join(cap_tid, NULL);
    pthread_join(yolo_tid, NULL);

    ov5640_standby(1);

    jpeg_close();
    yolo_close();
    frmbuf_close();
    ov5640_close();

    for (int i = 0; i < FRAME_BUF_CNT; i++) {
        if (g_cached_pool[i]) 
            free(g_cached_pool[i]);
    }

    pthread_mutex_destroy(&g_state.jpeg_mutex);
    pthread_mutex_destroy(&g_state.det_mutex);
    pthread_mutex_destroy(&g_state.cmd_mutex);
    pthread_mutex_destroy(&g_state.settings_mutex);
    pthread_mutex_destroy(&g_state.pred_mutex);
    pthread_mutex_destroy(&g_state.frame_mutex);
    if (g_state.jpeg_buf) free(g_state.jpeg_buf);

    printf("[main] Clean exit\n");
    return 0;
}
