#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>
#include <signal.h>
#include <pthread.h>
#include "cam_system/config.h"

typedef struct {
    float bbox[4];
    int   class_id;
    float confidence;
    int   track_id;
    int   is_target;
} detection_t;

typedef struct {
    int   valid;
    float cx, cy;      /* 归一化 screen 中心 [0,1] */
    float w, h;        /* 归一化 bbox 宽高 */
    int   class_id;
    int   track_id;
} predict_box_t;

typedef enum {
    TRACK_CMD_NONE = 0,
    TRACK_CMD_START,
    TRACK_CMD_STOP,
    TRACK_CMD_SELECT
} track_cmd_t;

/* 每帧的 ISP offset 快照 */
typedef struct {
    int xoff;
    int yoff;
} frame_offset_t;

typedef struct {
    int cam_enabled;
    int yolo_enabled;
    int quit;

    int tracking_active;
    int tracking_target_id;

    int latest_frame_idx;
    unsigned int frame_seq;
    unsigned int status_seq;

    /* 每个帧缓冲对应的 offset 快照 */
    pthread_mutex_t frame_mutex;
    frame_offset_t frame_offsets[FRAME_BUF_CNT];

    /* 预测框互斥锁 */
    pthread_mutex_t pred_mutex;

    /* 跟踪预测框 (tracker 每帧更新 @30fps) */
    predict_box_t pred_box;

    pthread_mutex_t jpeg_mutex;
    uint8_t  *jpeg_buf;
    size_t    jpeg_size;
    unsigned int jpeg_seq;

    pthread_mutex_t det_mutex;
    detection_t detections[YOLO_MAX_DET];
    int det_count;
    unsigned int det_seq;
    unsigned int det_frame_seq;
    int det_frame_xoff;
    int det_frame_yoff;

    pthread_mutex_t cmd_mutex;
    track_cmd_t track_cmd;
    int track_target_id;
    track_cmd_t cmd_queue[CONTROL_CMD_QUEUE_LEN];
    int cmd_target_queue[CONTROL_CMD_QUEUE_LEN];
    int cmd_head;
    int cmd_tail;
    int cmd_count;

    pthread_mutex_t settings_mutex;
    int enabled_classes[YOLO_NUM_CLASSES];
    int enabled_class_count;
    int max_det;

} app_state_t;

static inline int app_state_load_int(const int *p)
{
    return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}

static inline void app_state_store_int(int *p, int v)
{
    __atomic_store_n(p, v, __ATOMIC_RELEASE);
}

static inline unsigned int app_state_load_u32(const unsigned int *p)
{
    return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}

static inline void app_state_store_u32(unsigned int *p, unsigned int v)
{
    __atomic_store_n(p, v, __ATOMIC_RELEASE);
}

static inline unsigned int app_state_inc_u32(unsigned int *p)
{
    return __atomic_add_fetch(p, 1U, __ATOMIC_RELEASE);
}

#ifdef __cplusplus
extern "C" {
#endif

extern app_state_t g_state;
extern uint8_t *g_frame_pool[FRAME_BUF_CNT];
extern uint8_t *g_cached_pool[FRAME_BUF_CNT];
extern volatile sig_atomic_t g_app_signal_exit_requested;

void app_state_init(app_state_t *s);

#ifdef __cplusplus
}
#endif

#endif
