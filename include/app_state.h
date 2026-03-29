#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>
#include <pthread.h>
#include "config.h"

typedef struct {
    float bbox[4];
    int   class_id;
    float confidence;
    int   track_id;
    int   is_target;
} detection_t;

typedef struct {
    volatile int   valid;
    volatile float cx, cy;      /* 归一化 screen 中心 [0,1] */
    volatile float w, h;        /* 归一化 bbox 宽高 */
    volatile int   class_id;
    volatile int   track_id;
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
    volatile int cam_enabled;
    volatile int yolo_enabled;
    volatile int quit;
    volatile int zoom_level;
    volatile int zoom_pending;

    volatile int tracking_active;
    volatile int tracking_target_id;

    volatile int latest_frame_idx;
    volatile unsigned int frame_seq;

    /* 每个帧缓冲对应的 offset 快照 */
    frame_offset_t frame_offsets[FRAME_BUF_CNT];

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

    pthread_mutex_t cmd_mutex;
    track_cmd_t track_cmd;
    int track_target_id;

    pthread_mutex_t settings_mutex;
    int enabled_classes[YOLO_NUM_CLASSES];
    int enabled_class_count;
    int max_det;

} app_state_t;

#ifdef __cplusplus
extern "C" {
#endif

extern app_state_t g_state;
extern uint8_t *g_frame_pool[FRAME_BUF_CNT];

void app_state_init(app_state_t *s);

#ifdef __cplusplus
}
#endif

#endif
