#ifndef TRACKER_H
#define TRACKER_H

#include "app_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 状态机 */
typedef enum {
    TRACK_STATE_IDLE = 0,
    TRACK_STATE_TRACKING,
    TRACK_STATE_COASTING,
    TRACK_STATE_LOST,
} track_state_t;

/* α-β 估计器 (像素空间) */
typedef struct {
    float ox, oy;       /* 位置估计 */
    float vx, vy;       /* 速度估计 (px/s) */
    int   initialized;
} ab_estimator_t;

/* 二阶临界阻尼执行层 */
typedef struct {
    float pos;
    float vel;
} damp_channel_t;

typedef struct {
    damp_channel_t x, y;
    int initialized;
} damp2_t;

/* 跟踪器 */
typedef struct {
    track_state_t state;
    int target_id;
    int target_class;

    /* 目标缓存 (SELECT 时保存) */
    int   select_valid;       /* 是否有有效的 select 缓存 */
    float select_sub_x;       /* 目标 subsample 坐标 */
    float select_sub_y;
    float select_bbox_w;      /* 归一化 bbox 宽高 (用于匹配) */
    float select_bbox_h;

    /* 估计层 */
    ab_estimator_t est;

    /* 执行层 */
    damp2_t damp;

    /* 状态机: 滑动窗口 */
    int match_history[SM_WINDOW_SIZE];
    int history_idx;
    int history_count;

    /* 上次目标位置 (subsample 像素) */
    float last_sub_x, last_sub_y;

    /* 启动稳定计数 */
    int startup_frames;

    /* 计时 */
    float last_seen_time;
    float lost_time;
    float last_predict_time;

} tracker_t;

void tracker_init(tracker_t *trk);
void tracker_handle_cmd(tracker_t *trk);
void tracker_update_detections(tracker_t *trk,
                               detection_t *dets, int det_count,
                               int snap_xoff, int snap_yoff,
                               float time_sec);
void tracker_predict(tracker_t *trk, float time_sec);
int tracker_is_active(const tracker_t *trk);
int tracker_get_target_id(const tracker_t *trk);
void tracker_stop(tracker_t *trk);
void tracker_reset_frame_flag(void);
void tracker_get_isp_offset(int *xoff, int *yoff);
void tracker_write_isp_offset(int x, int y);

/* 调参 (保留接口) */
void tracker_tune(tracker_t *trk, const char *key, float value);

#ifdef __cplusplus
}
#endif

#endif
