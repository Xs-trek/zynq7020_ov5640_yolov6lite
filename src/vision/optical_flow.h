#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 清空光流状态 */
void of_init(void);

/* 当前是否处于有效追踪状态 */
int of_is_tracking(void);

/* 用当前帧中的目标框初始化局部稀疏光流
 * scr_cx, scr_cy, scr_w, scr_h 均为归一化屏幕坐标
 */
void of_reset(const uint8_t* rgb_data,
              float scr_cx, float scr_cy,
              float scr_w, float scr_h);

/* 高频补帧：当前帧执行光流，输出当前帧目标在 sub 空间中的绝对坐标 */
int of_track(const uint8_t* rgb_data,
             unsigned int frame_seq,
             int cur_snap_xoff, int cur_snap_yoff,
             float* out_abs_x, float* out_abs_y);

/* YOLO 延迟结果到达时，对光流进行校准
 * yolo_seq     : YOLO 检测对应的历史帧序号
 * yolo_abs_x/y : 该历史帧中的目标绝对 sub 坐标
 * w/h          : YOLO 检测框宽高（归一化）
 * curr_rgb     : 当前帧图像（用于必要时重建特征点）
 * cur_snap_x/y : 当前帧快照
 */
void of_calibrate(unsigned int yolo_seq,
                  float yolo_abs_x, float yolo_abs_y,
                  float w, float h,
                  const uint8_t* curr_rgb,
                  int cur_snap_x, int cur_snap_y);

#ifdef __cplusplus
}
#endif

#endif
