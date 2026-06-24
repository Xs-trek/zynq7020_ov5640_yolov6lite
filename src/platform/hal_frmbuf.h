#ifndef HAL_FRMBUF_H
#define HAL_FRMBUF_H

#include <stdint.h>

/* 初始化 V4L2 采集后端，返回 0 成功。 */
int frmbuf_init(void);

/* 返回后端持有的帧缓冲虚拟地址。 */
uint8_t *frmbuf_get_buffer(int buf_idx);

/* 采集一帧，返回实际完成的 mmap buffer index。 */
int frmbuf_capture_frame(int preferred_idx, int timeout_ms);

/* 释放已完成帧；V4L2 后端会重新 QBUF。 */
int frmbuf_release_frame(int buf_idx);

/* 停止 IP */
void frmbuf_stop(void);

/* 关闭清理 */
void frmbuf_close(void);

#endif
