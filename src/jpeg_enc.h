#ifndef JPEG_ENC_H
#define JPEG_ENC_H

#include <stdint.h>

/* 初始化 JPEG 编码器 */
int jpeg_init(void);

/* 将 RGB888 帧编码为 JPEG，结果存入 g_state.jpeg_buf/size */
int jpeg_encode_frame(const uint8_t *rgb_data);

/* 关闭编码器 */
void jpeg_close(void);

#endif
