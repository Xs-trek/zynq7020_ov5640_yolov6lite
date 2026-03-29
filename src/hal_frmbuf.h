#ifndef HAL_FRMBUF_H
#define HAL_FRMBUF_H

#include <stdint.h>

/* 初始化 frmbuf_wr IP：释放复位 + 配置参数
 * 返回 0 成功 */
int frmbuf_init(void);

/* 启动单帧采集到指定地址
 * buf_idx: 帧缓冲索引 (0/1/2) */
int frmbuf_start_frame(int buf_idx);

/* 等待当前帧完成
 * timeout_ms: 超时毫秒数
 * 返回 0 完成，-1 超时 */
int frmbuf_wait_done(int timeout_ms);

/* 停止 IP */
void frmbuf_stop(void);

/* 关闭清理 */
void frmbuf_close(void);

#endif
