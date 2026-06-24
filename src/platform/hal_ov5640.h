#ifndef HAL_OV5640_H
#define HAL_OV5640_H

#include <stdint.h>

/* 初始化 OV5640 V4L2 控制后端，返回 0 成功，-1 失败。 */
int ov5640_init(void);

/* 进入/退出 standby（软件休眠） */
int ov5640_standby(int standby);

/* 原子写入 ISP offset；通过 packed V4L2 control 进入内核 group write。 */
int ov5640_apply_isp_offset(int x_offset, int y_offset);

/* 控制 ISP scaler bit。当前跟踪逻辑只需要开/关，不暴露任意 0x5001 写入。 */
int ov5640_set_scaler_enabled(int enabled);

/* 关闭 V4L2 subdev fd。 */
void ov5640_close(void);

#endif
