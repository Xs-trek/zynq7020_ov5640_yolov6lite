#ifndef HAL_OV5640_H
#define HAL_OV5640_H

#include <stdint.h>

/* 初始化 OV5640：上电 + I2C 配置 VGA 30fps RGB565
 * 返回 0 成功，-1 失败 */
int ov5640_init(void);

/* 进入/退出 standby（软件休眠） */
int ov5640_standby(int standby);

/* ── 数字云台 ── */
int ov5640_set_digital_ptz(int roi_cx, int roi_cy, int zoom_level);

/* 获取当前 ROI 中心（sensor 坐标）*/
void ov5640_get_current_roi(int *roi_cx, int *roi_cy);

/* 重置到默认居中 zoom=1x */
int ov5640_reset_ptz(void);

/* 关闭 I2C */
void ov5640_close(void);

/* 读写单个寄存器（供外部诊断用） */
int ov5640_write_reg(uint16_t reg, uint8_t val);
int ov5640_read_reg(uint16_t reg, uint8_t *val);

#endif
