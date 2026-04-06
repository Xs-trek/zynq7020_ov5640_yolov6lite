/* config.h — 全局硬件常量与配置 */
#ifndef CONFIG_H
#define CONFIG_H

/* ── 帧参数 ── */
#define IMG_W           640
#define IMG_H           480
#define IMG_BPP         3
#define FRAME_SIZE      (IMG_W * IMG_H * IMG_BPP)   /* 921600 */
#define FRAME_BUF_CNT   3

/* ── PL 地址映射 ── */
#define FRMBUF_RST_GPIO_BASE  0x41200000
#define FRMBUF_WR_BASE        0x43C10000
#define FRAME_BUF_BASE        0x3F000000
#define FRAME_BUF_0           0x3F000000
#define FRAME_BUF_1           0x3F100000
#define FRAME_BUF_2           0x3F200000

/* ── v_frmbuf_rst 寄存器偏移 ── */
#define FRMBUF_GPIO_RESET     0x00   /* 全部复位          */
#define FRMBUF_GPIO_RELEASE   0x04   /* bit2=1 释放 wr    */

/* ── v_frmbuf_wr 寄存器偏移 (PG278 V2.4) ── */
#define FRMBUF_CTRL           0x00
#define FRMBUF_GIE            0x04
#define FRMBUF_IER            0x08
#define FRMBUF_ISR            0x0C
#define FRMBUF_WIDTH          0x10
#define FRMBUF_HEIGHT         0x18
#define FRMBUF_STRIDE         0x20
#define FRMBUF_FMT            0x28
#define FRMBUF_ADDR           0x30
#define FRMBUF_ADDR_HIGH      0x34
#define FRMBUF_FMT_RGB8       20

/* OV5640 控制 GPIO
 * zynq_gpio (PS EMIO): base=903
 * PDN = 903 + 67 = 970
 * RST = 903 + 68 = 971
 */
#define GPIO_CAM_PDN          970
#define GPIO_CAM_RST          971

/* ── OV5640 I2C ── */
#define OV5640_I2C_ADDR       0x3C
#define I2C_DEV_PATH          "/dev/i2c-2"

/* ── OV5640 Sensor 物理参数 ── */
#define SENSOR_FULL_W         2624      /* 物理窗口最大宽 (X: 0~2623) */
#define SENSOR_FULL_H         1944      /* 物理窗口最大高 (Y: 4~1947) */
#define SENSOR_Y_START_MIN    4         /* Y 起始最小值 */
#define SENSOR_X_START_MIN    0         /* X 起始最小值 */

/* ── ISP 窗口 / 数字云台参数 ── */
#define SUBSAMPLE_FACTOR      2         /* 当前 2x subsample 固定 */
#define ZOOM_MIN              100       /* 1.0x */
#define ZOOM_MAX              200       /* 2.0x (P3: subsample=2x 下极限) */
#define ZOOM_DEFAULT          100       /* 默认 1.0x */

/* zoom=1x 时的 sensor 窗口（与 init 寄存器表一致） */
#define ISP_WIN_1X_W          2624      /* 0x0000 ~ 0x0A3F */
#define ISP_WIN_1X_H          1944      /* 0x0004 ~ 0x079B */
#define ISP_WIN_1X_CX         1312      /* 中心 X */
#define ISP_WIN_1X_CY         976       /* 中心 Y = (4+1947)/2 ≈ 976 */

/* subsampled 后中心（zoom=1x, offset=(16,4) → 居中值） */
#define SUBSAMPLED_1X_W       1312      /* 2624/2 */
#define SUBSAMPLED_1X_H       972       /* 1944/2 */

/* PAN 极限：offset 最大范围（subsampled - output） */
#define PAN_MAX_X             672       /* 1312 - 640 */
#define PAN_MAX_Y             492       /* 972 - 480 */

/* ── 网络 ── */
#define HTTP_PORT             8080
#define MAX_HTTP_CLIENTS      4
#define MAX_WS_CLIENTS        4
#define MJPEG_BOUNDARY        "frameboundary"

/* ── YOLO ── */
#define YOLO_INPUT_W          256
#define YOLO_INPUT_H          192
#define YOLO_CONF_THRESH      0.5f
#define YOLO_NMS_THRESH       0.5f
#define YOLO_MAX_DET          20
#define YOLO_NUM_CLASSES      80

/* ── JPEG ── */
#define JPEG_QUALITY          70

/* ── 模型路径（板上部署路径） ── */
#define MODEL_PARAM_PATH      "/home/petalinux/yolov6lite_s.ncnn.param"
#define MODEL_BIN_PATH        "/home/petalinux/yolov6lite_s.ncnn.bin"

/* ── 跟踪参数 ── */

/* α-β 估计层 */
#define AB_ALPHA            0.9f      /* 位置修正增益 */
#define AB_BETA_RATE        1.71f     /* 速度修正增益 (1/s, 频率无关) */

#define OF_AB_ALPHA         0.45f
#define OF_AB_BETA_RATE     0.85f

/* 二阶临界阻尼执行层 */
#define DAMP_OMEGA          8.0f      /* 自然频率 (rad/s) */

/* 状态机 */
#define SM_WINDOW_SIZE      10        /* 滑动窗口大小 */
#define SM_CONF_TRACK       0.5f      /* 跟踪阈值 */
#define SM_CONF_COAST       0.2f      /* 滑行阈值 */
#define SM_LOST_TIMEOUT     3.0f      /* 丢失超时(秒) */

/* 目标匹配 */
#define MATCH_DIST_PX       150.0f    /* subsample 像素距离阈值 */

/* ISP offset 默认值 */
#define ISP_DEFAULT_XOFF    16
#define ISP_DEFAULT_YOFF    4

#endif /* CONFIG_H */
