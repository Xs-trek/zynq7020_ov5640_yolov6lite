/* hal_ov5640.c — OV5640 硬件抽象层
 *
 * 管理：上电时序、I2C 寄存器初始化、standby、ISP 窗口数字云台
 * P3: 新增 ov5640_set_digital_ptz() 实现 pan+zoom
 */
#include "hal_ov5640.h"
#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/* ── 寄存器表结构 ── */
typedef struct {
    uint16_t addr;
    uint8_t  val;
} reg_entry_t;

/* ── VGA 30fps RGB565 寄存器表（手册修正版） ── */
static const reg_entry_t ov5640_init_regs[] = {
    /* 软件复位 */
    {0x3008, 0x82},
    {0x3008, 0x02},

    /* 系统/IO 控制 */
    {0x3103, 0x02},
    {0x3017, 0xff},
    {0x3018, 0xff},

    /* 模块复位与时钟门控 */
    {0x3002, 0x1c},
    {0x3006, 0xc3},

    /* PLL 配置 */
    {0x3037, 0x13},
    {0x3108, 0x01},

    /* 模拟控制 */
    {0x3630, 0x36}, {0x3631, 0x0e}, {0x3632, 0xe2}, {0x3633, 0x12},
    {0x3621, 0xe0}, {0x3704, 0xa0}, {0x3703, 0x5a}, {0x3715, 0x78},
    {0x3717, 0x01}, {0x370b, 0x60}, {0x3705, 0x1a}, {0x3905, 0x02},
    {0x3906, 0x10}, {0x3901, 0x0a}, {0x3731, 0x12}, {0x3600, 0x08},
    {0x3601, 0x33}, {0x302d, 0x60}, {0x3620, 0x52}, {0x371b, 0x20},
    {0x471c, 0x50},

    /* AEC/AGC */
    {0x3a13, 0x43}, {0x3a18, 0x00}, {0x3a19, 0xf8},
    {0x3503, 0x00},

    /* 模拟控制续 */
    {0x3635, 0x13}, {0x3636, 0x03}, {0x3634, 0x40}, {0x3622, 0x01},

    /* 50/60Hz 检测 */
    {0x3c01, 0x34}, {0x3c04, 0x28}, {0x3c05, 0x98},
    {0x3c06, 0x00}, {0x3c07, 0x08}, {0x3c08, 0x00},
    {0x3c09, 0x1c}, {0x3c0a, 0x9c}, {0x3c0b, 0x40},

    /* ISP 窗口偏移（初始值，P3 运行时会动态修改） */
    {0x3810, 0x00}, {0x3811, 0x10}, {0x3812, 0x00},

    /* 传感器控制 */
    {0x3708, 0x64},

    /* BLC */
    {0x4001, 0x02}, {0x4005, 0x1a},

    /* 系统控制 */
    {0x3000, 0x00}, {0x3004, 0xff},

    /* 输出格式：RGB565 */
    {0x4300, 0x61},
    {0x501f, 0x01},

    /* JPEG 相关 */
    {0x440e, 0x00}, {0x460b, 0x35},

    /* ISP 控制 */
    {0x5000, 0xa7},

    /* AEC 目标/范围 */
    {0x3a0f, 0x30}, {0x3a10, 0x28}, {0x3a1b, 0x30},
    {0x3a1e, 0x26}, {0x3a11, 0x60}, {0x3a1f, 0x14},

    /* LENC */
    {0x5800, 0x23}, {0x5801, 0x14}, {0x5802, 0x0f}, {0x5803, 0x0f},
    {0x5804, 0x12}, {0x5805, 0x26}, {0x5806, 0x0c}, {0x5807, 0x08},
    {0x5808, 0x05}, {0x5809, 0x05}, {0x580a, 0x08}, {0x580b, 0x0d},
    {0x580c, 0x08}, {0x580d, 0x03}, {0x580e, 0x00}, {0x580f, 0x00},
    {0x5810, 0x03}, {0x5811, 0x09}, {0x5812, 0x07}, {0x5813, 0x03},
    {0x5814, 0x00}, {0x5815, 0x01}, {0x5816, 0x03}, {0x5817, 0x08},
    {0x5818, 0x0d}, {0x5819, 0x08}, {0x581a, 0x05}, {0x581b, 0x06},
    {0x581c, 0x08}, {0x581d, 0x0e}, {0x581e, 0x29}, {0x581f, 0x17},
    {0x5820, 0x11}, {0x5821, 0x11}, {0x5822, 0x15}, {0x5823, 0x28},
    {0x5824, 0x46}, {0x5825, 0x26}, {0x5826, 0x08}, {0x5827, 0x26},
    {0x5828, 0x64}, {0x5829, 0x26}, {0x582a, 0x24}, {0x582b, 0x22},
    {0x582c, 0x24}, {0x582d, 0x24}, {0x582e, 0x06}, {0x582f, 0x22},
    {0x5830, 0x40}, {0x5831, 0x42}, {0x5832, 0x24}, {0x5833, 0x26},
    {0x5834, 0x24}, {0x5835, 0x22}, {0x5836, 0x22}, {0x5837, 0x26},
    {0x5838, 0x44}, {0x5839, 0x24}, {0x583a, 0x26}, {0x583b, 0x28},
    {0x583c, 0x42}, {0x583d, 0xce},

    /* AWB */
    {0x5180, 0xff}, {0x5181, 0xf2}, {0x5182, 0x00}, {0x5183, 0x14},
    {0x5184, 0x25}, {0x5185, 0x24}, {0x5186, 0x09}, {0x5187, 0x09},
    {0x5188, 0x09}, {0x5189, 0x75}, {0x518a, 0x54}, {0x518b, 0xe0},
    {0x518c, 0xb2}, {0x518d, 0x42}, {0x518e, 0x3d}, {0x518f, 0x56},
    {0x5190, 0x46}, {0x5191, 0xf8}, {0x5192, 0x04}, {0x5193, 0x70},
    {0x5194, 0xf0}, {0x5195, 0xf0}, {0x5196, 0x03}, {0x5197, 0x01},
    {0x5198, 0x04}, {0x5199, 0x12}, {0x519a, 0x04}, {0x519b, 0x00},
    {0x519c, 0x06}, {0x519d, 0x82}, {0x519e, 0x38},

    /* Gamma */
    {0x5480, 0x01}, {0x5481, 0x08}, {0x5482, 0x14}, {0x5483, 0x28},
    {0x5484, 0x51}, {0x5485, 0x65}, {0x5486, 0x71}, {0x5487, 0x7d},
    {0x5488, 0x87}, {0x5489, 0x91}, {0x548a, 0x9a}, {0x548b, 0xaa},
    {0x548c, 0xb8}, {0x548d, 0xcd}, {0x548e, 0xdd}, {0x548f, 0xea},
    {0x5490, 0x1d},

    /* CCM */
    {0x5381, 0x1e}, {0x5382, 0x5b}, {0x5383, 0x08}, {0x5384, 0x0a},
    {0x5385, 0x7e}, {0x5386, 0x88}, {0x5387, 0x7c}, {0x5388, 0x6c},
    {0x5389, 0x10}, {0x538a, 0x01}, {0x538b, 0x98},

    /* SDE */
    {0x5580, 0x06}, {0x5583, 0x40}, {0x5584, 0x10},
    {0x5589, 0x10}, {0x558a, 0x00}, {0x558b, 0xf8},
    {0x501d, 0x40},

    /* CIP/DNS */
    {0x5300, 0x08}, {0x5301, 0x30}, {0x5302, 0x10}, {0x5303, 0x00},
    {0x5304, 0x08}, {0x5305, 0x30}, {0x5306, 0x08}, {0x5307, 0x16},
    {0x5309, 0x08}, {0x530a, 0x30}, {0x530b, 0x04}, {0x530c, 0x06},
    {0x5025, 0x00},

    /* ═══ VGA 30fps 关键配置 ═══ */
    {0x3035, 0x11},
    {0x3036, 0x46},
    {0x3c07, 0x07},
    {0x3820, 0x47},
    {0x3821, 0x07},
    {0x3814, 0x31},
    {0x3815, 0x31},

    /* ISP 输入窗口 */
    {0x3800, 0x00}, {0x3801, 0x00},
    {0x3802, 0x00}, {0x3803, 0x04},
    {0x3804, 0x0a}, {0x3805, 0x3f},
    {0x3806, 0x07}, {0x3807, 0x9b},

    /* 输出尺寸 640×480 */
    {0x3808, 0x02}, {0x3809, 0x80},
    {0x380a, 0x01}, {0x380b, 0xe0},

    /* HTS / VTS */
    {0x380c, 0x07}, {0x380d, 0x68},
    {0x380e, 0x03}, {0x380f, 0xd8},

    {0x3813, 0x04},

    /* 模拟控制（VGA 值） */
    {0x3618, 0x00}, {0x3612, 0x29},
    {0x3709, 0x52}, {0x370c, 0x03},

    /* AEC 最大曝光 = VTS */
    {0x3a02, 0x03}, {0x3a03, 0xd8},
    {0x3a14, 0x03}, {0x3a15, 0xd8},

    /* BLC */
    {0x4004, 0x02},

    /* DVP / MIPI 杂项 */
    {0x4713, 0x03}, {0x4407, 0x04}, {0x460c, 0x22}, {0x4837, 0x22},
    {0x3824, 0x02},

    /* ISP 控制 */
    {0x5001, 0xa3},

    /* 夜间模式 */
    {0x3b07, 0x0a},

    /* 测试图案关闭 */
    {0x503d, 0x00},

    /* FREX/Strobe */
    {0x3016, 0x02}, {0x301c, 0x02},
    {0x3019, 0x02}, {0x3019, 0x00},

    /* DVP CLK */
    {0x4720, 0x20},
};
#define INIT_REG_COUNT (sizeof(ov5640_init_regs) / sizeof(ov5640_init_regs[0]))

/* ── sysfs GPIO 操作 ── */
static int gpio_export(int gpio)
{
    char buf[64];
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        if (errno == EBUSY) return 0;
        return -1;
    }
    int n = snprintf(buf, sizeof(buf), "%d", gpio);
    int ret = write(fd, buf, n);
    close(fd);
    if (ret < 0 && errno != EBUSY) return -1;
    return 0;
}

static int gpio_set_dir(int gpio, const char *dir)
{
    char path[128];
    snprintf(path, sizeof(path),
             "/sys/class/gpio/gpio%d/direction", gpio);
    for (int i = 0; i < 20; i++) {
        int fd = open(path, O_WRONLY);
        if (fd >= 0) {
            ssize_t n = write(fd, dir, strlen(dir));
            close(fd);
            if (n < 0) {
                perror("[ov5640] gpio_set_dir write");
                return -1;
            }
            return 0;
        }
        usleep(50000);
    }
    fprintf(stderr, "[ov5640] gpio%d direction timeout\n", gpio);
    return -1;
}

static int gpio_set_val(int gpio, int val)
{
    char path[128], vbuf[4];
    snprintf(path, sizeof(path),
             "/sys/class/gpio/gpio%d/value", gpio);
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "[ov5640] gpio%d open value failed\n", gpio);
        return -1;
    }
    int n = snprintf(vbuf, sizeof(vbuf), "%d", val);
    ssize_t w = write(fd, vbuf, n);
    close(fd);
    if (w < 0) {
        perror("[ov5640] gpio_set_val write");
        return -1;
    }
    return 0;
}

/* ── I2C ── */
static int i2c_fd = -1;

static int i2c_open(void)
{
    i2c_fd = open(I2C_DEV_PATH, O_RDWR);
    if (i2c_fd < 0) { perror("[ov5640] open i2c"); return -1; }
    if (ioctl(i2c_fd, I2C_SLAVE, OV5640_I2C_ADDR) < 0) {
        perror("[ov5640] ioctl I2C_SLAVE");
        close(i2c_fd); i2c_fd = -1;
        return -1;
    }
    return 0;
}

int ov5640_write_reg(uint16_t reg, uint8_t val)
{
    uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff), val };
    for (int retry = 0; retry < 3; retry++) {
        if (write(i2c_fd, buf, 3) == 3) return 0;
        usleep(5000);
    }
    fprintf(stderr, "[ov5640] I2C write fail: 0x%04x=0x%02x\n", reg, val);
    return -1;
}

int ov5640_read_reg(uint16_t reg, uint8_t *val)
{
    uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff) };
    if (write(i2c_fd, addr, 2) != 2) return -1;
    if (read(i2c_fd, val, 1) != 1) return -1;
    return 0;
}

/* ── P3: 当前 ROI 状态（sensor 坐标系） ── */
static int cur_roi_cx = ISP_WIN_1X_CX;   /* 1312 */
static int cur_roi_cy = ISP_WIN_1X_CY;   /* 976  */
static int cur_zoom   = ZOOM_DEFAULT;     /* 100  */

/* ── 辅助：clamp ── */
static int clamp_int(int val, int lo, int hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* ── 数字云台核心实现 ── */

int ov5640_set_digital_ptz(int roi_cx, int roi_cy, int zoom_level)
{
    (void)roi_cx;
    (void)roi_cy;

    /* ── 1. 钳位 zoom ── */
    zoom_level = clamp_int(zoom_level, ZOOM_MIN, ZOOM_MAX);

    /* ── 2. 计算 offset ── */

    const int base_x_offset = ISP_DEFAULT_XOFF;
    const int base_y_offset = ISP_DEFAULT_YOFF;
    const int sub_w = SUBSAMPLED_1X_W;  /* 1312 */
    const int sub_h = SUBSAMPLED_1X_H;  /* 972  */

    /* zoom_level 是 100-200 的整数，代表 1.0x-2.0x */
    /* effective = sub / (zoom_level/100) = sub * 100 / zoom_level */
    int eff_w = sub_w * 100 / zoom_level;
    int eff_h = sub_h * 100 / zoom_level;

    int x_offset = base_x_offset + (sub_w - eff_w) / 2;
    int y_offset = base_y_offset + (sub_h - eff_h) / 2;

    /* 钳位：确保输出区域不超出有效阵列 */
    int max_x_offset = (sub_w - IMG_W) / 2;
    int max_y_offset = (sub_h - IMG_H) / 2;
    x_offset = clamp_int(x_offset, 0, max_x_offset);
    y_offset = clamp_int(y_offset, 0, max_y_offset);

    printf("[ov5640] PTZ: zoom=%d%% eff=%dx%d offset=(%d,%d) max=(%d,%d)\n",
           zoom_level, eff_w, eff_h, x_offset, y_offset,
           max_x_offset, max_y_offset);

    /* ── 3. Group Write (官方样例使用 group 3) ── */
    ov5640_write_reg(0x3212, 0x03);  /* start group 3 */

    /* 只写 offset，不改窗口和输出尺寸 */
    ov5640_write_reg(0x3810, (x_offset >> 8) & 0x0F);
    ov5640_write_reg(0x3811, x_offset & 0xFF);
    ov5640_write_reg(0x3812, (y_offset >> 8) & 0x07);
    ov5640_write_reg(0x3813, y_offset & 0xFF);

    ov5640_write_reg(0x3212, 0x13);  /* end group 3 */
    ov5640_write_reg(0x3212, 0xA3);  /* launch group 3 */

    /* 等待一帧生效 */
    usleep(40000);

    /* ── 4. 回读验证 ── */
    {
        uint8_t r10, r11, r12, r13;
        ov5640_read_reg(0x3810, &r10);
        ov5640_read_reg(0x3811, &r11);
        ov5640_read_reg(0x3812, &r12);
        ov5640_read_reg(0x3813, &r13);

        int rb_xoff = ((r10 & 0x0F) << 8) | r11;
        int rb_yoff = ((r12 & 0x07) << 8) | r13;

        printf("[ov5640] PTZ readback: xoff=%d(%d) yoff=%d(%d) %s\n",
               rb_xoff, x_offset, rb_yoff, y_offset,
               (rb_xoff == x_offset && rb_yoff == y_offset) ? "OK" : "MISMATCH!");
    }

    /* ── 5. 更新内部状态 ── */
    cur_roi_cx = ISP_WIN_1X_CX;
    cur_roi_cy = ISP_WIN_1X_CY;
    cur_zoom = zoom_level;

    return 0;
}

void ov5640_get_current_roi(int *roi_cx, int *roi_cy)
{
    if (roi_cx) *roi_cx = cur_roi_cx;
    if (roi_cy) *roi_cy = cur_roi_cy;
}

int ov5640_reset_ptz(void)
{
    printf("[ov5640] Resetting PTZ to center, zoom=1x\n");
    return ov5640_set_digital_ptz(ISP_WIN_1X_CX, ISP_WIN_1X_CY, ZOOM_DEFAULT);
}

/* ── 公开接口 ── */

int ov5640_init(void)
{
    printf("[ov5640] Power-up sequence...\n");

    gpio_export(GPIO_CAM_PDN);
    gpio_export(GPIO_CAM_RST);
    
    if (gpio_set_dir(GPIO_CAM_PDN, "out") < 0) return -1;
    if (gpio_set_dir(GPIO_CAM_RST, "out") < 0) return -1;
    if (gpio_set_val(GPIO_CAM_PDN, 1) < 0) return -1;
    if (gpio_set_val(GPIO_CAM_RST, 0) < 0) return -1;
    usleep(50000);
    if (gpio_set_val(GPIO_CAM_PDN, 0) < 0) return -1;
    usleep(20000);
    if (gpio_set_val(GPIO_CAM_RST, 1) < 0) return -1;
    usleep(50000);

    printf("[ov5640] PDN=0, RST=1\n");

    if (i2c_open() < 0) return -1;

    /* Chip ID */
    uint8_t id_h, id_l;
    ov5640_read_reg(0x300a, &id_h);
    ov5640_read_reg(0x300b, &id_l);
    uint16_t chip_id = ((uint16_t)id_h << 8) | id_l;
    printf("[ov5640] Chip ID: 0x%04X %s\n", chip_id,
           chip_id == 0x5640 ? "(OK)" : "(MISMATCH!)");
    if (chip_id != 0x5640) return -1;

    /* 写寄存器表 */
    printf("[ov5640] Writing %d registers...\n", (int)INIT_REG_COUNT);
    int errs = 0;
    for (size_t i = 0; i < INIT_REG_COUNT; i++) {
        if (ov5640_write_reg(ov5640_init_regs[i].addr, ov5640_init_regs[i].val) < 0)
            errs++;
        if (ov5640_init_regs[i].addr == 0x3008 && ov5640_init_regs[i].val == 0x82)
            usleep(100000);
        else if (ov5640_init_regs[i].addr == 0x3008 && ov5640_init_regs[i].val == 0x02)
            usleep(20000);
        else
            usleep(1000);
    }
    printf("[ov5640] Done, I2C errors: %d\n", errs);

    /* 回读验证 */
    struct { uint16_t reg; uint8_t expect; const char *name; } checks[] = {
        {0x3036, 0x46, "PLL_MUL"},  {0x3814, 0x31, "X_SUB"},
        {0x3815, 0x31, "Y_SUB"},    {0x380c, 0x07, "HTS_H"},
        {0x380d, 0x68, "HTS_L"},    {0x380e, 0x03, "VTS_H"},
        {0x380f, 0xd8, "VTS_L"},    {0x3820, 0x47, "FLIP"},
        {0x4300, 0x61, "FMT"},      {0x501f, 0x01, "ISP"},
        {0x3824, 0x02, "DVP_DIV"},  {0x4004, 0x02, "BLC"},
    };
    for (size_t i = 0; i < sizeof(checks)/sizeof(checks[0]); i++) {
        uint8_t v;
        ov5640_read_reg(checks[i].reg, &v);
        printf("  0x%04X [%s]: 0x%02X %s\n",
               checks[i].reg, checks[i].name, v,
               v == checks[i].expect ? "OK" : "MISMATCH!");
    }

    /* P3: 回读 ISP scaler 寄存器（诊断信息） */
    {
        uint8_t v;
        printf("[ov5640] ISP Scaler diagnostics:\n");
        ov5640_read_reg(0x5600, &v); printf("  0x5600 SCALE_CTRL0: 0x%02X\n", v);
        ov5640_read_reg(0x5601, &v); printf("  0x5601 SCALE_CTRL1: 0x%02X\n", v);
        ov5640_read_reg(0x5602, &v); printf("  0x5602 XSC_H:       0x%02X\n", v);
        ov5640_read_reg(0x5603, &v); printf("  0x5603 XSC_L:       0x%02X\n", v);
        ov5640_read_reg(0x5604, &v); printf("  0x5604 YSC_H:       0x%02X\n", v);
        ov5640_read_reg(0x5605, &v); printf("  0x5605 YSC_L:       0x%02X\n", v);
    }

    /* 等待 ISP 稳定 */
    printf("[ov5640] Waiting 500ms for ISP stabilization...\n");
    usleep(500000);

    /* P3: 初始化 PTZ 到居中 */
    cur_roi_cx = ISP_WIN_1X_CX;
    cur_roi_cy = ISP_WIN_1X_CY;
    cur_zoom = ZOOM_DEFAULT;
    printf("[ov5640] PTZ initialized: center=(%d,%d) zoom=%d%%\n",
           cur_roi_cx, cur_roi_cy, cur_zoom);

    return 0;
}

int ov5640_standby(int standby)
{
    if (standby) {
        printf("[ov5640] Entering standby\n");
        return ov5640_write_reg(0x3008, 0x42);
    } else {
        printf("[ov5640] Waking up\n");
        int ret = ov5640_write_reg(0x3008, 0x02);
        usleep(500000);
        return ret;
    }
}

void ov5640_close(void)
{
    if (i2c_fd >= 0) {
        close(i2c_fd);
        i2c_fd = -1;
    }
}
