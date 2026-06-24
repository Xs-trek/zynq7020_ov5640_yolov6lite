// SPDX-License-Identifier: GPL-2.0-only
/*
 * Project OV5640 control-plane driver.
 *
 * This driver intentionally covers only the current project contract:
 * fixed 640x480 RGB565 DVP mode, software standby, ISP offset group write,
 * and scaler enable/disable. Frame capture remains in the existing PL
 * frmbuf path and is not exposed as a V4L2 video node in A09.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#define CAM_OV5640_CID_BASE          (V4L2_CID_USER_BASE + 0x2000)
#define CAM_OV5640_CID_INITIALIZE    (CAM_OV5640_CID_BASE + 0)
#define CAM_OV5640_CID_STANDBY       (CAM_OV5640_CID_BASE + 1)
#define CAM_OV5640_CID_ISP_OFFSET    (CAM_OV5640_CID_BASE + 2)
#define CAM_OV5640_CID_SCALER_ENABLE (CAM_OV5640_CID_BASE + 3)

#define OV5640_CHIP_ID_REG 0x300a
#define OV5640_REG_SYS_CTRL0 0x3008
#define OV5640_REG_SYS_CTRL0_SW_PWDN 0x42
#define OV5640_REG_SYS_CTRL0_SW_PWUP 0x02

#define ISP_DEFAULT_XOFF 16
#define ISP_DEFAULT_YOFF 4
#define PAN_MAX_X 672
#define PAN_MAX_Y 492

struct cam_ov5640_reg {
	u16 addr;
	u8 val;
};

static const struct cam_ov5640_reg cam_ov5640_init_regs[] = {
	{0x3008, 0x82}, {0x3008, 0x02},
	{0x3103, 0x02}, {0x3017, 0xff}, {0x3018, 0xff},
	{0x3002, 0x1c}, {0x3006, 0xc3},
	{0x3037, 0x13}, {0x3108, 0x01},
	{0x3630, 0x36}, {0x3631, 0x0e}, {0x3632, 0xe2}, {0x3633, 0x12},
	{0x3621, 0xe0}, {0x3704, 0xa0}, {0x3703, 0x5a}, {0x3715, 0x78},
	{0x3717, 0x01}, {0x370b, 0x60}, {0x3705, 0x1a}, {0x3905, 0x02},
	{0x3906, 0x10}, {0x3901, 0x0a}, {0x3731, 0x12}, {0x3600, 0x08},
	{0x3601, 0x33}, {0x302d, 0x60}, {0x3620, 0x52}, {0x371b, 0x20},
	{0x471c, 0x50},
	{0x3a13, 0x43}, {0x3a18, 0x00}, {0x3a19, 0xf8}, {0x3503, 0x00},
	{0x3635, 0x13}, {0x3636, 0x03}, {0x3634, 0x40}, {0x3622, 0x01},
	{0x3c01, 0x34}, {0x3c04, 0x28}, {0x3c05, 0x98},
	{0x3c06, 0x00}, {0x3c07, 0x08}, {0x3c08, 0x00},
	{0x3c09, 0x1c}, {0x3c0a, 0x9c}, {0x3c0b, 0x40},
	{0x3810, 0x00}, {0x3811, 0x10}, {0x3812, 0x00},
	{0x3708, 0x64},
	{0x4001, 0x02}, {0x4005, 0x1a},
	{0x3000, 0x00}, {0x3004, 0xff},
	{0x4300, 0x61}, {0x501f, 0x01},
	{0x440e, 0x00}, {0x460b, 0x35},
	{0x5000, 0xa7},
	{0x3a0f, 0x30}, {0x3a10, 0x28}, {0x3a1b, 0x30},
	{0x3a1e, 0x26}, {0x3a11, 0x60}, {0x3a1f, 0x14},
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
	{0x5180, 0xff}, {0x5181, 0xf2}, {0x5182, 0x00}, {0x5183, 0x14},
	{0x5184, 0x25}, {0x5185, 0x24}, {0x5186, 0x09}, {0x5187, 0x09},
	{0x5188, 0x09}, {0x5189, 0x75}, {0x518a, 0x54}, {0x518b, 0xe0},
	{0x518c, 0xb2}, {0x518d, 0x42}, {0x518e, 0x3d}, {0x518f, 0x56},
	{0x5190, 0x46}, {0x5191, 0xf8}, {0x5192, 0x04}, {0x5193, 0x70},
	{0x5194, 0xf0}, {0x5195, 0xf0}, {0x5196, 0x03}, {0x5197, 0x01},
	{0x5198, 0x04}, {0x5199, 0x12}, {0x519a, 0x04}, {0x519b, 0x00},
	{0x519c, 0x06}, {0x519d, 0x82}, {0x519e, 0x38},
	{0x5480, 0x01}, {0x5481, 0x08}, {0x5482, 0x14}, {0x5483, 0x28},
	{0x5484, 0x51}, {0x5485, 0x65}, {0x5486, 0x71}, {0x5487, 0x7d},
	{0x5488, 0x87}, {0x5489, 0x91}, {0x548a, 0x9a}, {0x548b, 0xaa},
	{0x548c, 0xb8}, {0x548d, 0xcd}, {0x548e, 0xdd}, {0x548f, 0xea},
	{0x5490, 0x1d},
	{0x5381, 0x1e}, {0x5382, 0x5b}, {0x5383, 0x08}, {0x5384, 0x0a},
	{0x5385, 0x7e}, {0x5386, 0x88}, {0x5387, 0x7c}, {0x5388, 0x6c},
	{0x5389, 0x10}, {0x538a, 0x01}, {0x538b, 0x98},
	{0x5580, 0x06}, {0x5583, 0x40}, {0x5584, 0x10},
	{0x5589, 0x10}, {0x558a, 0x00}, {0x558b, 0xf8}, {0x501d, 0x40},
	{0x5300, 0x08}, {0x5301, 0x30}, {0x5302, 0x10}, {0x5303, 0x00},
	{0x5304, 0x08}, {0x5305, 0x30}, {0x5306, 0x08}, {0x5307, 0x16},
	{0x5309, 0x08}, {0x530a, 0x30}, {0x530b, 0x04}, {0x530c, 0x06},
	{0x5025, 0x00},
	{0x3035, 0x11}, {0x3036, 0x46}, {0x3c07, 0x07},
	{0x3820, 0x47}, {0x3821, 0x07}, {0x3814, 0x31}, {0x3815, 0x31},
	{0x3800, 0x00}, {0x3801, 0x00}, {0x3802, 0x00}, {0x3803, 0x04},
	{0x3804, 0x0a}, {0x3805, 0x3f}, {0x3806, 0x07}, {0x3807, 0x9b},
	{0x3808, 0x02}, {0x3809, 0x80}, {0x380a, 0x01}, {0x380b, 0xe0},
	{0x380c, 0x07}, {0x380d, 0x68}, {0x380e, 0x03}, {0x380f, 0xd8},
	{0x3813, 0x04},
	{0x3618, 0x00}, {0x3612, 0x29}, {0x3709, 0x52}, {0x370c, 0x03},
	{0x3a02, 0x03}, {0x3a03, 0xd8}, {0x3a14, 0x03}, {0x3a15, 0xd8},
	{0x4004, 0x02},
	{0x4713, 0x03}, {0x4407, 0x04}, {0x460c, 0x22}, {0x4837, 0x22},
	{0x3824, 0x02}, {0x5001, 0xa3}, {0x3b07, 0x0a},
	{0x503d, 0x00}, {0x3016, 0x02}, {0x301c, 0x02},
	{0x3019, 0x02}, {0x3019, 0x00}, {0x4720, 0x20},
};

struct cam_ov5640 {
	struct i2c_client *client;
	struct clk *xclk;
	struct gpio_desc *pwdn_gpio;
	struct gpio_desc *reset_gpio;
	struct mutex lock;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	struct v4l2_ctrl_handler ctrls;
	bool powered;
	bool initialized;
	bool standby;
	bool scaler_enabled;
	u16 xoff;
	u16 yoff;
};

static inline struct cam_ov5640 *to_cam_ov5640(struct v4l2_subdev *sd)
{
	return container_of(sd, struct cam_ov5640, sd);
}

static inline struct cam_ov5640 *ctrl_to_cam_ov5640(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct cam_ov5640, ctrls);
}

static void cam_ov5640_hw_reset(struct cam_ov5640 *sensor)
{
	if (sensor->pwdn_gpio)
		gpiod_set_value_cansleep(sensor->pwdn_gpio, 1);
	if (sensor->reset_gpio)
		gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	usleep_range(50000, 60000);
	if (sensor->pwdn_gpio)
		gpiod_set_value_cansleep(sensor->pwdn_gpio, 0);
	usleep_range(20000, 30000);
	if (sensor->reset_gpio)
		gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	usleep_range(50000, 60000);
}

static int cam_ov5640_write_reg(struct cam_ov5640 *sensor, u16 reg, u8 val)
{
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	int ret;
	int retry;

	for (retry = 0; retry < 3; retry++) {
		ret = i2c_master_send(sensor->client, buf, sizeof(buf));
		if (ret == sizeof(buf))
			return 0;
		usleep_range(5000, 6000);
	}

	dev_err(&sensor->client->dev,
		"I2C write failed reg=0x%04x val=0x%02x ret=%d\n",
		reg, val, ret);
	return ret < 0 ? ret : -EIO;
}

static int cam_ov5640_read_reg(struct cam_ov5640 *sensor, u16 reg, u8 *val)
{
	u8 addr[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[2] = {
		{
			.addr = sensor->client->addr,
			.flags = 0,
			.len = sizeof(addr),
			.buf = addr,
		},
		{
			.addr = sensor->client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
		},
	};
	int ret;

	ret = i2c_transfer(sensor->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret == ARRAY_SIZE(msgs))
		return 0;

	return ret < 0 ? ret : -EIO;
}

static int cam_ov5640_power_on(struct cam_ov5640 *sensor)
{
	if (sensor->powered)
		return 0;

	if (sensor->xclk) {
		int ret = clk_prepare_enable(sensor->xclk);

		if (ret)
			return ret;
	}

	cam_ov5640_hw_reset(sensor);

	sensor->powered = true;
	return 0;
}

static void cam_ov5640_power_off(struct cam_ov5640 *sensor)
{
	if (!sensor->powered)
		return;

	if (sensor->reset_gpio)
		gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	if (sensor->pwdn_gpio)
		gpiod_set_value_cansleep(sensor->pwdn_gpio, 1);
	if (sensor->xclk)
		clk_disable_unprepare(sensor->xclk);

	sensor->powered = false;
}

static int cam_ov5640_check_id(struct cam_ov5640 *sensor)
{
	u8 id_h = 0, id_l = 0;
	int ret;
	u16 chip_id;

	ret = cam_ov5640_power_on(sensor);
	if (ret)
		return ret;

	ret = cam_ov5640_read_reg(sensor, OV5640_CHIP_ID_REG, &id_h);
	if (ret)
		return ret;
	ret = cam_ov5640_read_reg(sensor, OV5640_CHIP_ID_REG + 1, &id_l);
	if (ret)
		return ret;

	chip_id = ((u16)id_h << 8) | id_l;
	if (chip_id != 0x5640) {
		dev_err(&sensor->client->dev,
			"unexpected chip id 0x%04x, expected 0x5640\n",
			chip_id);
		return -ENODEV;
	}

	dev_info(&sensor->client->dev, "OV5640 chip id 0x%04x\n", chip_id);
	return 0;
}

static int cam_ov5640_write_table(struct cam_ov5640 *sensor)
{
	size_t i;
	int ret;

	ret = cam_ov5640_power_on(sensor);
	if (ret)
		return ret;

	cam_ov5640_hw_reset(sensor);

	for (i = 0; i < ARRAY_SIZE(cam_ov5640_init_regs); i++) {
		ret = cam_ov5640_write_reg(sensor,
					   cam_ov5640_init_regs[i].addr,
					   cam_ov5640_init_regs[i].val);
		if (ret)
			return ret;

		if (cam_ov5640_init_regs[i].addr == OV5640_REG_SYS_CTRL0 &&
		    cam_ov5640_init_regs[i].val == 0x82)
			msleep(100);
		else if (cam_ov5640_init_regs[i].addr == OV5640_REG_SYS_CTRL0 &&
			 cam_ov5640_init_regs[i].val == OV5640_REG_SYS_CTRL0_SW_PWUP)
			usleep_range(20000, 30000);
		else
			usleep_range(1000, 2000);
	}

	sensor->initialized = true;
	sensor->standby = false;
	sensor->scaler_enabled = true;
	sensor->xoff = ISP_DEFAULT_XOFF;
	sensor->yoff = ISP_DEFAULT_YOFF;
	dev_info(&sensor->client->dev, "fixed VGA RGB565 mode initialized\n");
	return 0;
}

static int cam_ov5640_apply_offset(struct cam_ov5640 *sensor, u16 xoff, u16 yoff)
{
	int ret;

	if (xoff > PAN_MAX_X)
		xoff = PAN_MAX_X;
	if (yoff > PAN_MAX_Y)
		yoff = PAN_MAX_Y;

	ret = cam_ov5640_write_reg(sensor, 0x3212, 0x03);
	if (ret)
		return ret;
	ret = cam_ov5640_write_reg(sensor, 0x3810, (xoff >> 8) & 0x0f);
	if (ret)
		return ret;
	ret = cam_ov5640_write_reg(sensor, 0x3811, xoff & 0xff);
	if (ret)
		return ret;
	ret = cam_ov5640_write_reg(sensor, 0x3812, (yoff >> 8) & 0x07);
	if (ret)
		return ret;
	ret = cam_ov5640_write_reg(sensor, 0x3813, yoff & 0xff);
	if (ret)
		return ret;
	ret = cam_ov5640_write_reg(sensor, 0x3212, 0x13);
	if (ret)
		return ret;
	ret = cam_ov5640_write_reg(sensor, 0x3212, 0xa3);
	if (ret)
		return ret;

	sensor->xoff = xoff;
	sensor->yoff = yoff;
	return 0;
}

static int cam_ov5640_set_scaler(struct cam_ov5640 *sensor, bool enabled)
{
	u8 val;
	int ret;

	ret = cam_ov5640_read_reg(sensor, 0x5001, &val);
	if (ret)
		return ret;

	if (enabled)
		val |= 0x20;
	else
		val &= ~0x20;

	ret = cam_ov5640_write_reg(sensor, 0x5001, val);
	if (ret)
		return ret;

	sensor->scaler_enabled = enabled;
	return 0;
}

static int cam_ov5640_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct cam_ov5640 *sensor = ctrl_to_cam_ov5640(ctrl);
	int ret = 0;

	switch (ctrl->id) {
	case CAM_OV5640_CID_INITIALIZE:
		ret = cam_ov5640_write_table(sensor);
		break;
	case CAM_OV5640_CID_STANDBY:
		if (ctrl->val)
			ret = cam_ov5640_write_reg(sensor, OV5640_REG_SYS_CTRL0,
						   OV5640_REG_SYS_CTRL0_SW_PWDN);
		else
			ret = cam_ov5640_write_reg(sensor, OV5640_REG_SYS_CTRL0,
						   OV5640_REG_SYS_CTRL0_SW_PWUP);
		if (!ret) {
			sensor->standby = ctrl->val;
			if (!ctrl->val)
				msleep(500);
		}
		break;
	case CAM_OV5640_CID_ISP_OFFSET:
		ret = cam_ov5640_apply_offset(sensor,
					      (ctrl->val >> 16) & 0x0fff,
					      ctrl->val & 0x0fff);
		break;
	case CAM_OV5640_CID_SCALER_ENABLE:
		ret = cam_ov5640_set_scaler(sensor, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops cam_ov5640_ctrl_ops = {
	.s_ctrl = cam_ov5640_s_ctrl,
};

static const struct v4l2_ctrl_config cam_ov5640_ctrls[] = {
	{
		.ops = &cam_ov5640_ctrl_ops,
		.id = CAM_OV5640_CID_INITIALIZE,
		.name = "Initialize",
		.type = V4L2_CTRL_TYPE_BUTTON,
	},
	{
		.ops = &cam_ov5640_ctrl_ops,
		.id = CAM_OV5640_CID_STANDBY,
		.name = "Standby",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
	},
	{
		.ops = &cam_ov5640_ctrl_ops,
		.id = CAM_OV5640_CID_ISP_OFFSET,
		.name = "ISP Offset",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0x0fff0fff,
		.step = 1,
		.def = (ISP_DEFAULT_XOFF << 16) | ISP_DEFAULT_YOFF,
	},
	{
		.ops = &cam_ov5640_ctrl_ops,
		.id = CAM_OV5640_CID_SCALER_ENABLE,
		.name = "Scaler Enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 1,
	},
};

static int cam_ov5640_init_controls(struct cam_ov5640 *sensor)
{
	struct v4l2_ctrl_handler *hdl = &sensor->ctrls;
	unsigned int i;

	v4l2_ctrl_handler_init(hdl, 4);
	hdl->lock = &sensor->lock;

	for (i = 0; i < ARRAY_SIZE(cam_ov5640_ctrls); i++)
		v4l2_ctrl_new_custom(hdl, &cam_ov5640_ctrls[i], NULL);

	if (hdl->error)
		return hdl->error;

	sensor->sd.ctrl_handler = hdl;
	return 0;
}

static void cam_ov5640_init_format(struct cam_ov5640 *sensor)
{
	memset(&sensor->format, 0, sizeof(sensor->format));
	sensor->format.width = 640;
	sensor->format.height = 480;
	sensor->format.code = MEDIA_BUS_FMT_RGB565_2X8_LE;
	sensor->format.field = V4L2_FIELD_NONE;
	sensor->format.colorspace = V4L2_COLORSPACE_SRGB;
}

static int cam_ov5640_enum_mbus_code(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_mbus_code_enum *code)
{
	(void)sd;
	(void)state;

	if (code->pad != 0 || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_RGB565_2X8_LE;
	return 0;
}

static int cam_ov5640_get_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *fmt)
{
	struct cam_ov5640 *sensor = to_cam_ov5640(sd);

	if (fmt->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(sd, state, 0);

		fmt->format = *try_fmt;
	} else {
		fmt->format = sensor->format;
	}
	mutex_unlock(&sensor->lock);

	return 0;
}

static int cam_ov5640_set_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *fmt)
{
	struct cam_ov5640 *sensor = to_cam_ov5640(sd);
	struct v4l2_mbus_framefmt fixed;

	if (fmt->pad != 0)
		return -EINVAL;

	fixed = fmt->format;
	fixed.width = 640;
	fixed.height = 480;
	fixed.code = MEDIA_BUS_FMT_RGB565_2X8_LE;
	fixed.field = V4L2_FIELD_NONE;
	fixed.colorspace = V4L2_COLORSPACE_SRGB;

	mutex_lock(&sensor->lock);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(sd, state, 0);

		*try_fmt = fixed;
	} else {
		sensor->format = fixed;
	}
	mutex_unlock(&sensor->lock);

	fmt->format = fixed;
	return 0;
}

static int cam_ov5640_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct cam_ov5640 *sensor = to_cam_ov5640(sd);
	int ret;

	mutex_lock(&sensor->lock);
	if (enable) {
		if (!sensor->initialized) {
			ret = cam_ov5640_write_table(sensor);
			if (ret)
				goto out;
		}
		ret = cam_ov5640_write_reg(sensor, OV5640_REG_SYS_CTRL0,
					   OV5640_REG_SYS_CTRL0_SW_PWUP);
		if (!ret) {
			sensor->standby = false;
			msleep(500);
		}
	} else {
		ret = cam_ov5640_write_reg(sensor, OV5640_REG_SYS_CTRL0,
					   OV5640_REG_SYS_CTRL0_SW_PWDN);
		if (!ret)
			sensor->standby = true;
	}

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_video_ops cam_ov5640_video_ops = {
	.s_stream = cam_ov5640_s_stream,
};

static const struct v4l2_subdev_pad_ops cam_ov5640_pad_ops = {
	.enum_mbus_code = cam_ov5640_enum_mbus_code,
	.get_fmt = cam_ov5640_get_fmt,
	.set_fmt = cam_ov5640_set_fmt,
};

static const struct media_entity_operations cam_ov5640_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_ops cam_ov5640_subdev_ops = {
	.video = &cam_ov5640_video_ops,
	.pad = &cam_ov5640_pad_ops,
};

static int cam_ov5640_probe(struct i2c_client *client)
{
	struct cam_ov5640 *sensor;
	int ret;

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->client = client;
	mutex_init(&sensor->lock);
	sensor->scaler_enabled = true;
	sensor->xoff = ISP_DEFAULT_XOFF;
	sensor->yoff = ISP_DEFAULT_YOFF;
	cam_ov5640_init_format(sensor);

	sensor->xclk = devm_clk_get_optional(&client->dev, "xclk");
	if (IS_ERR(sensor->xclk)) {
		ret = PTR_ERR(sensor->xclk);
		goto destroy_mutex;
	}

	sensor->pwdn_gpio = devm_gpiod_get_optional(&client->dev, "powerdown",
						    GPIOD_OUT_LOW);
	if (IS_ERR(sensor->pwdn_gpio)) {
		ret = PTR_ERR(sensor->pwdn_gpio);
		goto destroy_mutex;
	}

	sensor->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset",
						     GPIOD_OUT_LOW);
	if (IS_ERR(sensor->reset_gpio)) {
		ret = PTR_ERR(sensor->reset_gpio);
		goto destroy_mutex;
	}

	v4l2_i2c_subdev_init(&sensor->sd, client, &cam_ov5640_subdev_ops);
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->sd.entity.ops = &cam_ov5640_media_ops;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto destroy_mutex;

	ret = cam_ov5640_check_id(sensor);
	if (ret)
		goto entity_cleanup;

	ret = cam_ov5640_init_controls(sensor);
	if (ret)
		goto entity_cleanup;

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto free_ctrls;

	dev_info(&client->dev, "cam ov5640 sensor subdev registered\n");
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	cam_ov5640_power_off(sensor);
destroy_mutex:
	mutex_destroy(&sensor->lock);
	return ret;
}

static int cam_ov5640_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct cam_ov5640 *sensor = to_cam_ov5640(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
	cam_ov5640_power_off(sensor);
	mutex_destroy(&sensor->lock);
	return 0;
}

static const struct of_device_id cam_ov5640_of_match[] = {
	{ .compatible = "cam-system,ov5640-control" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, cam_ov5640_of_match);

static const struct i2c_device_id cam_ov5640_id[] = {
	{ "cam-ov5640-control", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, cam_ov5640_id);

static struct i2c_driver cam_ov5640_i2c_driver = {
	.driver = {
		.name = "cam-ov5640-control",
		.of_match_table = cam_ov5640_of_match,
	},
	.id_table = cam_ov5640_id,
	.probe_new = cam_ov5640_probe,
	.remove = cam_ov5640_remove,
};

module_i2c_driver(cam_ov5640_i2c_driver);

MODULE_DESCRIPTION("Project OV5640 control-plane V4L2 subdev driver");
MODULE_AUTHOR("cam_system project");
MODULE_LICENSE("GPL");
