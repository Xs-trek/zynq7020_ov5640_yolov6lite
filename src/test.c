/* test.c - ISP 平移测试框架 */
#include "test.h"
#include "tracker.h"
#include "hal_ov5640.h"
#include "config.h"
#include <stdio.h>
#include <string.h>

/* 状态变量 */
static test_mode_t g_test_mode = TEST_MODE_NONE;
static int g_test_active = 0;
static float g_test_start_time = 0.0f;
static float g_last_print_time = 0.0f;
static float g_last_call_time = 0.0f;

static int g_last_ox = -1;
static int g_last_oy = -1;

/* 辅助函数声明 */
static int clampi(int v, int lo, int hi);
static void write_pan_offset(int ox, int oy);

/* 公开接口实现框架 */

void test_init(void)
{
    /* 初始化所有状态变量 */
    g_test_mode = TEST_MODE_NONE;
    g_test_active = 0;
    g_test_start_time = 0.0f;
    g_last_print_time = 0.0f;
    g_last_call_time = 0.0f;
    g_last_ox = -1;
    g_last_oy = -1;
    printf("[test] 框架已初始化\n");
}

void test_start(test_mode_t mode)
{
    /* 开始测试模式框架 */
    if (mode == TEST_MODE_NONE) {
        test_stop();
        return;
    }
    g_test_mode = mode;
    g_test_active = 1;
    printf("[test] 测试模式 %d 已启动\n", mode);
}

void test_stop(void)
{
    /* 停止测试框架 */
    if (g_test_active) {
        printf("[test] 测试已停止\n");
    }
    g_test_active = 0;
    g_test_mode = TEST_MODE_NONE;

    printf("[test] 已恢复默认状态\n");
}

int test_is_active(void)
{
    /* 返回测试是否激活 */
    return g_test_active;
}

test_mode_t test_get_mode(void)
{
    /* 返回当前测试模式 */
    return g_test_mode;
}

void test_get_current_offset(int *xoff, int *yoff)
{
    /* 获取当前偏移量框架 */
    if (xoff) *xoff = (g_last_ox >= 0) ? g_last_ox : ISP_DEFAULT_XOFF;
    if (yoff) *yoff = (g_last_oy >= 0) ? g_last_oy : ISP_DEFAULT_YOFF;
}

void test_predict(float time_sec)
{
    /* 预测与更新框架 */
    if (!g_test_active) return;
    if (g_test_mode != TEST_MODE_ISP_PATH) return;

    printf("[test] 预测函数被调用，时间: %.2f\n", time_sec);
}

/* 辅助函数框架 */

static int clampi(int v, int lo, int hi)
{
    /* 数值钳位函数框架 */
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void write_pan_offset(int ox, int oy)
{
    /* 写入平移偏移量框架 */
    if (ox == g_last_ox && oy == g_last_oy) return;

    g_last_ox = ox;
    g_last_oy = oy;
}