#ifndef TEST_H
#define TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 测试模式 */
typedef enum {
    TEST_MODE_NONE = 0,
    TEST_MODE_ISP_PATH,   /* ISP 硬编码路径移动测试 */
} test_mode_t;

/* 初始化测试模块 */
void test_init(void);

/* 启动测试 */
void test_start(test_mode_t mode);

/* 停止测试 */
void test_stop(void);

/* 是否在测试中 */
int test_is_active(void);

/* 获取当前测试模式 */
test_mode_t test_get_mode(void);

/* 每帧调用一次（在 capture 线程帧间隙）
 * time_sec: 当前时间戳（秒）
 */
void test_predict(float time_sec);

/* 获取当前测试产生的 offset（供诊断用） */
void test_get_current_offset(int *xoff, int *yoff);

#ifdef __cplusplus
}
#endif

#endif
