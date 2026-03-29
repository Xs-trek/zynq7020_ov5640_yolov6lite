#ifndef CAPTURE_H
#define CAPTURE_H

/* 采集线程入口，传入 NULL
 * 持续运行，通过 g_state.quit 退出
 * 通过 g_state.cam_enabled 暂停/恢复 */
void *capture_thread(void *arg);

#endif
