#ifndef YOLO_DETECT_H
#define YOLO_DETECT_H

#ifdef __cplusplus
extern "C" {
#endif

/* P2 阶段实现，当前为空占位 */
void *yolo_thread(void *arg);
int   yolo_init(void);
void  yolo_close(void);

#ifdef __cplusplus
}
#endif

#endif
