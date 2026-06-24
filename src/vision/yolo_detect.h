#ifndef YOLO_DETECT_H
#define YOLO_DETECT_H

#ifdef __cplusplus
extern "C" {
#endif

void *yolo_thread(void *arg);
int   yolo_init(void);
void  yolo_close(void);

#ifdef __cplusplus
}
#endif

#endif
