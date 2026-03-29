#ifndef NET_SERVER_H
#define NET_SERVER_H

/* 启动 HTTP + WebSocket 服务器（阻塞，在主线程事件循环中运行）
 * 端口: HTTP_PORT (8080) */
int net_server_run(void);

/* 通知所有 WebSocket 客户端发送检测结果 */
void net_ws_broadcast_detections(void);

/* 停止服务器 */
void net_server_stop(void);

#endif
