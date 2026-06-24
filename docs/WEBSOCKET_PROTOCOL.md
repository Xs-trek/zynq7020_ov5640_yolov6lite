# WebSocket 受限协议说明

版本：0.1

阶段：A07 本地实现

适用端点：`GET /ws`

## 1. 定位

本项目的 WebSocket 代码是第一方受限协议栈，只服务当前手机前端的状态与检测框同步，不声明为通用 RFC 6455 服务器库。

A07 不新增前端 JSON 业务命令，不负责启动或停止 `cam_system` 进程，也不改变 REST API 的控制语义。

## 2. 握手边界

- 端点：`GET /ws`
- 传输：可信本地局域网内明文 HTTP upgrade。
- 认证：当前无认证、无 TLS，不得暴露到外网。
- 兼容目标：当前前端和常规浏览器 WebSocket upgrade 请求。

当前 HTTP 解析仍是项目内轻量 parser，不是通用 HTTP 服务器。

## 3. 服务端发送

服务端只发送未 mask 的 WebSocket 帧：

| 类型 | opcode | 用途 | 限制 |
| --- | --- | --- | --- |
| text | `0x1` | 发布 JSON 状态、检测框、兼容 text pong。 | payload <= 65535 bytes。 |
| pong | `0xA` | 响应客户端 control ping。 | payload <= 125 bytes。 |
| close | `0x8` | 协议错误、过大负载或正常关闭。 | payload 为 close code。 |

检测状态消息保持既有 JSON 语义：

- 必含 `status`。
- 必含 `detections`。
- tracking 有预测框时可含 `predicted_target`。
- 关闭 YOLO 后必须发布 `detections: []`，用于前端清框。

## 4. 客户端接收

客户端到服务端的帧必须符合下列子集：

| 类型 | opcode | 行为 |
| --- | --- | --- |
| text | `0x1` | payload <= 1024 bytes；仅识别兼容心跳 `{"ping": number}` 并返回 `{"pong": number}`，其它文本忽略。 |
| ping | `0x9` | payload <= 125 bytes；返回相同 payload 的 pong。 |
| pong | `0xA` | 忽略。 |
| close | `0x8` | 返回 normal close 后断开连接。 |

所有客户端帧必须 masked，这是浏览器到服务器 WebSocket 的基本要求。

## 5. 明确拒绝项

收到下列输入时服务端关闭该 WebSocket 连接：

| 输入 | close code |
| --- | --- |
| 未 masked 客户端帧 | `1002` |
| 分片帧、continuation、`FIN=0` | `1002` |
| RSV bit 非 0 | `1002` |
| binary 或未支持 opcode | `1002` |
| control frame 使用扩展长度或长度超过 125 | `1002` |
| close frame payload 长度为 1 | `1002` |
| 64-bit payload length 或 payload 超过 1024 bytes | `1009` |

## 6. 设计原因

- 当前系统的主要目标是前端状态收敛和检测框推送，不需要通用消息路由。
- network 线程是单线程 `select()` 模型，慢客户端或异常输入不得长期阻塞该线程。
- 受限 parser 可以被主机侧单元测试覆盖，比散落在事件循环中的 ad hoc 解析更容易审查。

## 7. 回归要求

涉及 `src/network/ws_protocol.*` 或 WebSocket 收发路径的变更，进入上板前必须至少执行：

```bash
./scripts/build/test_ws_protocol.sh
cmake --build build -j2
git diff --check
```

若变更影响前端可见状态，还必须执行 `CAM_SYSTEM_TEST_CASES.md` 中的 `TC-WS-001`、`TC-WS-002` 和恢复测试。
