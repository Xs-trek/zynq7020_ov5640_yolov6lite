# cam_system 测试用例

版本：1.0
适用文档：`CAM_SYSTEM_SOFTWARE_TOP_LEVEL_SPEC.md` v1.4
状态：V4L2-only 正式清理版冻结门禁；测试矩阵仍覆盖 WebSocket 受限协议栈

## 1. 测试策略

每个测试用例的结果必须记录为：

- PASS
- FAIL
- BLOCKED，并说明原因

板端测试默认针对 rootfs/package 安装的 `/usr/bin/cam_system` 执行。临时测试二进制只允许作为明确记录的开发例外，不作为冻结门禁或固化依据。

测试过程中不得手工修改 kernel、PL、device tree、boot 文件或模型文件。A06 允许通过 OTA 更新受控 `image.ub`，该镜像内含项目 recipe 生成的 device tree 和 rootfs。

单次上板测试从启动被测程序开始计时，必须在 20 分钟内结束并恢复到 camera off / YOLO off。若任一步骤阻塞超过预算，立即执行恢复测试并停止本轮，不继续追加测试。

历史依据：`logs/board/20260527-144925-runtime-ab/CONCLUSIONS.md` 记录的 PS-only YOLO baseline 约为 346 ms/帧，capture/JPEG 不是主瓶颈。因此冻结门禁只做短窗口功能回归，不做长时间性能 soak test。

## 1.1 冻结门禁执行顺序和时间预算

| 顺序 | 用例 | 预算 | 说明 |
| --- | --- | --- | --- |
| 1 | TC-BRD-001 | 2 分钟 | 只确认网络、SSH、模型文件。 |
| 2 | TC-BRD-002、TC-BRD-003A、TC-BRD-003B、TC-BRD-003C、TC-BRD-004、TC-BRD-005 | 5 分钟 | 确认 package 安装、PS_KEY1 启动入口、线程亲和性和 OV5640 V4L2 控制面。 |
| 3 | TC-REST-001、TC-REST-002、TC-REST-003 | 3 分钟 | API 和 settings 边界。 |
| 4 | TC-MJPG-001、TC-MJPG-002 | 3 分钟 | MJPEG 连通与慢客户端背压回归。 |
| 5 | TC-WS-001、TC-WS-002 | 3 分钟 | WS 状态收敛、YOLO off 清框。 |
| 6 | 前端人工快速冒烟 | 3 分钟 | camera on/off、YOLO on/off、断开重连。 |
| 7 | TC-REC-001 | 2 分钟 | 恢复到前端可接管状态。 |

总预算 20 分钟。若需要观察真实检测框，必须提前准备画面中有 person 的场景；否则 `detections=[]` 不能判为 YOLO 失败。

## 2. 主机侧检查

### TC-HOST-001 编译检查

覆盖需求：NFR-008、CR-009  
步骤：

1. 执行 `git status --short`。
2. 执行 `cmake --build build -j2`。

期望结果：

- 编译成功。
- 工作区变更是预期变更，并可通过 `git status` 识别。

### TC-HOST-002 diff 格式检查

覆盖需求：CR-009  
步骤：

1. 执行 `git diff --check`。

期望结果：

- 无空白符或 patch 格式错误。

### TC-HOST-003 静态分析

覆盖需求：CR-008、NFR-008  
步骤：

1. 执行：
   `cppcheck --enable=warning,style,performance,portability --suppress=missingIncludeSystem --inline-suppr --quiet src include`
2. 若执行基于 `build/compile_commands.json` 的全项目 cppcheck，需要把 `third_party/cjson/cJSON.c` 告警按顶层规格说明 `DEV-001` 记录为三方偏差。

期望结果：

- 一方用户态源码不出现新增告警。
- 全项目检查除顶层规格说明中记录的三方偏差 `DEV-001` 外，不出现新增告警。

### TC-HOST-004 PS_KEY1 launcher 本地验证

覆盖需求：IF-009、FR-013、NFR-008
步骤：

1. 执行 `./scripts/build/validate_cam_launcher.sh`。
2. 执行 `./scripts/petalinux/stage_cam_launcher.sh`。
3. 执行 `petalinux-build -c device-tree`。
4. 执行 `petalinux-build -c cam-launcher`。

期望结果：

- host 侧 `cam-launcher.c` 严格告警编译通过。
- init 脚本 shell 语法检查通过。
- device-tree 片段包含 PS_KEY1/MIO10、低有效、`KEY_PROG1=148`、20 ms 消抖。
- PetaLinux device-tree 和 `cam-launcher` recipe 构建通过。

### TC-HOST-005 WebSocket 受限协议栈本地验证

覆盖需求：IF-007、NFR-011、CR-011
步骤：

1. 执行 `./scripts/build/test_ws_protocol.sh`。

期望结果：

- masked text、126 扩展长度、ping、close 解析通过。
- unmasked、分片、binary、超大 payload 被拒绝。
- 服务端 text header 和扩展长度 header 生成符合当前受限协议说明。

## 3. 板端准备

### TC-BRD-001 连通性检查

覆盖需求：NFR-006  
步骤：

1. 确认板子已上电并完成登录。
2. 执行 `ping -c 1 -W 1 <board-ip>`。
3. 通过 SSH 确认 `whoami`、`hostname` 和模型文件。

期望结果：

- ping 成功。
- SSH 可以作为调试态 `debug` 用户或项目配置的等价非 root 用户登录，且不依赖串口交互。
- `/usr/share/cam-system/models/yolov6lite_s/yolov6lite_s.ncnn.param` 和 `.bin` 存在。

### TC-BRD-002 Package 安装检查

覆盖需求：NFR-006  
步骤：

1. 确认 `/usr/bin/cam_system` 存在且可执行。
2. 确认 `/usr/share/cam-system/manifest/cam-system-artifacts.sha256` 存在。
3. 确认 `/usr/share/cam-system/models/yolov6lite_s/` 下 `.param` 和 `.bin` 存在。
4. 未经明确要求，不复制临时程序覆盖 package 安装结果。

期望结果：

- `/usr/bin/cam_system` 存在且可执行。
- package manifest 和模型文件存在。
- 未修改 kernel、PL、device tree、boot 文件。

### TC-BRD-003A cam-launcher 安装和服务状态

覆盖需求：IF-009、FR-013
步骤：

1. 确认 `/usr/sbin/cam-launcher` 存在且可执行。
2. 确认 `/etc/init.d/cam-launcher` 存在且可执行。
3. 执行 `/etc/init.d/cam-launcher status`。
4. 确认 `pidof cam_system` 初始无输出。

期望结果：

- `cam-launcher` 正在运行。
- `cam_system` 默认不运行，等待按键或显式启动。

### TC-BRD-003B gpio-keys input 设备检查

覆盖需求：IF-009、DEV-006
步骤：

1. 读取 `/proc/bus/input/devices`。
2. 读取 `/tmp/cam-launcher.log`。
3. 读取 `/proc/interrupts | grep cam-ps-key1` 并记录初始计数。
4. 短按一次 PS_KEY1。
5. 再次读取 `/proc/interrupts | grep cam-ps-key1`。
6. 必要时读取 `dmesg | grep -Ei 'gpio-keys|cam-ps-key1|input|pinctrl'`。

期望结果：

- 存在 `gpio-keys` input 设备。
- 设备能力包含 `KEY_PROG1` 对应的 key capability。
- `cam-launcher` 日志显示正在监听某个 `/dev/input/eventX`。
- `cam-ps-key1` 中断计数在按键后增加。

若本用例失败，不继续按键启动测试，应先定位 MIO10 pinmux、device tree 是否进入 FIT、`gpio-keys` probe 是否失败。

### TC-BRD-003C PS_KEY1 启动程序

覆盖需求：FR-001、IF-001、IF-009
步骤：

1. 确认 `pidof cam_system` 初始无输出。
2. 用户短按一次 PS_KEY1。
3. 等待最多 5 秒后检查 `pidof cam_system`。
4. 请求 `GET /api/status`。
5. 记录当前 PID，再短按一次 PS_KEY1。
6. 再次检查 `pidof cam_system`。

期望结果：

- 第一次按键后 `cam_system` 进程正在运行。
- HTTP server 在 `8080` 端口响应。
- 初始状态包含 `cam_enabled=false`、`yolo_enabled=false`、`tracking=false`。
- `status` 中不得包含 `zoom` 字段。
- 第二次按键不产生第二个 `cam_system` 进程，原 PID 不变化。

### TC-BRD-003D 显式启动程序兜底

覆盖需求：FR-001、IF-001  
步骤：

1. 停止旧的 `cam_system` 进程。
2. 用 sudo 启动：
   `sudo nohup /usr/bin/cam_system > /tmp/cam_system.log 2>&1 &`
3. 请求 `GET /api/status`。

期望结果：

- 进程正在运行。
- HTTP server 在 `8080` 端口响应。
- 初始状态包含 `cam_enabled=false`、`yolo_enabled=false`、`tracking=false`。
- `status` 中不得包含 `zoom` 字段。

### TC-BRD-004 线程亲和性检查

覆盖需求：NFR-009、7.1 线程划分
步骤：

1. 在 TC-BRD-003C 或 TC-BRD-003D 启动后读取 `/tmp/cam_system.log`。
2. 读取 `/proc/$(pidof cam_system)/task/*/status` 中的 `Cpus_allowed_list`。

期望结果：

- 日志包含 `main/network thread bound to Core 0`。
- 日志包含 `capture/control thread created on Core 0`。
- 日志包含 `YOLO thread created on Core 1`。
- `/proc` 中存在一个只允许 CPU1 的线程，且其它 `cam_system` 线程只允许 CPU0。

### TC-BRD-005 OV5640 V4L2 控制面检查

覆盖需求：FR-001、DEV-008、NFR-008
步骤：

1. 确认 `cam_ov5640_control` 模块已加载。
2. 确认 `/dev/v4l-subdev0` 或其它 `v4l-subdevX` 存在，且 name 包含 `cam-ov5640-control`。
3. 启动 `cam_system`，读取日志或 `dmesg`。
4. 执行一次 camera on、短时 `/stream` 采样、camera off。
5. 停止 `cam_system`，再启动一次并重复第 4 步。

期望结果：

- `cam_system` 日志包含 `Using V4L2 control subdev` 和 `V4L2 control backend initialized`。
- `dmesg` 至少出现一次 `fixed VGA RGB565 mode initialized`。
- 第二次启动后仍能收到 MJPEG body，不能停在一帧或连接中。
- 程序必须通过 V4L2 控制面初始化；若 V4L2 subdev 缺失或初始化失败，程序应 fail fast，不得回退到旧用户态 I2C/GPIO 实现。
- 测试结束必须 camera off；若不继续前端测试，应停止 `cam_system`。

## 4. REST API 测试

### TC-REST-001 POST body 完整接收

覆盖需求：IF-002、IF-003  
步骤：

1. 使用 `curl -H 'Content-Type: application/json' -d ...` 发送 `POST /api/camera`，body 为 `{"enabled":true}`。
2. 使用同样方式发送 `POST /api/yolo`，body 为 `{"enabled":true}`。

期望结果：

- 两个请求都返回 HTTP 200 和 `{"status":"ok"}`。
- 不出现 `missing enabled` 错误。

### TC-REST-002 启用后状态检查

覆盖需求：IF-001、FR-012  
步骤：

1. 执行 TC-REST-001 后，请求 `GET /api/status`。

期望结果：

- `cam_enabled=true`。
- `yolo_enabled=true`。

### TC-REST-003 settings 边界检查

覆盖需求：FR-007、FR-008、FR-008A  
步骤：

1. `POST /api/settings`，body 为 `{"max_det":-1}`。
2. `POST /api/settings`，body 为 `{"max_det":1000}`。
3. `POST /api/settings`，body 为 `{"classes":[]}`。
4. `POST /api/settings`，body 为 `{"classes":[0]}`。
5. `POST /api/settings`，body 为 `{"classes":[999]}`。

期望结果：

- 步骤 1 到 4 返回 HTTP 200。
- 步骤 3 不得导致 YOLO 解码直接关闭；其语义为不做类别过滤。
- 步骤 5 返回 HTTP 400，并包含 invalid settings 错误。
- 所有请求后程序仍可响应。

### TC-REST-004 命令队列顺序

覆盖需求：FR-006、tracking 生命周期规则  
步骤：

1. 在 YOLO 启用且尽量已有检测结果后，发送：
   `POST /api/track {"action":"select","target_id":0}`
2. 立即发送：
   `POST /api/track {"action":"start"}`
3. 立即发送：
   `POST /api/track {"action":"stop"}`
4. 至少等待 1 秒后查询状态。

期望结果：

- 已接受请求返回 HTTP 200，除非命令队列已满。
- 程序不崩溃。
- 最终状态 `tracking=false`。

### TC-REST-005 正式版拒绝开发期入口

覆盖需求：IF-005、FR-012、CR-009
步骤：

1. 发送 `POST /api/track {"action":"test"}`。
2. 发送 `POST /api/tune {"omega_stable":1}`。

期望结果：

- 步骤 1 返回 HTTP 400，并包含 invalid action 错误。
- 步骤 2 返回 HTTP 404，并包含 unknown endpoint 错误。
- 程序不启动任何 ISP path test 或 tune 行为。

## 5. 前端可见状态测试

### TC-WS-001 YOLO 开启后发布检测或空帧

覆盖需求：IF-007、FR-003、FR-012  
步骤：

1. WebSocket 客户端连接 `ws://<board-ip>:8080/ws`。
2. 开启 camera 和 YOLO。
3. 等待最多 10 秒。

期望结果：

- WebSocket 连接保持。
- 消息包含 `status` 和 `detections`。
- `status` 中不得包含 `zoom` 字段。
- 若画面中没有目标，`detections` 为空是可接受结果。

### TC-WS-002 YOLO 关闭后清空检测框

覆盖需求：FR-009、FR-010、YOLO off 生命周期规则  
步骤：

1. 开启 camera 和 YOLO。
2. 等待前端或 WebSocket 客户端收到至少一条包含 `detections` 字段的消息。
3. 发送 `POST /api/yolo {"enabled":false}`。
4. 在最多 2 秒内观察 WebSocket 消息。

期望结果：

- 状态最终显示 `yolo_enabled=false`。
- WebSocket 收到 `detections: []`。
- 前端画面不再残留旧检测框。
- YOLO 关闭后的 in-flight 推理结果不得再次发布旧检测。

### TC-WS-003 tracking 停止后清空 predicted target

覆盖需求：FR-011  
步骤：

1. 若存在有效检测目标，启动 tracking。
2. 确认 WebSocket 中出现 `predicted_target`。
3. 发送 `POST /api/track {"action":"stop"}`。
4. 观察 WebSocket 和 status。

期望结果：

- 状态显示 `tracking=false`。
- 后续 WebSocket 消息不再包含 `predicted_target`。

## 6. MJPEG 测试

### TC-MJPG-001 视频流连通性

覆盖需求：IF-006  
步骤：

1. 开启 camera。
2. 执行：
   `curl --max-time 3 -D /tmp/cam_mjpeg_headers.txt -o /tmp/cam_mjpeg_sample.bin http://<board-ip>:8080/stream`
3. 检查 header 和 sample 大小。

期望结果：

- header 包含 HTTP 200 和 `multipart/x-mixed-replace`。
- sample 大小大于 0 字节。

### TC-MJPG-002 慢客户端不影响 REST

覆盖需求：NFR-003  
步骤：

1. 打开 `/stream` 并保持连接，或让其慢速超时。
2. 并行请求 `GET /api/status`。
3. 慢客户端保持时间不得超过 10 秒。

期望结果：

- REST status 仍可响应。
- 程序不退出。
- MJPEG 不得因一次正常 `EAGAIN/EWOULDBLOCK` 直接被服务端判定为失败。

## 7. 恢复测试

### TC-REC-001 恢复到前端可接管状态

覆盖需求：生命周期规则  
步骤：

1. 发送 `POST /api/track {"action":"stop"}`。
2. 发送 `POST /api/settings`，classes 为 `0..79`，`max_det=20`。
3. 发送 `POST /api/yolo {"enabled":false}`。
4. 发送 `POST /api/camera {"enabled":false}`。
5. 请求 `GET /api/status`。

期望结果：

- `cam_enabled=false`。
- `yolo_enabled=false`。
- `tracking=false`。
- `target_id=null`。
- 前端可通过原 API 正常接管。

## 8. 测试用例自审

覆盖性检查：

- 主机侧编译/静态检查由 TC-HOST-001 到 TC-HOST-003 覆盖。
- WebSocket 受限 parser 由 TC-HOST-005 覆盖。
- 用户态部署边界由 TC-BRD-002 覆盖。
- POST body 接收回归由 TC-REST-001 覆盖。
- 输入边界由 TC-REST-003 覆盖。
- 命令队列行为由 TC-REST-004 覆盖。
- 之前遗漏的前端检测框残留 bug 由 TC-WS-002 覆盖。
- MJPEG 流由 TC-MJPG-001 和 TC-MJPG-002 覆盖。
- 前端接管状态由 TC-REC-001 覆盖。
- PS_KEY1 启动入口由 TC-HOST-004、TC-BRD-003A、TC-BRD-003B、TC-BRD-003C 覆盖。

已知缺口：

- 仓库内已有 WebSocket 帧 parser 自动化测试，但尚未提供完整浏览器前端自动化测试。
- 前端 overlay 视觉断言仍依赖人工观察或外部前端测试。
- 本测试集不覆盖 CPU 负载和推理延迟回归。
- 本测试集不覆盖长时间 soak test。
- 本地测试无法证明真实物理输入链路，必须由 TC-BRD-003B/003C 上板验证。

建议：

- 后续凡是影响前端可见状态的变更，必须通过 TC-WS-002 或等价自动化 WebSocket 测试后才能接受。
