# cam_system 测试目录

本目录预留给主机侧自动化测试。

当前权威测试计划见 `docs/CAM_SYSTEM_TEST_CASES.md`。只有当测试可以在不产生板端硬件副作用的前提下执行时，才在本目录新增可执行测试。

当前主机侧测试入口：

- `scripts/build/test_ws_protocol.sh`
- `scripts/build/test_cmd_handler_route.sh`
- `scripts/build/check_project_static.sh`
