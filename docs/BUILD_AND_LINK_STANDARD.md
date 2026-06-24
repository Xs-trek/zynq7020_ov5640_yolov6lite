# cam_system 编译与链接规范

版本：0.4

## 1. 目标

本文档约束 `cam_system` 用户态程序的编译、链接和安装入口。目标是让同一份源码在 PetaLinux SDK 环境下可重复构建，并在 PetaLinux/Yocto recipe 中从受管 sysroot 依赖构建。

## 2. 构建入口

标准构建入口为：

```sh
./scripts/build/build_arm.sh
```

脚本默认使用：

```sh
~/workdir/plnx_prj/zynq_plnx/images/linux/sdk/environment-setup-cortexa9t2hf-neon-xilinx-linux-gnueabi
```

如 SDK 路径变化，使用：

```sh
PETALINUX_SDK_ENV=/path/to/environment-setup-cortexa9t2hf-neon-xilinx-linux-gnueabi ./scripts/build/build_arm.sh
```

如需指定构建目录，使用：

```sh
BUILD_DIR=/tmp/cam_system-build ./scripts/build/build_arm.sh
```

## 3. CMake 约束

- 不使用全局 `include_directories()` 和 `link_directories()` 扩散依赖。
- 所有 include 路径通过 `target_include_directories(cam_system PRIVATE ...)` 声明。
- ncnn 和 TurboJPEG 以 imported target 形式进入链接图。
- OpenCV 通过 `find_package(OpenCV REQUIRED CONFIG)` 获取。
- pthread 通过 `find_package(Threads REQUIRED)` 和 `Threads::Threads` 获取。
- C/C++ 标准固定为 C11 / C++11。
- 默认开启 `-Wall -Wextra -Wformat=2 -Wno-unused-parameter`，但当前阶段不启用全局 `-Werror`。
- `CAM_SYSTEM_DEPENDENCY_MODE` 必须显式区分依赖来源：
  - `EXTERNAL`：本地 SDK 调试构建，继续使用历史 ARM install 路径。
  - `YOCTO`：PetaLinux recipe 构建，只从 recipe sysroot 查找 ncnn、OpenCV、TurboJPEG。

## 4. 三方依赖路径

默认本地调试构建使用 `CAM_SYSTEM_DEPENDENCY_MODE=EXTERNAL`，路径如下：

| 依赖 | CMake cache 变量 | 默认值 |
| --- | --- | --- |
| ncnn | `CAM_SYSTEM_NCNN_ROOT` | `$HOME/ncnn/build-arm/install` |
| libjpeg-turbo | `CAM_SYSTEM_JPEGTURBO_ROOT` | `$HOME/workdir/libjpeg-turbo/build-arm/install` |
| OpenCV | `CAM_SYSTEM_OPENCV_ROOT` | `${CMAKE_SOURCE_DIR}/third_party/opencv` |

可通过 CMake 参数覆盖，例如：

```sh
./scripts/build/build_arm.sh -DCAM_SYSTEM_NCNN_ROOT=/opt/ncnn-arm
```

PetaLinux recipe 构建使用：

```cmake
-DCAM_SYSTEM_DEPENDENCY_MODE=YOCTO
```

该模式不读取历史外部路径，而是通过 `DEPENDS = "ncnn cam-opencv libjpeg-turbo openssl zlib"` 进入的 Yocto recipe sysroot 查找依赖。

## 5. Toolchain 约束

`cmake/toolchain.cmake` 只接受已经 source 过 PetaLinux SDK 的环境。

它从 SDK 提供的 `CC`、`CXX`、`SDKTARGETSYSROOT` 中提取：

- 编译器路径
- CPU/ABI/安全加固 flags
- sysroot

SDK `CC/CXX` 中的 `--sysroot=...` 会被移除，统一交给 `CMAKE_SYSROOT` 管理，避免命令行重复注入 sysroot。

## 6. 链接策略

本地调试构建链接策略保持冻结基线语义：

- ncnn 使用静态库 `libncnn.a`。
- TurboJPEG 使用共享库 `libturbojpeg.so`，与当前板端运行库部署方式一致。
- OpenCV 使用当前 `third_party/opencv` 内的静态库。
- OpenSSL 仍以 `ssl`、`crypto` 形式链接。
- 构建产物不得携带宿主机绝对 RPATH；板端运行库解析交给 `/usr/lib`、rootfs 或后续 package 管理。

PetaLinux recipe 构建链接策略：

- ncnn 使用 `ncnn` recipe 生成的静态库和 SDK 头文件。
- OpenCV 使用项目本地 `cam-opencv` recipe 生成的 `core,imgproc,video` 静态库；YOCTO 模式不消费 OpenCVConfig，避免导入其它 recipe sysroot 的绝对路径。
- TurboJPEG 使用 `libjpeg-turbo` recipe 生成的 `libturbojpeg` shared library。
- OpenSSL、zlib、C++ runtime 由 Yocto sysroot 和 runtime package 提供。

## 7. 安装目标

CMake install 当前提供两个安装目标：

- `cam_system` -> `${CMAKE_INSTALL_BINDIR}`，目标 rootfs 通常为 `/usr/bin`
- `yolov6lite_s.ncnn.param/bin` -> `${CMAKE_INSTALL_DATADIR}/cam-system/models/yolov6lite_s`，目标 rootfs 通常为 `/usr/share/cam-system/models/yolov6lite_s`

当前用户态程序通过 `include/cam_system/config.h` 固定读取：

```text
/usr/share/cam-system/models/yolov6lite_s/yolov6lite_s.ncnn.param
/usr/share/cam-system/models/yolov6lite_s/yolov6lite_s.ncnn.bin
```

源码 recipe 构建时，CMake install 结果由 `cam-system.bb` 安装到 rootfs。临时 `/tmp/cam_system_agent` 仅允许作为调试手段，不是 A03 package 化路径。

## 8. 验证命令

编译/链接规范化变更接受前必须执行：

```sh
git diff --check
./scripts/build/build_arm.sh
cppcheck --enable=warning,style,performance,portability --inconclusive --std=c++11 --force -Iinclude -Isrc -Ithird_party --suppress=missingIncludeSystem --suppress=unusedFunction src include
```

已知静态分析偏差仍以 `CAM_SYSTEM_SOFTWARE_TOP_LEVEL_SPEC.md` 为准。

## 9. ELF 产物合同

`scripts/build/verify_elf.sh` 是 a04 后的强制产物验证入口。它在两个路径中执行：

- `./scripts/build/build_arm.sh` 本地 SDK 构建结束后。
- `cam-system.bb` 的 `do_compile:append()` 中，作为 recipe `files/verify_elf.sh` 构建期工具执行。

该脚本属于开发/构建阶段工具，不属于应用运行时内容。`stage_cam_system.sh` 会把它复制到 PetaLinux recipe `files/`，但不会把它放入 `cam_system-src.tar.gz`，也不会安装到目标 rootfs。

验证内容：

- 产物必须是 ARM 32-bit ELF。
- ELF attribute 必须显示 ARMv7、NEON、hard-float ABI。
- 禁止出现 Cortex-A9 不支持的 VFPv4/ARMv8 FP attribute。
- 禁止携带 `RPATH` 或 `RUNPATH`。
- 动态依赖必须落在当前 rootfs/recipe 可解释集合内：动态加载器、`libturbojpeg`、OpenSSL、zlib、C/C++ runtime、glibc 基础库。

如果未来确实新增运行库，必须先更新 `cam-system.bb` 的 `RDEPENDS`，再通过 `CAM_SYSTEM_VERIFY_ALLOW_EXTRA_NEEDED` 或脚本白名单显式记录原因。禁止让未知 `NEEDED` 静默进入镜像。
