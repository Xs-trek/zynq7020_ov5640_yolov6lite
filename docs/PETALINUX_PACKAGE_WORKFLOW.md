# cam_system PetaLinux Package 工作流

版本：0.3

## 1. 当前阶段

当前采用 A03 source package 化：`cam-system.bb` 不再打包外部预编译二进制，而是打包当前 Git 提交中的构建所需源码子集，并在 PetaLinux/Yocto 中通过 `cmake` 从 recipe sysroot 编译应用。

A03 已建立 `libjpeg-turbo`、`ncnn` 和 `cam-opencv` 组件 recipe，并完成组件级构建验证。应用 recipe 使用 `CAM_SYSTEM_DEPENDENCY_MODE=YOCTO`，通过 `DEPENDS` 引入上述 recipe 产物。由于 recipe 产物替换了历史外部二进制，首次切换时必须执行功能和性能回归；当前冻结基线后续已完成快速回归，只有变更 recipe、rootfs、boot image 或相关运行代码时才重新触发。

## 2. 标准命令

从 `cam_system` 工作空间执行：

```sh
./scripts/petalinux/stage_cam_system.sh
```

默认 PetaLinux 工程路径：

```sh
~/workdir/plnx_prj/zynq_plnx
```

如工程路径变化：

```sh
PLNX_PROJECT=/path/to/zynq_plnx ./scripts/petalinux/stage_cam_system.sh
```

同步脚本要求 `cam_system` Git 工作区干净。脚本通过 `git archive HEAD -- CMakeLists.txt assets cmake include src third_party/cjson` 生成源码 tarball，因此任何未提交修改都不会被打包，必须先提交或放弃。

## 3. Recipe 约束

同步脚本会更新：

```text
project-spec/meta-user/recipes-apps/cam-system/cam-system.bb
project-spec/meta-user/recipes-apps/cam-system/files/cam_system-src.tar.gz
project-spec/meta-user/recipes-apps/cam-system/files/cam-system-artifacts.sha256
```

当前 recipe 从源码构建应用，并由 CMake install 安装应用和模型。不再随应用包携带历史 `cam_system` 二进制、`libturbojpeg.so` 或 `libjpeg.so`。

编译期依赖由 Yocto `DEPENDS` 管理：

```bitbake
DEPENDS = "ncnn cam-opencv libjpeg-turbo openssl zlib"
```

运行期依赖由 Yocto `RDEPENDS` 管理：

```bitbake
RDEPENDS:${PN} += "libturbojpeg libssl libcrypto zlib libstdc++ libgcc"
```

A03 后，`libturbojpeg` 的来源由 meta-user 中的 `libjpeg-turbo_3.1.4.bb` 固定到官方 tag `3.1.4`。同步命令：

```sh
./scripts/petalinux/stage_libjpeg_turbo.sh
```

该脚本会把 recipe 同步到 PetaLinux 工程，并在 `project-spec/meta-user/conf/petalinuxbsp.conf` 中固定：

```bitbake
PREFERRED_VERSION_libjpeg-turbo = "3.1.4"
```

组件级验证命令：

```sh
cd ~/workdir/plnx_prj/zynq_plnx
petalinux-build -c libjpeg-turbo -x cleansstate
petalinux-build -c libjpeg-turbo
```

A03 验证结果：`do_patch`、`do_compile`、`do_populate_sysroot`、`do_package_qa` 均通过，sysroot 产物包含 `libturbojpeg.so.0.4.0` 和 `libjpeg.so.62.4.0`。

A03 还建立了 ncnn 官方 tag recipe。同步命令：

```sh
./scripts/petalinux/stage_ncnn.sh
```

该脚本会把 recipe 同步到 PetaLinux 工程，并在 `project-spec/meta-user/conf/petalinuxbsp.conf` 中固定：

```bitbake
PREFERRED_VERSION_ncnn = "20260526"
```

组件级验证命令：

```sh
cd ~/workdir/plnx_prj/zynq_plnx
petalinux-build -c ncnn -x cleansstate
petalinux-build -c ncnn
```

注意：ncnn recipe 固定官方 tag `20260526`。当前已上板验证过的历史外部二进制显示 `1.0.20260313`，但该版本号来自 ncnn CMake 构建日期，不能反推出源码 commit；因此首次切换 A03 source recipe 时必须重点检查模型加载、检测框、YOLO 开关语义和推理耗时。

A03 验证结果：`do_fetch`、`do_populate_lic`、`do_configure`、`do_compile`、`do_populate_sysroot`、`do_package_qa` 均通过。sysroot 产物包含 `libncnn.a`、`ncnn.pc`、`ncnnConfig.cmake` 和 ncnn SDK 头文件；`ncnn.pc` 显示版本 `1.0.20260526`。

A03 还建立了项目本地最小 OpenCV recipe，包名为 `cam-opencv`，避免覆盖 PetaLinux 自带 `opencv_4.5.2.bb` 及其 bbappend。同步命令：

```sh
./scripts/petalinux/stage_cam_opencv.sh
```

该脚本会把 recipe 同步到 PetaLinux 工程，并在 `project-spec/meta-user/conf/petalinuxbsp.conf` 中固定：

```bitbake
PREFERRED_VERSION_cam-opencv = "4.13.0"
```

组件级验证命令：

```sh
cd ~/workdir/plnx_prj/zynq_plnx
petalinux-build -c cam-opencv -x cleansstate
petalinux-build -c cam-opencv
```

注意：`cam-opencv` 仅构建当前 optical flow 使用的 `core,imgproc,video` 静态库。首次切换 A03 source recipe 时必须检查 tracking 选择、启动、停止、重复操作后 MJPEG 和 WS 是否仍正常。

A03 验证结果：初次构建因缺少 `zlib.h` 失败，已补充 `DEPENDS = "zlib"`；随后 `do_fetch`、`do_populate_lic`、`do_configure`、`do_compile`、`do_populate_sysroot`、`do_package_qa` 均通过。sysroot 产物包含 `libopencv_core.a`、`libopencv_imgproc.a`、`libopencv_video.a`、`libtegra_hal.a` 和 OpenCV 4.13.0 CMake config。

## 4. 运行路径

应用安装到：

```text
/usr/bin/cam_system
```

模型安装到：

```text
/usr/share/cam-system/models/yolov6lite_s/yolov6lite_s.ncnn.param
/usr/share/cam-system/models/yolov6lite_s/yolov6lite_s.ncnn.bin
```

产物校验清单安装到：

```text
/usr/share/cam-system/manifest/cam-system-artifacts.sha256
```

该路径契约与 `include/cam_system/config.h` 一致。后续若模型需要运行时在线更新，应另行讨论 `/var/lib/cam-system` 或独立 data 分区，不在本阶段直接混用。

## 5. 验证边界

本阶段完成后需要本地验证：

```sh
./scripts/petalinux/stage_cam_system.sh
petalinux-build -c cam-system
petalinux-build
```

若变更 package、rootfs、boot image 或相关运行代码，上板验证仍按既定 20 分钟以内流程执行。通过前端功能测试前，不把新 package 化镜像视为可固化状态。
