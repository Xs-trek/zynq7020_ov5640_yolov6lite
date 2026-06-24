SUMMARY = "Project OV5640 control-plane V4L2 subdev driver"
SECTION = "kernel/modules"
LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"

SRC_URI = "file://Makefile \
           file://cam-ov5640-control.c \
           file://cam-dvp-bridge.c \
           "

S = "${WORKDIR}"
B = "${S}"

inherit module

KERNEL_MODULE_AUTOLOAD += "cam-ov5640-control cam-dvp-bridge"

do_install() {
    install -d ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${B}/cam-ov5640-control.ko ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/cam-ov5640-control.ko
    install -m 0644 ${B}/cam-dvp-bridge.ko ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/cam-dvp-bridge.ko
}

RPROVIDES:${PN} += "kernel-module-cam-ov5640-control"
RPROVIDES:${PN} += "kernel-module-cam-dvp-bridge"
