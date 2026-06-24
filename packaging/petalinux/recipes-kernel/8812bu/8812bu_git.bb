SUMMARY = "RTL8812BU/RTL8822BU USB WiFi kernel module"
SECTION = "kernel/modules"
LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://LICENSE;md5=ab842b299d0a92fb908d6eb122cd6de9"

SRC_URI = "git://github.com/morrownr/88x2bu-20210702.git;protocol=https;branch=main"
SRCREV = "fecac340fb117eb979f4bb6d28e29730384c382b"

PV = "5.13.1+git${SRCPV}"
S = "${WORKDIR}/git"
B = "${S}"

inherit module

KERNEL_MODULE_AUTOLOAD += "8812bu"
KERNEL_MODULE_PROBECONF += "8812bu"
module_conf_8812bu = "options 8812bu rtw_drv_log_level=0"

MAKE_TARGETS = "modules"
EXTRA_OEMAKE += " \
    KSRC=${STAGING_KERNEL_DIR} \
    KVER=${KERNEL_VERSION} \
    ARCH=${ARCH} \
    CROSS_COMPILE=${TARGET_PREFIX} \
    CONFIG_BT_COEXIST=n \
    CONFIG_RTW_LOG_LEVEL=0 \
    "

do_install() {
    install -d ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${B}/8812bu.ko ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/8812bu.ko
}

RPROVIDES:${PN} += "kernel-module-8812bu"
