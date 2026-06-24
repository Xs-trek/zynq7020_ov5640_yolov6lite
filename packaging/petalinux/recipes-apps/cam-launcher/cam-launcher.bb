SUMMARY = "PS_KEY1 input-event launcher for cam_system"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = " \
    file://cam-launcher.c \
    file://cam-launcher \
"

S = "${WORKDIR}"
B = "${WORKDIR}/build"

inherit update-rc.d

INITSCRIPT_NAME = "cam-launcher"
INITSCRIPT_PARAMS = "start 60 5 2 . stop 40 0 1 6 ."

RDEPENDS:${PN} += "cam-system"

do_compile() {
    install -d ${B}
    ${CC} ${CFLAGS} -Wall -Wextra -Werror -std=c11 -D_GNU_SOURCE \
        ${WORKDIR}/cam-launcher.c -o ${B}/cam-launcher ${LDFLAGS}
}

do_install() {
    install -d ${D}${sbindir}
    install -m 0755 ${B}/cam-launcher ${D}${sbindir}/cam-launcher

    install -d ${D}${sysconfdir}/init.d
    install -m 0755 ${WORKDIR}/cam-launcher ${D}${sysconfdir}/init.d/cam-launcher
}

FILES:${PN} += " \
    ${sbindir}/cam-launcher \
    ${sysconfdir}/init.d/cam-launcher \
"
