SUMMARY = "Bootloader-backed A/B OTA control scripts for cam_system"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = " \
    file://cam-ota \
    file://cam-ota-bootguard \
"

S = "${WORKDIR}"

inherit update-rc.d

INITSCRIPT_NAME = "cam-ota-bootguard"
INITSCRIPT_PARAMS = "start 03 5 2 . stop 97 0 1 6 ."

do_install() {
    install -d ${D}${sbindir}
    install -m 0755 ${WORKDIR}/cam-ota ${D}${sbindir}/cam-ota

    install -d ${D}${sysconfdir}/init.d
    install -m 0755 ${WORKDIR}/cam-ota-bootguard ${D}${sysconfdir}/init.d/cam-ota-bootguard
}

FILES:${PN} += " \
    ${sbindir}/cam-ota \
    ${sysconfdir}/init.d/cam-ota-bootguard \
"
