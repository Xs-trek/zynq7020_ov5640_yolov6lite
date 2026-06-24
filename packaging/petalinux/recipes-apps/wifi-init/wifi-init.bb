SUMMARY = "Interactive WiFi init script"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://S50wifi"

S = "${WORKDIR}"

inherit update-rc.d

INITSCRIPT_NAME = "S50wifi"
INITSCRIPT_PARAMS = "start 50 5 2 . stop 20 0 1 6 ."

RDEPENDS:${PN} += "wpa-supplicant wpa-supplicant-cli wpa-supplicant-passphrase"

do_install() {
    install -d ${D}${sysconfdir}/init.d
    install -m 0755 ${WORKDIR}/S50wifi ${D}${sysconfdir}/init.d/S50wifi
}
