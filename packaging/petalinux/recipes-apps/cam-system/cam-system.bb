SUMMARY = "Camera system with YOLO detection"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://cam_system-src.tar.gz \
           file://cam-system-artifacts.sha256 \
           file://verify_elf.sh \
           "

S = "${WORKDIR}/cam_system-src"

inherit cmake pkgconfig

DEPENDS = "ncnn cam-opencv libjpeg-turbo openssl zlib"

EXTRA_OECMAKE += " \
    -DCAM_SYSTEM_DEPENDENCY_MODE=YOCTO \
    -DCAM_SYSTEM_ENABLE_WARNINGS=ON \
    -DCMAKE_SKIP_RPATH=ON \
    "

RDEPENDS:${PN} += "libturbojpeg libssl libcrypto zlib libstdc++ libgcc"

do_compile:append() {
    ${WORKDIR}/verify_elf.sh ${B}/cam_system
}

CAM_SYSTEM_MODEL_DIR = "${datadir}/cam-system/models/yolov6lite_s"
CAM_SYSTEM_MANIFEST_DIR = "${datadir}/cam-system/manifest"

do_install:append() {
    install -d ${D}${CAM_SYSTEM_MANIFEST_DIR}
    install -m 0644 ${WORKDIR}/cam-system-artifacts.sha256 ${D}${CAM_SYSTEM_MANIFEST_DIR}/cam-system-artifacts.sha256
}

FILES:${PN} += " \
    ${bindir}/cam_system \
    ${CAM_SYSTEM_MODEL_DIR}/yolov6lite_s.ncnn.param \
    ${CAM_SYSTEM_MODEL_DIR}/yolov6lite_s.ncnn.bin \
    ${CAM_SYSTEM_MANIFEST_DIR}/cam-system-artifacts.sha256 \
    "
