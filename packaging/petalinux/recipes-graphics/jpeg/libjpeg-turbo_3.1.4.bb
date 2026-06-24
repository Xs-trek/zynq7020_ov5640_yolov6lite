SUMMARY = "Hardware accelerated JPEG compression/decompression library"
DESCRIPTION = "libjpeg-turbo is a JPEG codec that uses SIMD instructions to accelerate baseline JPEG compression and decompression."
HOMEPAGE = "https://libjpeg-turbo.org/"
SECTION = "libs"

LICENSE = "IJG & BSD-3-Clause & Zlib"
LIC_FILES_CHKSUM = " \
    file://LICENSE.md;md5=5e22220994831d7f0b91d09d94c89a63 \
    file://README.ijg;md5=a180facd2677ae93a773dd74a6ee57fd \
    "

SRC_URI = " \
    git://github.com/libjpeg-turbo/libjpeg-turbo.git;protocol=https;branch=main \
    file://0001-libjpeg-turbo-remove-install-rpath.patch \
    "
SRCREV = "e352b02f794f701407b39af08576035ba3360d60"

S = "${WORKDIR}/git"

PE = "1"

PROVIDES = "jpeg"
RPROVIDES:${PN} += "jpeg"
RREPLACES:${PN} += "jpeg"
RCONFLICTS:${PN} += "jpeg"

inherit cmake pkgconfig

EXTRA_OECMAKE += " \
    -DENABLE_SHARED=ON \
    -DENABLE_STATIC=OFF \
    -DWITH_TURBOJPEG=ON \
    -DWITH_JAVA=OFF \
    "

# ARM/NEON SIMD is enabled by default when the tune supports it. Disable SIMD
# only for target tunes that do not advertise NEON support.
EXTRA_OECMAKE:append:class-target:arm = " ${@bb.utils.contains('TUNE_FEATURES', 'neon', '', '-DWITH_SIMD=OFF', d)}"
EXTRA_OECMAKE:append:class-target:armeb = " ${@bb.utils.contains('TUNE_FEATURES', 'neon', '', '-DWITH_SIMD=OFF', d)}"

PACKAGES =+ "jpeg-tools libturbojpeg"

DESCRIPTION:jpeg-tools = "Client programs for accessing libjpeg functionality."
FILES:jpeg-tools = "${bindir}/*"

DESCRIPTION:libturbojpeg = "A SIMD-accelerated JPEG codec exposing the TurboJPEG API."
FILES:libturbojpeg = "${libdir}/libturbojpeg.so.*"

FILES:${PN}-dev += " \
    ${includedir}/turbojpeg.h \
    ${libdir}/libturbojpeg.so \
    ${libdir}/pkgconfig/libturbojpeg.pc \
    ${libdir}/cmake/${BPN} \
    "

BBCLASSEXTEND = "native nativesdk"
