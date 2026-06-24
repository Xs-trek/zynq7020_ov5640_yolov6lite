SUMMARY = "High-performance neural network inference framework"
DESCRIPTION = "ncnn is a neural network inference framework optimized for mobile and embedded platforms."
HOMEPAGE = "https://github.com/Tencent/ncnn"
SECTION = "libs"

LICENSE = "BSD-3-Clause & BSD-2-Clause & Zlib"
LIC_FILES_CHKSUM = "file://LICENSE.txt;md5=d2141d1a2c978a089d3de7a67db0bbd2"

SRC_URI = "git://github.com/Tencent/ncnn.git;protocol=https;branch=master"
SRCREV = "e54f7b1f88434e1d844ea0551b880a1cfb079ce1"

S = "${WORKDIR}/git"

inherit cmake pkgconfig

EXTRA_OECMAKE += " \
    -DNCNN_VERSION=20260526 \
    -DNCNN_SHARED_LIB=OFF \
    -DNCNN_INSTALL_SDK=ON \
    -DNCNN_BUILD_TOOLS=OFF \
    -DNCNN_BUILD_EXAMPLES=OFF \
    -DNCNN_BUILD_TESTS=OFF \
    -DNCNN_BUILD_BENCHMARK=OFF \
    -DNCNN_OPENMP=OFF \
    -DNCNN_VULKAN=OFF \
    -DNCNN_SYSTEM_GLSLANG=OFF \
    -DNCNN_C_API=OFF \
    -DNCNN_PLATFORM_API=ON \
    -DNCNN_THREADS=ON \
    -DNCNN_PIXEL=ON \
    -DNCNN_PIXEL_ROTATE=OFF \
    -DNCNN_PIXEL_AFFINE=OFF \
    -DNCNN_PIXEL_DRAWING=OFF \
    -DNCNN_RUNTIME_CPU=OFF \
    -DNCNN_VFPV4=OFF \
    -DNCNN_INT8=ON \
    -DNCNN_BF16=ON \
    -DNCNN_ARM82=OFF \
    -DNCNN_ARM82DOT=OFF \
    -DNCNN_ARM82FP16FML=OFF \
    -DNCNN_ARM84BF16=OFF \
    -DNCNN_ARM84I8MM=OFF \
    "

ALLOW_EMPTY:${PN} = "1"

FILES:${PN}-dev += " \
    ${includedir}/ncnn \
    ${libdir}/cmake/ncnn \
    ${libdir}/pkgconfig/ncnn.pc \
    "
FILES:${PN}-staticdev += "${libdir}/libncnn.a"

BBCLASSEXTEND = "native nativesdk"
