SUMMARY = "Project-local minimal OpenCV build"
DESCRIPTION = "A static OpenCV build limited to the core, imgproc, and video modules used by cam_system optical flow."
HOMEPAGE = "https://opencv.org/"
SECTION = "libs"

DEPENDS = "zlib"

LICENSE = "Apache-2.0"
LIC_FILES_CHKSUM = "file://LICENSE;md5=3b83ef96387f14655fc854ddc3c6bd57"

SRC_URI = "git://github.com/opencv/opencv.git;protocol=https;branch=4.x"
SRCREV = "fe38fc608f6acb8b68953438a62305d8318f4fcd"

S = "${WORKDIR}/git"

inherit cmake pkgconfig

EXTRA_OECMAKE += " \
    -DBUILD_LIST=core,imgproc,video \
    -DBUILD_SHARED_LIBS=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_DOCS=OFF \
    -DBUILD_opencv_apps=OFF \
    -DBUILD_opencv_world=OFF \
    -DBUILD_JAVA=OFF \
    -DBUILD_FAT_JAVA_LIB=OFF \
    -DBUILD_opencv_python2=OFF \
    -DBUILD_opencv_python3=OFF \
    -DBUILD_opencv_python_bindings_generator=OFF \
    -DBUILD_opencv_java_bindings_generator=OFF \
    -DBUILD_opencv_js_bindings_generator=OFF \
    -DBUILD_opencv_objc_bindings_generator=OFF \
    -DWITH_1394=OFF \
    -DWITH_ADE=OFF \
    -DWITH_CUDA=OFF \
    -DWITH_EIGEN=OFF \
    -DWITH_FFMPEG=OFF \
    -DWITH_GSTREAMER=OFF \
    -DWITH_GTK=OFF \
    -DWITH_IPP=OFF \
    -DWITH_ITT=OFF \
    -DWITH_JASPER=OFF \
    -DWITH_JPEG=OFF \
    -DWITH_OPENCL=OFF \
    -DWITH_OPENEXR=OFF \
    -DWITH_OPENJPEG=OFF \
    -DWITH_OPENMP=OFF \
    -DWITH_PNG=OFF \
    -DWITH_PROTOBUF=OFF \
    -DWITH_QT=OFF \
    -DWITH_TBB=OFF \
    -DWITH_TIFF=OFF \
    -DWITH_V4L=OFF \
    -DWITH_WEBP=OFF \
    -DWITH_CAROTENE=ON \
    -DENABLE_CCACHE=OFF \
    -DENABLE_FAST_MATH=OFF \
    -DENABLE_NEON=ON \
    -DENABLE_PRECOMPILED_HEADERS=OFF \
    -DOPENCV_ENABLE_ALLOCATOR_STATS=OFF \
    -DOPENCV_GENERATE_PKGCONFIG=OFF \
    "

do_install:append() {
    rm -rf ${D}${bindir}
    rm -rf ${D}${datadir}/opencv4
    rm -rf ${D}${datadir}/licenses/opencv4
}

ALLOW_EMPTY:${PN} = "1"

FILES:${PN}-dev += " \
    ${includedir}/opencv4 \
    ${libdir}/cmake/opencv4 \
    "
FILES:${PN}-staticdev += " \
    ${libdir}/libopencv_core.a \
    ${libdir}/libopencv_imgproc.a \
    ${libdir}/libopencv_video.a \
    ${libdir}/opencv4/3rdparty/*.a \
    "
