cmake_minimum_required(VERSION 3.10)

project(cam_system LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_SKIP_RPATH ON)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "Install prefix" FORCE)
endif()

include(GNUInstallDirs)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(CAM_SYSTEM_ENABLE_WARNINGS "Enable project warning flags" ON)
set(CAM_SYSTEM_DEPENDENCY_MODE "EXTERNAL" CACHE STRING
    "Dependency mode: EXTERNAL uses historical ARM install paths, YOCTO uses recipe sysroot packages")
set_property(CACHE CAM_SYSTEM_DEPENDENCY_MODE PROPERTY STRINGS EXTERNAL YOCTO)

set(CAM_SYSTEM_NCNN_ROOT "$ENV{HOME}/ncnn/build-arm/install"
    CACHE PATH "ncnn ARM install prefix")
set(CAM_SYSTEM_JPEGTURBO_ROOT "$ENV{HOME}/workdir/libjpeg-turbo/build-arm/install"
    CACHE PATH "libjpeg-turbo ARM install prefix")
set(CAM_SYSTEM_OPENCV_ROOT "${CMAKE_SOURCE_DIR}/third_party/opencv"
    CACHE PATH "OpenCV ARM install prefix")

set(OpenCV_DIR "${CAM_SYSTEM_OPENCV_ROOT}/lib/cmake/opencv4"
    CACHE PATH "OpenCV CMake package directory")

set(CAM_SYSTEM_NCNN_LIBRARY "${CAM_SYSTEM_NCNN_ROOT}/lib/libncnn.a"
    CACHE FILEPATH "ncnn library")
set(CAM_SYSTEM_TURBOJPEG_LIBRARY "${CAM_SYSTEM_JPEGTURBO_ROOT}/lib/libturbojpeg.so"
    CACHE FILEPATH "TurboJPEG library")

function(cam_system_require_path path description)
    if(NOT EXISTS "${path}")
        message(FATAL_ERROR "${description} not found: ${path}")
    endif()
endfunction()

find_package(Threads REQUIRED)

if(CAM_SYSTEM_DEPENDENCY_MODE STREQUAL "EXTERNAL")
    cam_system_require_path("${CAM_SYSTEM_NCNN_ROOT}/include" "ncnn include directory")
    cam_system_require_path("${CAM_SYSTEM_NCNN_LIBRARY}" "ncnn library")
    cam_system_require_path("${CAM_SYSTEM_JPEGTURBO_ROOT}/include" "libjpeg-turbo include directory")
    cam_system_require_path("${CAM_SYSTEM_TURBOJPEG_LIBRARY}" "TurboJPEG library")
    cam_system_require_path("${OpenCV_DIR}/OpenCVConfig.cmake" "OpenCVConfig.cmake")

    find_package(OpenCV REQUIRED CONFIG)

    add_library(ncnn::ncnn STATIC IMPORTED)
    set_target_properties(ncnn::ncnn PROPERTIES
        IMPORTED_LOCATION "${CAM_SYSTEM_NCNN_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${CAM_SYSTEM_NCNN_ROOT}/include"
    )

    add_library(TurboJPEG::turbojpeg UNKNOWN IMPORTED)
    set_target_properties(TurboJPEG::turbojpeg PROPERTIES
        IMPORTED_LOCATION "${CAM_SYSTEM_TURBOJPEG_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${CAM_SYSTEM_JPEGTURBO_ROOT}/include"
    )

    set(CAM_SYSTEM_NCNN_TARGET ncnn::ncnn)
    set(CAM_SYSTEM_TURBOJPEG_TARGET TurboJPEG::turbojpeg)
elseif(CAM_SYSTEM_DEPENDENCY_MODE STREQUAL "YOCTO")
    find_path(CAM_SYSTEM_YOCTO_OPENCV_INCLUDE_DIR
        NAMES opencv2/core.hpp
        PATH_SUFFIXES include/opencv4)
    find_library(CAM_SYSTEM_YOCTO_OPENCV_VIDEO_LIBRARY
        NAMES opencv_video)
    find_library(CAM_SYSTEM_YOCTO_OPENCV_IMGPROC_LIBRARY
        NAMES opencv_imgproc)
    find_library(CAM_SYSTEM_YOCTO_OPENCV_CORE_LIBRARY
        NAMES opencv_core)
    find_library(CAM_SYSTEM_YOCTO_TEGRA_HAL_LIBRARY
        NAMES tegra_hal
        PATH_SUFFIXES opencv4/3rdparty)
    find_library(CAM_SYSTEM_YOCTO_ZLIB_LIBRARY
        NAMES z)

    cam_system_require_path("${CAM_SYSTEM_YOCTO_OPENCV_INCLUDE_DIR}" "Yocto OpenCV include directory")
    cam_system_require_path("${CAM_SYSTEM_YOCTO_OPENCV_VIDEO_LIBRARY}" "Yocto OpenCV video library")
    cam_system_require_path("${CAM_SYSTEM_YOCTO_OPENCV_IMGPROC_LIBRARY}" "Yocto OpenCV imgproc library")
    cam_system_require_path("${CAM_SYSTEM_YOCTO_OPENCV_CORE_LIBRARY}" "Yocto OpenCV core library")
    cam_system_require_path("${CAM_SYSTEM_YOCTO_TEGRA_HAL_LIBRARY}" "Yocto OpenCV tegra_hal library")
    cam_system_require_path("${CAM_SYSTEM_YOCTO_ZLIB_LIBRARY}" "Yocto zlib library")

    set(OpenCV_INCLUDE_DIRS "${CAM_SYSTEM_YOCTO_OPENCV_INCLUDE_DIR}")
    set(OpenCV_LIBS
        "${CAM_SYSTEM_YOCTO_OPENCV_VIDEO_LIBRARY}"
        "${CAM_SYSTEM_YOCTO_OPENCV_IMGPROC_LIBRARY}"
        "${CAM_SYSTEM_YOCTO_OPENCV_CORE_LIBRARY}"
        "${CAM_SYSTEM_YOCTO_TEGRA_HAL_LIBRARY}"
        "${CAM_SYSTEM_YOCTO_ZLIB_LIBRARY}"
    )

    find_package(ncnn CONFIG QUIET)
    if(TARGET ncnn)
        set(CAM_SYSTEM_NCNN_TARGET ncnn)
    elseif(TARGET ncnn::ncnn)
        set(CAM_SYSTEM_NCNN_TARGET ncnn::ncnn)
    else()
        find_path(CAM_SYSTEM_YOCTO_NCNN_INCLUDE_DIR
            NAMES ncnn/net.h
            PATH_SUFFIXES include)
        find_library(CAM_SYSTEM_YOCTO_NCNN_LIBRARY
            NAMES ncnn)
        cam_system_require_path("${CAM_SYSTEM_YOCTO_NCNN_INCLUDE_DIR}" "Yocto ncnn include directory")
        cam_system_require_path("${CAM_SYSTEM_YOCTO_NCNN_LIBRARY}" "Yocto ncnn library")
        add_library(ncnn::ncnn STATIC IMPORTED)
        set_target_properties(ncnn::ncnn PROPERTIES
            IMPORTED_LOCATION "${CAM_SYSTEM_YOCTO_NCNN_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${CAM_SYSTEM_YOCTO_NCNN_INCLUDE_DIR}"
        )
        set(CAM_SYSTEM_NCNN_TARGET ncnn::ncnn)
    endif()

    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(TURBOJPEG QUIET IMPORTED_TARGET libturbojpeg)
    endif()
    if(TARGET PkgConfig::TURBOJPEG)
        set(CAM_SYSTEM_TURBOJPEG_TARGET PkgConfig::TURBOJPEG)
    else()
        find_path(CAM_SYSTEM_YOCTO_TURBOJPEG_INCLUDE_DIR
            NAMES turbojpeg.h
            PATH_SUFFIXES include)
        find_library(CAM_SYSTEM_YOCTO_TURBOJPEG_LIBRARY
            NAMES turbojpeg)
        cam_system_require_path("${CAM_SYSTEM_YOCTO_TURBOJPEG_INCLUDE_DIR}" "Yocto TurboJPEG include directory")
        cam_system_require_path("${CAM_SYSTEM_YOCTO_TURBOJPEG_LIBRARY}" "Yocto TurboJPEG library")
        add_library(TurboJPEG::turbojpeg UNKNOWN IMPORTED)
        set_target_properties(TurboJPEG::turbojpeg PROPERTIES
            IMPORTED_LOCATION "${CAM_SYSTEM_YOCTO_TURBOJPEG_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${CAM_SYSTEM_YOCTO_TURBOJPEG_INCLUDE_DIR}"
        )
        set(CAM_SYSTEM_TURBOJPEG_TARGET TurboJPEG::turbojpeg)
    endif()
else()
    message(FATAL_ERROR "Unsupported CAM_SYSTEM_DEPENDENCY_MODE=${CAM_SYSTEM_DEPENDENCY_MODE}")
endif()

set(CAM_SYSTEM_C_SOURCES
    src/platform/hal_ov5640.c
    src/platform/hal_frmbuf.c
    src/capture/capture.c
    src/media/jpeg_enc.c
    src/network/net_server.c
    src/network/ws_protocol.c
    src/network/cmd_handler.c
    src/control/tracker.c
    third_party/cjson/cJSON.c
)

set(CAM_SYSTEM_CXX_SOURCES
    src/app/main.cpp
    src/vision/yolo_detect.cpp
    src/vision/optical_flow.cpp
)

add_executable(cam_system
    ${CAM_SYSTEM_C_SOURCES}
    ${CAM_SYSTEM_CXX_SOURCES}
)

target_include_directories(cam_system
    PRIVATE
        "${CMAKE_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/third_party"
        ${OpenCV_INCLUDE_DIRS}
)

if(CAM_SYSTEM_ENABLE_WARNINGS)
    target_compile_options(cam_system
        PRIVATE
            -Wall
            -Wextra
            -Wformat=2
            -Wno-unused-parameter
    )
endif()

target_link_libraries(cam_system
    PRIVATE
        ${CAM_SYSTEM_NCNN_TARGET}
        ${CAM_SYSTEM_TURBOJPEG_TARGET}
        ${OpenCV_LIBS}
        ssl
        crypto
        Threads::Threads
        dl
        m
)

install(TARGETS cam_system RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES
    assets/models/yolov6lite_s/yolov6lite_s.ncnn.param
    assets/models/yolov6lite_s/yolov6lite_s.ncnn.bin
    DESTINATION ${CMAKE_INSTALL_DATADIR}/cam-system/models/yolov6lite_s
)
