cmake_minimum_required(VERSION 3.10)
project(cam_system C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

set(NCNN_DIR      "$ENV{HOME}/ncnn/build-arm/install")
set(JPEGTURBO_DIR "$ENV{HOME}/workdir/libjpeg-turbo/build-arm/install")

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/third_party
    ${NCNN_DIR}/include
    ${JPEGTURBO_DIR}/include
)

link_directories(
    ${NCNN_DIR}/lib
    ${JPEGTURBO_DIR}/lib
)

set(C_SOURCES
    src/hal_mmio.c
    src/hal_ov5640.c
    src/hal_frmbuf.c
    src/capture.c
    src/jpeg_enc.c
    src/net_server.c
    src/cmd_handler.c
    src/tracker.c
    src/test.c
    third_party/cJSON.c
)

set(CXX_SOURCES
    src/main.cpp
    src/yolo_detect.cpp
)

add_executable(cam_system ${C_SOURCES} ${CXX_SOURCES})

target_link_libraries(cam_system
    ncnn
    turbojpeg
    ssl
    crypto
    pthread
    dl
    m
)

install(TARGETS cam_system DESTINATION bin)
