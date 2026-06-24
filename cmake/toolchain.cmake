# Zynq ARM 交叉编译工具链
# 使用前必须 source PetalLinux SDK environment-setup 脚本

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

if("$ENV{CC}" STREQUAL "" OR "$ENV{CXX}" STREQUAL "")
    message(FATAL_ERROR "CC/CXX are not set. Source the PetaLinux SDK environment-setup script first.")
endif()

if("$ENV{SDKTARGETSYSROOT}" STREQUAL "")
    message(FATAL_ERROR "SDKTARGETSYSROOT is not set. Source the PetaLinux SDK environment-setup script first.")
endif()

function(cam_system_split_sdk_compiler env_name out_path out_flags)
    separate_arguments(_cmd UNIX_COMMAND "$ENV{${env_name}}")
    list(LENGTH _cmd _len)
    if(_len LESS 1)
        message(FATAL_ERROR "${env_name} is empty after parsing")
    endif()

    list(GET _cmd 0 _path)
    list(REMOVE_AT _cmd 0)

    set(_flags)
    foreach(_flag IN LISTS _cmd)
        if(_flag MATCHES "^--sysroot=")
            continue()
        endif()
        list(APPEND _flags "${_flag}")
    endforeach()

    string(REPLACE ";" " " _flags_str "${_flags}")
    set(${out_path} "${_path}" PARENT_SCOPE)
    set(${out_flags} "${_flags_str}" PARENT_SCOPE)
endfunction()

cam_system_split_sdk_compiler(CC CC_PATH CC_FLAGS)
cam_system_split_sdk_compiler(CXX CXX_PATH CXX_FLAGS)

set(CMAKE_C_COMPILER   "${CC_PATH}")
set(CMAKE_CXX_COMPILER "${CXX_PATH}")
set(CMAKE_C_FLAGS       "${CC_FLAGS}"  CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS     "${CXX_FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_SYSROOT       "$ENV{SDKTARGETSYSROOT}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
