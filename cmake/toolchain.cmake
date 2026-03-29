# Zynq ARM 交叉编译工具链
# 使用前必须 source PetalLinux SDK environment-setup 脚本

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# SDK 的 $CC/$CXX 格式: "编译器路径 flags..."
# CMake 只要编译器路径，flags 放到 CMAKE_C_FLAGS

# 从 $CC 中提取第一个词作为编译器路径，其余作为 flags
string(REPLACE " " ";" CC_LIST  "$ENV{CC}")
string(REPLACE " " ";" CXX_LIST "$ENV{CXX}")

list(GET CC_LIST  0 CC_PATH)
list(GET CXX_LIST 0 CXX_PATH)

list(REMOVE_AT CC_LIST  0)
list(REMOVE_AT CXX_LIST 0)

string(REPLACE ";" " " CC_FLAGS  "${CC_LIST}")
string(REPLACE ";" " " CXX_FLAGS "${CXX_LIST}")

set(CMAKE_C_COMPILER   "${CC_PATH}")
set(CMAKE_CXX_COMPILER "${CXX_PATH}")
set(CMAKE_C_FLAGS       "${CC_FLAGS}"  CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS     "${CXX_FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_SYSROOT       "$ENV{SDKTARGETSYSROOT}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
