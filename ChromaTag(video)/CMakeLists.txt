########### 版本要求
cmake_minimum_required(VERSION 3.13)

########### 项目名称
project(ChromaTag)

###########  c++版本
set(CMAKE_CXX_STANDARD 14)

###########  源文件路径
aux_source_directory(. SRC_LIST)

###########  添加库
# opncv库
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
    add_definitions(-DOPENCV_FOUND)
endif()

## lcm库
#include(FindPkgConfig)
#pkg_check_modules(LCM QUIET lcm)
#if(lcm_FOUND)
#    add_definitions(-DLCM_FOUND)
#    include_directories(${LCM_INCLUDE_DIRS})
#    link_directories(${LCM_DIRS})
#    set(Dependencies ${Dependencies} ${LCM_LIBRARIES})
#endif()

###########  添加子文件夹
include_directories(Utilities)
include_directories(Utilities/Common)
include_directories(Utilities/LCM_Types)
include_directories(Utilities/Math)
include_directories(Utilities/Vision)
include_directories(Detector)

add_subdirectory(Utilities)
add_subdirectory(Detector)

###########  设置可执行文件
add_executable(${PROJECT_NAME} ${SRC_LIST})

###########  设置链接库
target_link_libraries(${PROJECT_NAME} JMD_ChromaTag Writer Options Timer ${Dependencies})