########### 版本要求
cmake_minimum_required(VERSION 3.13)

########### 项目名称
project(AprilTag_video_)

###########  c++版本
set(CMAKE_CXX_STANDARD 14)

###########  源文件路径
aux_source_directory(. SRC_LIST)
aux_source_directory(common COMMON_SRC)
aux_source_directory(apriltag APRILTAG_SRCS)

###########  添加系统库
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

###########  添加子文件夹
include_directories(common)
include_directories(apriltag)

###########  编译库
add_library(apriltag SHARED ${APRILTAG_SRCS} ${COMMON_SRC})
target_link_libraries(apriltag Threads::Threads m)

###########  编译可执行文件
add_executable(${PROJECT_NAME} ${SRC_LIST})

###########  设置链接库
target_link_libraries(${PROJECT_NAME} apriltag ${OpenCV_LIBS})