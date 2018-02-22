cmake_minimum_required(VERSION 2.8.11)
project(driver_demo)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/") 
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
set (CMAKE_CXX_FLAGS "-std=c++11 -Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")
add_definitions(-D__DRIVER_NONE_RELEASE__)
find_package(OpenCV 3.0.0 REQUIRED)
find_package(Glog REQUIRED)
find_package(GFlags REQUIRED)
find_library(DRIVER_LIB
NAME xpdriver
HINTS "${PROJECT_SOURCE_DIR}/../build"
)
include_directories(${PROJECT_SOURCE_DIR}/../include/)
set(SOURCE_APPS
driver_demo.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_APPS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} glog gflags ${DRIVER_LIB})
