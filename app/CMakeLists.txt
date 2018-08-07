cmake_minimum_required(VERSION 2.8.11)
project(boteye_demo)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/") 
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
# NEON or SSE
if((CMAKE_SYSTEM_PROCESSOR MATCHES "^arm"))
  set(NEON_FLAGS "-D__ARM_NEON__ -DENABLE_NEON -D__ODROID_XU4__ -mfloat-abi=hard -mfpu=neon -Ofast")
  message("Enabling neon for armv7 " ${NEON_FLAGS})
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch64")
  set(NEON_FLAGS "-D__ARM_NEON__ -DENABLE_NEON -D__RK3399__ -Ofast")
  message("Enabling neon for armv8 " ${NEON_FLAGS})
else()
  add_definitions(-mssse3)
endif()


set (CMAKE_CXX_FLAGS "-std=c++11 -Wno-deprecated-declarations ${CMAKE_CXX_FLAGS} ${NEON_FLAGS}")
add_definitions(-D__DRIVER_NONE_RELEASE__)
find_package(OpenCV 3.0.0 REQUIRED)
find_package(Glog REQUIRED)
find_package(GFlags REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})
find_library(DEPTH_LIB
NAME xpdepth 
HINTS "${PROJECT_SOURCE_DIR}/libs"
)
find_library(PARAM_LIB
NAME xpparam 
HINTS "${PROJECT_SOURCE_DIR}/libs"
)

find_library(DRIVER_LIB
NAME xpdriver
HINTS "${PROJECT_SOURCE_DIR}/../build"
)

include_directories(${PROJECT_SOURCE_DIR}/../include/)
include_directories(${PROJECT_SOURCE_DIR}/include/)
set(SOURCE_DEPTH_APPS
depth_demo.cpp
)

add_executable(depth_demo ${SOURCE_DEPTH_APPS})
target_link_libraries(depth_demo ${OpenCV_LIBS} glog gflags ${DRIVER_LIB} ${DEPTH_LIB} ${PARAM_LIB})

include_directories(${PROJECT_SOURCE_DIR}/../include/)
set(SOURCE_APPS
driver_demo.cpp
)

add_executable(driver_demo ${SOURCE_APPS})
target_link_libraries(driver_demo ${OpenCV_LIBS} glog gflags ${DRIVER_LIB})

add_subdirectory(tests)
