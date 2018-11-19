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

# get ubuntu release version
if (EXISTS /etc/os-release)
execute_process(COMMAND bash "-c" "awk '/^VERSION_ID=/' /etc/os-release | cut -c13-17"
    OUTPUT_VARIABLE RELEASE_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
else()
  message(STATUS "====Can not find /etc/os-release!")
endif()
message(STATUS "RELEASE_VERSION=${RELEASE_VERSION}")



set (CMAKE_CXX_FLAGS "-std=c++11 -Wno-deprecated-declarations ${CMAKE_CXX_FLAGS} ${NEON_FLAGS}")
add_definitions(-D__DRIVER_NONE_RELEASE__)
find_package(OpenCV 3.0.0 REQUIRED)
find_package(Glog REQUIRED)
find_package(GFlags REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

find_library(DRIVER_LIB
NAME xpdriver
HINTS "${PROJECT_SOURCE_DIR}/../build"
)
find_library(PARAM_LIB
  NAME xpparam
  HINTS "${PROJECT_SOURCE_DIR}/libs_${CMAKE_SYSTEM_PROCESSOR}_${RELEASE_VERSION}"
)

include_directories(${PROJECT_SOURCE_DIR}/../include/)
include_directories(${PROJECT_SOURCE_DIR}/include/)

add_executable(driver_demo driver_demo.cpp)
target_link_libraries(driver_demo ${OpenCV_LIBS} glog gflags ${DRIVER_LIB})

add_executable(calib_io_demo calib_io_demo.cpp)
target_link_libraries(calib_io_demo glog gflags ${DRIVER_LIB})

add_executable(calib_convert_to_half calib_convert_to_half.cpp)
target_link_libraries(calib_convert_to_half glog gflags ${OpenCV_LIBS} ${DRIVER_LIB} ${PARAM_LIB})

add_subdirectory(depth_demo)
add_subdirectory(tests)
