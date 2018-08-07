cmake_minimum_required(VERSION 2.8.11)

# If on OSX, force cmake to use gcc-6/5 instead of the built-in gcc (AppleClang)
# The compiler overriden has to happen before calling "project"
if (APPLE)
  if (EXISTS /usr/local/bin/g++-6)
    message("Override CXX compiler to /usr/local/bin/g++-6!")
    set(CMAKE_C_COMPILER   /usr/local/bin/gcc-6)
    set(CMAKE_CXX_COMPILER /usr/local/bin/g++-6)
  elseif (EXISTS /usr/local/bin/g++-5)
    message("Override CXX compiler to /usr/local/bin/g++-5!")
    set(CMAKE_C_COMPILER   /usr/local/bin/gcc-5)
    set(CMAKE_CXX_COMPILER /usr/local/bin/g++-5)
  else ()
    message(FATAL_ERROR "Cannot find GNU gcc on Mac!")
  endif()
endif()

# External flag to define if we're compiling for the binary release
if (XP_BIN_RELEASE)
  message("Compile for XP binary release!")
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
    message("Set CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
  elseif(NOT "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    set(CMAKE_BUILD_TYPE Release)
    message("Force reset CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
  endif()
endif()

# Cross Compile
if ("$ENV{CROSS_COMPILE_FLAG}" STREQUAL "XU4")
  message("Cross Compile For XU4")
  SET(CMAKE_SYSTEM_NAME Linux)
  SET(CMAKE_SYSTEM_PROCESSOR armv7l)
  SET(CMAKE_C_COMPILER /usr/bin/arm-linux-gnueabihf-gcc)
  SET(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)
  SET(CMAKE_BUILD_TYPE Release)
endif()

project(xpdriver)

# Define the minimum supported firmware version number and generate the firmware_config.h
set(MIN_FIRMWARE_VERSION_MAJOR 0)
set(MIN_FIRMWARE_VERSION_MINOR 8)
set(MIN_FIRMWARE_VERSION_PATCH 5)
configure_file(
 cmake/firmware_config.h.in
 ${PROJECT_SOURCE_DIR}/include/driver/firmware_config.h
)

# NEON or SSE
if((CMAKE_SYSTEM_PROCESSOR MATCHES "^arm"))
  set(NEON_FLAGS "-D__ARM_NEON__ -DENABLE_NEON -mfloat-abi=hard -mfpu=neon -Ofast")
  message("Enabling neon for armv7 " ${NEON_FLAGS})
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch64")
  set(NEON_FLAGS "-D__ARM_NEON__ -DENABLE_NEON -Ofast")
  message("Enabling neon for armv8 " ${NEON_FLAGS})
else()
  # if the processor is not arm, we most likely do not care about speed.
  if (NOT XP_BIN_RELEASE)
    # Turn on debug printing etc
    add_definitions(-D__DEVELOPMENT_DEBUG_MODE__)
    # Export compile_commands.json
    set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
  endif()
  add_definitions(-mssse3)
endif()

if (CYGWIN)
  set(CMAKE_CXX_FLAGS "-std=gnu++11 -Wno-deprecated-declarations ${CMAKE_CXX_FLAGS} ${NEON_FLAGS}")
else()
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wno-deprecated-declarations ${CMAKE_CXX_FLAGS} ${NEON_FLAGS}")
endif()
set(CMAKE_C_FLAGS "-std=gnu99 -Wno-deprecated-declarations ${CMAKE_C_FLAGS} ${NEON_FLAGS}")

#TODO(le) Enable to debug with glog (require GLOG)
#add_definitions(-D__DRIVER_GLOG_DEBUG__)
find_package(OpenCV 3.0.0 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 2.4.3 QUIET)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
  endif()
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${OpenCV_INCLUDE_DIRS}
)
set(SOURCES
 src/XP_sensor.cc
 src/XP_sensor_driver.cc
 src/v4l2.cc
 src/helper/timer.cc
 src/helper/counter_32_to_64.cc
 src/helper/basic_image_utils.cc
)

set(DRIVER_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include"
 CACHE string "driver includes")

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# For binary release
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION XP/lib_${CMAKE_SYSTEM_PROCESSOR}
  RUNTIME DESTINATION XP/lib_${CMAKE_SYSTEM_PROCESSOR}  # for cygwin
  ARCHIVE DESTINATION XP/lib_${CMAKE_SYSTEM_PROCESSOR}  # for cygwin
)

# We also need to install essential headers to work together with libxpdriver.so
install(DIRECTORY include/driver DESTINATION XP/include)
