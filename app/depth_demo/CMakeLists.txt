cmake_minimum_required(VERSION 2.8.11)
project(depth_demo)

if ("${RELEASE_VERSION}" STREQUAL "16.04" OR "${RELEASE_VERSION}" STREQUAL "14.04")
  find_library(DEPTH_LIB
    NAME xpdepth 
    HINTS "${PROJECT_SOURCE_DIR}/../libs_${CMAKE_SYSTEM_PROCESSOR}_${RELEASE_VERSION}"
  )
  find_library(PARAM_LIB
    NAME xpparam 
    HINTS "${PROJECT_SOURCE_DIR}/../libs_${CMAKE_SYSTEM_PROCESSOR}_${RELEASE_VERSION}"
  )

  include_directories(${PROJECT_SOURCE_DIR}/../include/)
  include_directories(${PROJECT_SOURCE_DIR}/include/)
  set(SOURCE_DEPTH_APPS
    depth_demo.cpp
  )

  add_executable(${PROJECT_NAME} ${SOURCE_DEPTH_APPS})
  target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    glog
    gflags
    ${DRIVER_LIB}
    ${DEPTH_LIB}
    ${PARAM_LIB}
  )
else()
  message(STATUS "no supported ubuntu release version found: ${RELEASE_VERSION}, will not compile depth_demo")
endif()
