cmake_minimum_required(VERSION 2.8.11)
project(tests)

include_directories(${PROJECT_SOURCE_DIR}/../include/)
set(SOURCE_APPS
  white_balance_test.cpp
)

add_executable(white_balance_test ${SOURCE_APPS})
target_link_libraries(white_balance_test
  ${OpenCV_LIBS}
  glog
  gflags
  ${DRIVER_LIB}
)

add_executable(ring_buffer_test ring_buffer_test.cpp)
