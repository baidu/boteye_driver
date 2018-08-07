/******************************************************************************
 * Copyright 2017-2018 Baidu Robotic Vision Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <glog/logging.h>
#include <driver/XP_sensor_driver.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#ifdef __ARM_NEON__
DEFINE_int32(cpu_core, 4, "bind program to run on specific core[0 ~ 7],"
             "being out-of-range indicates no binding, only valid on ARM platform");
#endif

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

#ifdef __ARM_NEON__
  if (FLAGS_cpu_core >= 0 && FLAGS_cpu_core < 8) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(FLAGS_cpu_core, &set);

    if (0 != sched_setaffinity(getpid(), sizeof(cpu_set_t), &set)) {
      std::cout << "sched_setaffinity return non-zero code" << std::endl;
      exit(1);
    }
  }
#endif  // __ARM_NEON__
  const int width = 640;
  const int height = 480;
  XPDRIVER::AutoWhiteBalance awb;
  for (int n = 0; n < 20; ++n) {
    cv::Mat img(height, width, CV_8UC3);

    cv::randu(img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

#ifdef __ARM_NEON__
    cv::Mat img_neon(height, width, CV_8UC3);
    img.copyTo(img_neon);
#endif

    awb.run_original(&img);

#ifdef __ARM_NEON__
    awb.run(&img_neon);
    for (int i = 0; i < height; ++i) {
      const uchar* img_ptr = img.ptr(i);
      const uchar* img_neon_ptr = img_neon.ptr(i);
      for (int j = 0; j < width; ++j) {
        CHECK_NEAR(*(img_ptr + 0), *(img_neon_ptr + 0), 1);
        CHECK_NEAR(*(img_ptr + 1), *(img_neon_ptr + 1), 1);
        CHECK_NEAR(*(img_ptr + 2), *(img_neon_ptr + 2), 1);
        img_ptr += 3;
        img_neon_ptr += 3;
      }
    }
#endif
  }

  return 0;
}
