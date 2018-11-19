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

/*
 * This example shows how to
 * 1) read calibration string(s) from the sensor;
 * 2) store the calibration string(s) to the sensor.
 *
 * The default behavior is to read calibration from the sensor.
 * If -write_calib_file is used, this program will first write the specified calib.yaml
 * and then read from sensor for sanity check.
 */

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <driver/XP_sensor_driver.h>
#include <iostream>
#include <fstream>
#include <string>

DEFINE_string(calib_file, "", "The calib.yaml file path to be stored into the sensor");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  XPDRIVER::XpSensorMultithread xp_sensor("", /*empty sensor type for auto-detect*/
                                          true, /* auto_gain*/
                                          false /*imu_from_image*/);
  if (!xp_sensor.init()) {
    std::cout << "Fail to initialize xp_sensor!\n";
    return -1;
  }

  std::string calib_str_src = "";
  std::string ir_calib_str_src = "";
  if (!FLAGS_calib_file.empty()) {
    std::ifstream ifs(FLAGS_calib_file);
    std::stringstream ss;
    ss << ifs.rdbuf();
    calib_str_src = ss.str();
    xp_sensor.store_calib_to_sensor(calib_str_src);
  }

  std::string calib_str;
  if (xp_sensor.get_calib_from_sensor(&calib_str)) {
    std::cout << calib_str << "\n";
  } else {
    std::cout << "Fail to read calibration string from sensor\n";
    return -1;
  }

  // Verify the read calibration string(s) with the ones read from calib file (if provided)
  if (!calib_str_src.empty()) {
    if (calib_str_src != calib_str) {
      LOG(ERROR) << "The calib string read from sensor does NOT match what we store!!!\n";
      return -1;
    }
  }
  return 0;
}
