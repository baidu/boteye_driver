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
 * This program is used to 
 * 1) read calibration string(s) from the sensor or file;
 * 2) convert calibration file to half scale;
 * 3) store the converted calib file to record path.
 *
 */

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <driver/XP_sensor_driver.h>
#include <XP/helper/param.h>
#include <iostream>
#include <fstream>
#include <string>

DEFINE_string(calib_yaml, "", "The calib.yaml file path to be converted");
DEFINE_string(dev_name, "", "which video dev name to open. Empty enables auto mode");
DEFINE_string(save_path, "", "The path to store converted calib file");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  XP::DuoCalibParam src_calib_param;
  XP::DuoCalibParam dst_calib_param;
  if (FLAGS_save_path.empty()) {
    LOG(ERROR) << "You must set save_path to save file!";
    return -1;
  }

  if (!FLAGS_calib_yaml.empty()) {
    std::cout << "Load calib from " << FLAGS_calib_yaml << std::endl;
    if (!src_calib_param.LoadCamCalibFromYaml(FLAGS_calib_yaml)) {
      LOG(ERROR) << FLAGS_calib_yaml << " cannot be loaded";
      return -1;
    }
  } else {
    XPDRIVER::XpSensorMultithread xp_sensor("", /*empty sensor type for auto-detect*/
                                            true, /* auto_gain*/
                                            false, /*imu_from_image*/
                                            FLAGS_dev_name);
    if (!xp_sensor.init()) {
      std::cout << "Fail to initialize xp_sensor!\n";
      return -1;
    }
    std::string calib_str = "";
    std::cout << "Loading calib file from sensor...\n";
    if (xp_sensor.get_calib_from_sensor(&calib_str)) {
      std::cout << "Load success!\n";
      src_calib_param.LoadFromString(calib_str);
    } else {
      LOG(ERROR) << "Load from sensor failed!\n";
      return -1;
    }
  }
  dst_calib_param = src_calib_param;
  dst_calib_param.ConvertToHalfScale();
  if (!dst_calib_param.WriteToYaml(FLAGS_save_path)) {
    LOG(ERROR) << "Save to " << FLAGS_save_path << " failed!";
  } else {
    std::cout << "Save to " << FLAGS_save_path << " success!\n";
  }

  return 0;
}
