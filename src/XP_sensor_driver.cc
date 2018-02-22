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
#include <driver/helper/xp_logging.h>
#include <driver/XP_sensor_driver.h>
#include <driver/v4l2.h>
#include <driver/helper/timer.h>  // for profiling timer
#include <opencv2/imgproc.hpp>
#ifdef __linux__
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <errno.h>
#include <iconv.h>
#include <linux/uvcvideo.h>
#include <fcntl.h>
#include <unistd.h>
#endif  // __linux__
#include <chrono>
#include <iostream>
#include <fstream>
#include <list>
#include <cassert>

using std::chrono::steady_clock;

namespace XPDRIVER {

#ifdef __linux__  // XP sensor driver only supports Linux for now.
XpSensorMultithread::XpSensorMultithread(const std::string& sensor_type_str,
                                         const bool use_auto_gain,
                                         const bool imu_from_image,
                                         const std::string& dev_name,
                                         const std::string& wb_mode) :
    sensor_type_str_(sensor_type_str),
    dev_name_(dev_name),
    is_running_(false),
    imu_from_image_(imu_from_image),
    use_auto_gain_(use_auto_gain),
    aec_index_updated_(false),
    aec_index_(100),
    aec_settle_(!use_auto_gain),
    use_auto_infrared_(false),
    infrared_index_updated_(false),
    infrared_index_(100),
    video_sensor_file_id_(-1),
    imaging_FPS_(25),
    wb_mode_str_(wb_mode) {
  pull_imu_rate_ = 0;
  stream_images_rate_ = 0;
}
XpSensorMultithread::~XpSensorMultithread() {
  if (is_running_) {
    this->stop();
  }
}

bool XpSensorMultithread::init(const int aec_index) {
  // TODO(mingyu): Add an is_init flag to protect from double initialization
  // TODO(mingyu): re-org v4l2_init to a better place
  if (!init_v4l2(dev_name_, &video_sensor_file_id_, &bufferinfo_)) {
    XP_LOG_ERROR(dev_name_ << " cannot be init");
    // try to turn stream off
    // TODO(mingyu): Make sure this "turn off" thing is needed
    stop_v4l2(&video_sensor_file_id_, bufferinfo_);
    return false;
  }

  XP_SENSOR::XPSensorSpec XP_sensor_spec;
  if (!XP_SENSOR::get_XP_sensor_spec(video_sensor_file_id_, &XP_sensor_spec)) {
    return false;
  }
  sensor_resolution_.RowNum = XP_sensor_spec.RowNum;
  sensor_resolution_.ColNum = XP_sensor_spec.ColNum;
  sensor_device_id_ = std::string(XP_sensor_spec.dev_id);
  sensor_soft_ver_unit_ = XP_sensor_spec.firmware_soft_version;
  if (sensor_type_str_ == "") {
    // if with input sensor_type, use auto-detected sensor_type
    sensor_type_ = XP_sensor_spec.sensor_type;
  } else {
    if (sensor_type_str_ == "XP") {
      sensor_type_ = SensorType::XP;
    } else if (sensor_type_str_ == "XP2") {
      sensor_type_ = SensorType::XP2;
    } else if (sensor_type_str_ == "XP3") {
      sensor_type_ = SensorType::XP3;
    } else if (sensor_type_str_ == "FACE") {
      sensor_type_ = SensorType::FACE;
    } else if (sensor_type_str_ == "XPIRL") {
      sensor_type_ = SensorType::XPIRL;
    } else if (sensor_type_str_ == "XPIRL2") {
      sensor_type_ = SensorType::XPIRL2;
    } else {
      XP_LOG_FATAL("Unsupported sensor type: " << sensor_type_str_);
    }
  }
  // enable or disable imu embed img funciton of firmware
  XP_SENSOR::xp_imu_embed_img(video_sensor_file_id_, imu_from_image_);

  aec_index_ = aec_index;
  constexpr bool verbose = false;  // Do NOT turn verbose on if not using the latest firmware
  XP_SENSOR::set_registers_to_default(video_sensor_file_id_,
                                      sensor_type_,
                                      aec_index_,
                                      verbose);
  // white balance
  if (sensor_type_str_ == "XP3"  ||
      sensor_type_str_ == "FACE" ||
      sensor_type_str_ == "XPIRL2") {
    whiteBalanceCorrector_.reset(new AutoWhiteBalance());
    assert(wb_mode_str_.empty() != true);
    assert(whiteBalanceCorrector_.get() != NULL);
    if (wb_mode_str_ == "auto") {
      // Don't need to do anything in auto white balance mode
      std::cout << "driver works in white balance auto mode" << std::endl;
    } else if (wb_mode_str_ == "disabled") {
      // disable white balance to get raw image
      std::cout << "driver disable white balance" << std::endl;
      whiteBalanceCorrector_->setWhiteBalancePresetMode(1.f, 1.f, 1.f);
    } else if (wb_mode_str_ == "preset") {
      // TODO(yanghongtian) : support preset mode here
      std::cout << "driver works in white balance preset mode" << std::endl;
    }
  }
  return true;
}

bool XpSensorMultithread::run() {
  if (is_running_) {
    // This sensor is already up and running.
    return false;
  }
  is_running_ = true;
  first_imu_clock_count_ = 0;  // TODO(mingyu): verify if we need to reset everytime

  thread_pool_.push_back(std::thread(&XpSensorMultithread::thread_ioctl_control, this));
  thread_pool_.push_back(std::thread(&XpSensorMultithread::thread_stream_images, this));
  if (!imu_from_image_) {
    thread_pool_.push_back(std::thread(&XpSensorMultithread::thread_pull_imu, this));
  }
  return true;
}

bool XpSensorMultithread::stop() {
  if (!is_running_) {
    // This sensor is NOT running.  Nothing to stop.
    return false;
  }
  is_running_ = false;
  for (std::thread& t : thread_pool_) {
    t.join();
  }
  stop_v4l2(&video_sensor_file_id_, bufferinfo_);
  raw_sensor_img_mmap_ptr_queue_.kill();
  return true;
}

bool XpSensorMultithread::set_image_data_callback(
    const XpSensorMultithread::ImageDataCallback& callback) {
  if (callback) {
    image_data_callback_ = callback;
    return true;
  }
  return false;
}

bool XpSensorMultithread::set_IR_data_callback(
    const XpSensorMultithread::ImageDataCallback& callback) {
  if (callback) {
    IR_data_callback_ = callback;
    return true;
  }
  return false;
}

bool XpSensorMultithread::set_imu_data_callback(
    const XpSensorMultithread::ImuDataCallback& callback) {
  if (callback) {
    imu_data_callback_ = callback;
    return true;
  }
  return false;
}

void XpSensorMultithread::thread_ioctl_control() {
  // TODO(mingyu): Put back thread param control
  XP_VLOG(1, "======== start thread_ioctl_control");

  uint8_t v4l2_buffer_cout = 0;
  while (is_running_) {
    XPDRIVER::ScopedLoopProfilingTimer loopProfilingTimer(
      "DuoVioTracker::thread_ioctl_control", 1);
    if (raw_sensor_img_mmap_ptr_queue_.size() >= V4L2_BUFFER_NUM - 1) {
      XP_LOG_ERROR("raw_sensor_img_mmap_ptr_queue_.size() = "
                 << raw_sensor_img_mmap_ptr_queue_.size()
                 << " images are used too slow");
      // [NOTE] We should NOT push more access and queue images as the img_data_ptr
      //        in mmap will wrap around, which will mess up the pointers already pushed
      //        into raw_sensor_img_mmap_ptr_queue_.
      continue;
    }
    uint8_t* img_data_ptr = nullptr;
    if (!access_next_img_and_queue_next(video_sensor_file_id_,
                                        &bufferinfo_,
                                        &img_data_ptr)) {
      continue;
    }
    // must drop beginning queue data as they are all zero.
    if (v4l2_buffer_cout <= V4L2_BUFFER_NUM) {
      v4l2_buffer_cout++;
      continue;
    }
    raw_sensor_img_mmap_ptr_queue_.push_back(img_data_ptr);
  }
  XP_VLOG(1, "======== terminate thread_ioctl_control raw_sensor_img_mmap_ptr_queue_.size() "
          << raw_sensor_img_mmap_ptr_queue_.size());
}

void XpSensorMultithread::convert_imu_axes(const XP_20608_data& imu_data,
                                           const SensorType sensor_type,
                                           XPDRIVER::ImuData* xp_imu_ptr) const {
  XPDRIVER::ImuData& xp_imu = *xp_imu_ptr;
  if (sensor_type == SensorType::XP) {
    xp_imu.accel[0] = imu_data.accel[0];
    xp_imu.accel[1] = imu_data.accel[1];
    xp_imu.accel[2] = imu_data.accel[2];
    xp_imu.ang_v[0] = imu_data.gyro[0] / 180.f * M_PI;
    xp_imu.ang_v[1] = imu_data.gyro[1] / 180.f * M_PI;
    xp_imu.ang_v[2] = imu_data.gyro[2] / 180.f * M_PI;
  } else if (sensor_type == SensorType::XP2 ||
             sensor_type == SensorType::XP3) {
    xp_imu.accel[0] = - imu_data.accel[0];
    xp_imu.accel[1] = - imu_data.accel[1];
    xp_imu.accel[2] =   imu_data.accel[2];
    xp_imu.ang_v[0] = - imu_data.gyro[0] / 180.f * M_PI;
    xp_imu.ang_v[1] = - imu_data.gyro[1] / 180.f * M_PI;
    xp_imu.ang_v[2] =   imu_data.gyro[2] / 180.f * M_PI;
  } else if (sensor_type == SensorType::FACE) {
    // TODO(mingyu): Fix the imu axes here
    xp_imu.accel[0] = - imu_data.accel[0];
    xp_imu.accel[1] = - imu_data.accel[1];
    xp_imu.accel[2] =   imu_data.accel[2];
    xp_imu.ang_v[0] = - imu_data.gyro[0] / 180.f * M_PI;
    xp_imu.ang_v[1] = - imu_data.gyro[1] / 180.f * M_PI;
    xp_imu.ang_v[2] =   imu_data.gyro[2] / 180.f * M_PI;
  } else if (sensor_type == SensorType::XPIRL2) {
    xp_imu.accel[0] = - imu_data.accel[0];
    xp_imu.accel[1] = - imu_data.accel[2];
    xp_imu.accel[2] = - imu_data.accel[1];
    xp_imu.ang_v[0] = - imu_data.gyro[0] / 180.f * M_PI;
    xp_imu.ang_v[1] = - imu_data.gyro[2] / 180.f * M_PI;
    xp_imu.ang_v[2] = - imu_data.gyro[1] / 180.f * M_PI;
  } else if (sensor_type == SensorType::XPIRL) {
    xp_imu.accel[0] = - imu_data.accel[0];
    xp_imu.accel[1] =   imu_data.accel[1];
    xp_imu.accel[2] = - imu_data.accel[2];
    xp_imu.ang_v[0] = - imu_data.gyro[0] / 180.f * M_PI;
    xp_imu.ang_v[1] =   imu_data.gyro[1] / 180.f * M_PI;
    xp_imu.ang_v[2] = - imu_data.gyro[2] / 180.f * M_PI;
  } else {
    XP_LOG_FATAL("Non-supported sensor type");
  }
}
void XpSensorMultithread::thread_pull_imu() {
  // TODO(mingyu): Put back thread param control
  const float clock_unit_ms = 1;
  Counter32To64 counter32To64(XP_CLOCK_32BIT_MAX_COUNT);
  XP_VLOG(1, "====== start thread_pull_imu ======");
  uint64_t last_clock_count_64 = 0;
  int startup_imu_count = 0;
  while (is_running_) {
    XPDRIVER::ScopedLoopProfilingTimer pull_imu_profiling_timer(
      "XpSensorMultithread::thread_pull_imu", 1);
    // sleep to get ~100 Hz rate
    std::this_thread::sleep_for(std::chrono::microseconds(9900));

    XP_20608_data imu_data;
    bool imu_access_ok = (XP_SENSOR::IMU_DataAccess(video_sensor_file_id_, &imu_data));
    if (imu_access_ok) {
      // when working in IMU pulling mode, the time stamp of the first several IMU is not stable
      // we drop the first 5 IMU frame here.
      if (startup_imu_count < 5) {
        ++startup_imu_count;
        continue;
      }
      uint64_t clock_count_wo_overflow = counter32To64.convertNewCount32(imu_data.clock_count);
      if (first_imu_clock_count_ == 0) {
        first_imu_clock_count_ = clock_count_wo_overflow;
      }
      if (last_clock_count_64 > 0) {
        if (last_clock_count_64 == clock_count_wo_overflow) {
          XP_LOG_WARNING("WARNING: Imu pulling too fast");
          continue;
        }
        // only for debug
        if (clock_count_wo_overflow < last_clock_count_64) {
          XP_LOG_FATAL(" clock_count_wo_overflow " << clock_count_wo_overflow
                     << " < last_clock_count_64 " << last_clock_count_64);
        }
        // TODO(mingyu): verify the time unit here
        if ((clock_count_wo_overflow - last_clock_count_64) * clock_unit_ms > 100) {
          XP_LOG_ERROR("IMU clock jumps. clock_count_64 " << clock_count_wo_overflow
                     << " last_clock_count_64 " << last_clock_count_64);
        }
      }
      last_clock_count_64 = clock_count_wo_overflow;
      // [NOTE] We have to flip the axes properly to align IMU coordinates with Camera L
      // The stored time_stamp is in 100us
      XPDRIVER::ImuData xp_imu;
      convert_imu_axes(imu_data, sensor_type_, &xp_imu);
      xp_imu.time_stamp = (clock_count_wo_overflow - first_imu_clock_count_) * clock_unit_ms * 10;
      if (imu_data_callback_ != nullptr) {
        imu_data_callback_(xp_imu);
      }

      ++pull_imu_count_;
      if (pull_imu_count_ > 10) {
        const int ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            steady_clock::now() - thread_pull_imu_pre_timestamp_).count();
        thread_pull_imu_pre_timestamp_ = steady_clock::now();
        pull_imu_rate_ = pull_imu_count_ * 1000 / ms;
        pull_imu_count_ = 0;
      }
    } else {
      XP_LOG_ERROR("XPDRIVER::IMU_DataAccess failed");
    }
  }
  XP_VLOG(1, "========= thread_pull_imu terminated =========");
}

void XpSensorMultithread::thread_stream_images() {
  // TODO(mingyu): Put back thread param control
  XP_VLOG(1, "======== start thread_stream_images thread");
  Counter32To64 counter32To64_img(XP_CLOCK_32BIT_MAX_COUNT);
  int frame_counter = 0;
  uint64_t last_img_count_wo_overflow_debug = 0;

  XPDRIVER::XP_SENSOR::ImuReader imu_reader;  // read IMU encoded in img

  // Compute running rate
  thread_stream_images_pre_timestamp_ = steady_clock::now();
  stream_images_count_ = 0;
  stream_images_rate_ = 0;

  // we need to copy mmap data to a buffer immediately
  // create a buffer that's twice the size of the anticipated data
  // imu data in memory
  int imu_data_pos = 0;
  const int imu_data_len = 17;
  // prevent imu data overflow
  Counter32To64 counter32To64_imu(XP_CLOCK_32BIT_MAX_COUNT);
  while (is_running_) {
    XPDRIVER::ScopedLoopProfilingTimer loopProfilingTimer(
      "XpSensorMultithread::thread_stream_images", 1);
    uint8_t* img_data_ptr = nullptr;
    if (!raw_sensor_img_mmap_ptr_queue_.wait_and_pop_front(&img_data_ptr)) {
      break;
    }
    ++frame_counter;
#ifdef __ARM_NEON__
    // since arm platform is buggy, signal the user that at least
    // 1 image is received.
    // TODO(mingyu): Does this problem still persist in our current sensor?
    if (frame_counter == 1) {
      XP_LOG_INFO("Receiving imgs" << std::endl);
    }
#endif
    // The first few imgs contain garbage data
    if (frame_counter < 4) {
      continue;
    }
    // Get IMU data if requested (only available for XP series sensor)
    if (imu_from_image_) {
      uint8_t* imu_burst_data_pos =  img_data_ptr;
      uint32_t imu_num = 0;
      // Support different versions of firmware. Old version is 25 Hz,
      // and the new version is 500 Hz, and we downsample to ~100 Hz
      if (*(imu_burst_data_pos + 16) == 0) {
        imu_num = 1;
      } else {
        imu_num = *(reinterpret_cast<uint32_t *>(imu_burst_data_pos + imu_data_len));
        // update burst imu data position.
        imu_burst_data_pos = imu_burst_data_pos + imu_data_len + 4;
      }
      pull_imu_count_ += imu_num;  // Will calculate the effective imu rate w/ image rate
      XP_20608_data imu_data;  // read IMU encoded in
      constexpr bool use_100us = false;
      for (int imu_i = 0; imu_i < imu_num; imu_i += 5) {
        if (imu_reader.get_imu_from_img(imu_burst_data_pos + imu_i * imu_data_len, &imu_data,
                                        use_100us)) {
          if (first_imu_clock_count_ == 0) {
            first_imu_clock_count_ = imu_reader.first_imu_clock_count();
            XP_VLOG(1, "Setting first_imu_clock_count_ " << first_imu_clock_count_);
          }
          // TODO(mingyu): the timestamp / clock count is so messy here...
          // Need to UNIFY
          uint64_t clock_count_wo_overflow =
              counter32To64_imu.convertNewCount32(imu_data.clock_count);

          XPDRIVER::ImuData xp_imu;
          convert_imu_axes(imu_data, sensor_type_, &xp_imu);
          // The XP clock unit is ms.  1 ms = 10 100us
          xp_imu.time_stamp = (clock_count_wo_overflow - first_imu_clock_count_) * 10;  // in 100us

          if (imu_data_callback_ != nullptr) {
            imu_data_callback_(xp_imu);
          }
        }
      }
    }

    // Start receiving image once we have received the first imu (for correct clock offset)
    if (first_imu_clock_count_ == 0) {
      continue;
    }

    uint64_t clock_count_with_overflow = 0;
    if (sensor_type_ == SensorType::XP ||
        sensor_type_ == SensorType::XP2 ||
        sensor_type_ == SensorType::XP3 ||
        sensor_type_ == SensorType::XPIRL ||
        sensor_type_ == SensorType::XPIRL2 ||
        sensor_type_ == SensorType::FACE) {
      clock_count_with_overflow = XP_SENSOR::get_timestamp_in_img(img_data_ptr + imu_data_pos);
    } else {
      XP_LOG_FATAL("Wrong sensor type");
    }
    uint64_t clock_count_wo_overflow =
        counter32To64_img.convertNewCount32(clock_count_with_overflow);
    if (last_img_count_wo_overflow_debug > clock_count_wo_overflow) {
      XP_LOG_FATAL("last_img_count_wo_overflow_debug > clock_count_wo_overflow "
                   << last_img_count_wo_overflow_debug
                   << " > " << clock_count_wo_overflow
                   << " overflow count " << counter32To64_img.getOverflowCount());
    }
    last_img_count_wo_overflow_debug = clock_count_wo_overflow;

    // [NOTE] img_time in sec
    const float clock_unit_sec = 1e-3;
    float img_time_sec =
        static_cast<float>(clock_count_wo_overflow - first_imu_clock_count_) * clock_unit_sec;
    // negative time usually suggests garbage data
    if (img_time_sec < 0) {
      continue;
    }

    // Get stereo images
    // [NOTE] The returned cv::Mat is CV_8UC1 if the sensor is mono-color,
    //        and CV_8UC3 if the sensor is color
    cv::Mat img_l, img_r;
    cv::Mat img_l_IR, img_r_IR;
    get_images_from_raw_data(img_data_ptr, &img_l, &img_r, &img_l_IR, &img_r_IR);

    // Control brightness with user input aec_index or aec (adjust every 5 frames)
    if (use_auto_gain_ && frame_counter % 5 == 3) {
      int new_aec_index = aec_index_;
      // [NOTE] Color image is converted to gray inside computeNewAecTableIndex
      if (XPDRIVER::computeNewAecTableIndex(img_l, aec_settle_, &new_aec_index)) {
        if (new_aec_index != aec_index_) {
          aec_index_ = new_aec_index;
          aec_index_updated_ = true;
        } else {
          // new_aec_index == aec_index_.  Let's mark aec settled down.
          if (!aec_settle_) {
            aec_settle_ = true;
          }
        }
      } else {
        XP_LOG_ERROR("computeNewAecTableIndex fails");
        aec_index_updated_ = false;
      }
    }

    if (aec_index_updated_) {
      aec_index_updated_ = false;  // reset
      const bool verbose = !use_auto_gain_;
      XP_SENSOR::set_aec_index(video_sensor_file_id_, aec_index_, verbose);
    }
    if (infrared_index_updated_) {
      infrared_index_updated_ = false;  // reset
      if (infrared_index_ != 0) {
        // don't set channel value, firmware can choose default channel
        XP_SENSOR::xp_infrared_ctl(video_sensor_file_id_, XP_SENSOR::pwm, 0, infrared_index_);
      } else {
        // close infrared light
        XP_SENSOR::xp_infrared_ctl(video_sensor_file_id_, XP_SENSOR::off, 0, 0);
      }
    }

    // Start to output images only if
    // 1) AEC has settled, or
    // 2) after a long waiting period, AEC still has problems to settle.
    // [NOTE] aec_settle_ is initialized to true if we are NOT using auto gain/exp control.
    if (!aec_settle_) {
      if (img_time_sec < 3.f) {
        // Keep trying to adjust aec for next iteration
        continue;
      } else {
        // Timeout and give up.  Mark aec settled down
        XP_LOG_ERROR("aec cannot settle during start up");
        aec_settle_ = true;
      }
    }

    // Intensionally process images after a short delay,
    // so that we can have IMU measurements queued up before the first image.
    if (img_time_sec <  0.05) continue;

    const float time_100us = img_time_sec * 10000;
    if (image_data_callback_ != nullptr) {
      image_data_callback_(img_l, img_r, time_100us);
    }

    if (IR_data_callback_ != nullptr && sensor_type_ == SensorType::XPIRL2) {
      IR_data_callback_(img_l_IR, img_r_IR, time_100us);
    }
    ++stream_images_count_;
    if (stream_images_count_ > 10) {
      const auto now_ts = steady_clock::now();
      const int ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now_ts - thread_stream_images_pre_timestamp_).count();
      thread_stream_images_pre_timestamp_ = now_ts;
      if (ms != 0) {
        stream_images_rate_ = stream_images_count_ * 1000 / ms;
      } else {
        stream_images_rate_ = 0;
      }
      stream_images_count_ = 0;
      if (imu_from_image_) {
        if (ms != 0) {
          pull_imu_rate_ = pull_imu_count_ * 1000 / ms;
        } else {
          pull_imu_rate_ = 0;
        }
        pull_imu_count_ = 0;
      }
    }

    XP_VLOG(1, "thread_stream_images pushed new img time " << img_time_sec);
    XP_VLOG(1, "======== thread_stream_images loop ends");
  }
  XP_VLOG(1, "======== terminate thread_stream_images");
}

bool XpSensorMultithread::set_auto_gain(const bool use_aec) {
  use_auto_gain_ = use_aec;
  return true;
}

bool XpSensorMultithread::set_aec_index(const int aec_index) {
  if (use_auto_gain_) {
    return false;
  }
  aec_index_ = aec_index;
  aec_index_updated_ = true;
  return true;
}

bool XpSensorMultithread::set_auto_infrared(const bool use_infrared) {
  use_auto_infrared_ = use_infrared;
  return true;
}

bool XpSensorMultithread::set_infrared_index(const int infrared_index) {
  if (!use_auto_infrared_) {
    return false;
  }
  infrared_index_ = infrared_index;
  infrared_index_updated_ = true;
  return true;
}

bool XpSensorMultithread::get_sensor_deviceid(std::string* device_id) {
  *device_id = sensor_device_id_;
  return true;
}

bool XpSensorMultithread::get_sensor_resolution(uint16_t* width, uint16_t* height) {
  if (sensor_resolution_.RowNum > 0 && sensor_resolution_.ColNum > 0) {
    *width = sensor_resolution_.ColNum;
    *height = sensor_resolution_.RowNum;
    return true;
  }
  return false;
}

bool XpSensorMultithread::is_color() const {
  return (sensor_type_ == SensorType::XP3 ||
          sensor_type_ == SensorType::XPIRL2 ||
          sensor_type_ == SensorType::FACE);
}

// handle XP XP2 XPIRL gray sensor image from raw data
bool XpSensorMultithread::get_v024_img_from_raw_data(const uint8_t* img_data_ptr,
                                                       cv::Mat* img_l_ptr,
                                                       cv::Mat* img_r_ptr) {
  const int row_num = sensor_resolution_.RowNum;
  const int col_num = sensor_resolution_.ColNum;
  int xp_shift_num = 0;

  zero_col_shift_detect(img_data_ptr, &xp_shift_num);
  handle_col_shift_case(const_cast<uint8_t*>(img_data_ptr), xp_shift_num);
  sensor_MT9V_image_separate(img_data_ptr, img_l_ptr, img_r_ptr);
}

// handle XP3 FACE color sensor image from raw data
bool XpSensorMultithread::get_v034_img_from_raw_data(const uint8_t* img_data_ptr,
                                                     cv::Mat* img_l_ptr,
                                                     cv::Mat* img_r_ptr) {
  const int row_num = sensor_resolution_.RowNum;
  const int col_num = sensor_resolution_.ColNum;
  cv::Mat img_l_mono(row_num, col_num, CV_8UC1);
  cv::Mat img_r_mono(row_num, col_num, CV_8UC1);

  int xp_shift_num = 0;
  zero_col_shift_detect(img_data_ptr, &xp_shift_num);
  handle_col_shift_case(const_cast<uint8_t*>(img_data_ptr), xp_shift_num);
  sensor_MT9V_image_separate(img_data_ptr, &img_l_mono, &img_r_mono);

  cv::Mat img_l_color(row_num, col_num, CV_8UC3);
  cv::Mat img_r_color(row_num, col_num, CV_8UC3);
  cv::cvtColor(img_l_mono, img_l_color, cv::COLOR_BayerGR2BGR);
  cv::cvtColor(img_r_mono, img_r_color, cv::COLOR_BayerGR2BGR);

  whiteBalanceCorrector_->run(&img_l_color);
  whiteBalanceCorrector_->run(&img_r_color);
  // FACE is basically XP3 with a special orientation configuration
  if (sensor_type_ == SensorType::FACE) {
    // TODO(mingyu): Figure out the flip / transpose used here
    cv::transpose(img_l_color, img_l_color);
    cv::flip(img_l_color, img_l_color, 1);
    cv::transpose(img_r_color, img_r_color);
    cv::flip(img_r_color, img_r_color, 0);
    *img_r_ptr = img_l_color;
    *img_l_ptr = img_r_color;
  } else {
    *img_l_ptr = img_l_color;
    *img_r_ptr = img_r_color;
  }
}

bool XpSensorMultithread::get_XPIRL2_img_from_raw_data(const uint8_t* img_data_ptr,
                                                       cv::Mat* img_l_ptr,
                                                       cv::Mat* img_r_ptr,
                                                       cv::Mat* img_l_IR_ptr,
                                                       cv::Mat* img_r_IR_ptr) {
  const int row_num = sensor_resolution_.RowNum;
  const int col_num = sensor_resolution_.ColNum;
  cv::Mat img_l_mono(row_num, col_num, CV_8UC1);
  cv::Mat img_r_mono(row_num, col_num, CV_8UC1);
  cv::Mat img_l_IR(row_num / 2 , col_num / 2, CV_8UC1);
  cv::Mat img_r_IR(row_num / 2 , col_num / 2, CV_8UC1);
  // THis function run slowly and need 65~85ms if we open auto-whitebalance
#ifndef __ARM_NEON__
  int xp_shift_num = 0;

  zero_col_shift_detect(img_data_ptr, &xp_shift_num);
  handle_col_shift_case(const_cast<uint8_t*>(img_data_ptr), xp_shift_num);
  for (int i = 0; i < row_num; ++i) {
    for (int j = 0; j < col_num; ++j) {
      if (i % 2 == 0 && j % 2 == 0) {
        img_l_IR.at<uint8_t>(i >> 1, j >> 1) =
            img_data_ptr[i * col_num * 2 + j * 2    ];
        img_r_IR.at<uint8_t>(i >> 1, j >> 1) =
            img_data_ptr[i * col_num* 2 + j* 2 + 1];
        if (i == 0 || j == 0 || i == row_num - 2 || j == col_num - 2) {
          img_l_mono.at<uint8_t>(i, j) =
              img_data_ptr[i * col_num * 2 + j * 2    ];
          img_r_mono.at<uint8_t>(i, j) =
              img_data_ptr[i * col_num * 2 + j * 2 + 1];
        } else {
          img_l_mono.at<uint8_t>(i, j) = (
            img_data_ptr[(i - 1) * col_num * 2 + (j - 1) * 2] +
            img_data_ptr[(i - 1) * col_num * 2 + (j + 1) * 2] +
            img_data_ptr[(i + 1) * col_num * 2 + (j - 1) * 2] +
            img_data_ptr[(i + 1) * col_num * 2 + (j + 1) * 2]) / 4;
          img_r_mono.at<uint8_t>(i, j) = (
              img_data_ptr[(i - 1) * col_num * 2 + (j - 1) * 2 + 1] +
              img_data_ptr[(i - 1) * col_num * 2 + (j + 1) * 2 + 1] +
              img_data_ptr[(i + 1) * col_num * 2 + (j - 1) * 2 + 1] +
              img_data_ptr[(i + 1) * col_num * 2 + (j + 1) * 2 + 1]) / 4;
        }
      } else {
        img_l_mono.at<uint8_t>(i, j) =
            img_data_ptr[i * col_num * 2 + j * 2    ];
        img_r_mono.at<uint8_t>(i, j) =
            img_data_ptr[i * col_num * 2 + j * 2 + 1];
      }
    }
  }
#else
  // TODO(yanghongtian): need add neon for XPIRL2

#endif

  cv::Mat img_l_color(row_num, col_num, CV_8UC3);
  cv::Mat img_r_color(row_num, col_num, CV_8UC3);

  cv::cvtColor(img_l_mono, img_l_color, cv::COLOR_BayerGB2BGR);
  cv::cvtColor(img_r_mono, img_r_color, cv::COLOR_BayerGB2BGR);
  // White balance need 20ms, So we need close it
  // whiteBalanceCorrector_->run(&img_l_color);
  // whiteBalanceCorrector_->run(&img_r_color);
  *img_l_IR_ptr = img_l_IR;
  *img_r_IR_ptr = img_r_IR;
  *img_l_ptr = img_l_color;
  *img_r_ptr = img_r_color;
}

bool XpSensorMultithread::get_images_from_raw_data(const uint8_t* img_data_ptr,
                                                   cv::Mat* img_l_ptr,
                                                   cv::Mat* img_r_ptr,
                                                   cv::Mat* img_l_IR_ptr,
                                                   cv::Mat* img_r_IR_ptr) {
  bool return_value = true;
  switch (sensor_type_) {
    case SensorType::XP:
    case SensorType::XP2:
    case SensorType::XPIRL:
      get_v024_img_from_raw_data(img_data_ptr, img_l_ptr, img_r_ptr);
      break;
    case SensorType::XP3:
    case SensorType::FACE:
      get_v034_img_from_raw_data(img_data_ptr, img_l_ptr, img_r_ptr);
      break;
    case SensorType::XPIRL2:
      get_XPIRL2_img_from_raw_data(img_data_ptr, img_l_ptr, img_r_ptr, img_l_IR_ptr, img_r_IR_ptr);
      break;
    default:
      std::cout << "Error sensor_type " << std::endl;
      return_value = false;
  }
  return return_value;
}

bool XpSensorMultithread::zero_col_shift_detect(const uint8_t* img_data_ptr,
                                                int* xp_shift_num) {
  const int row_num = sensor_resolution_.RowNum;
  const int col_num = sensor_resolution_.ColNum;
  static int frame_cout = 0;
  int R_start = col_num - 1;
  int L_start = 0;
  bool nonzero_det = false;
  *xp_shift_num = 0;

  frame_cout++;
  // detect right zero column
  for (int j = R_start; j > 0; --j) {
    nonzero_det = false;
    for (int i = 0.1 * row_num; i < row_num; i += 0.2 * row_num) {
      nonzero_det |= (img_data_ptr[i * col_num * 2 + j * 2 + 1] > 0);
    }
    if (nonzero_det) {
      if (*xp_shift_num == 0) {
        break;
      } else {
        if (frame_cout < 3) {
          XP_LOG_INFO("R2L_shift_num = " << *xp_shift_num);
        }
        return true;
      }
    } else {
      *xp_shift_num = col_num - j;
    }
  }
  // detect Left zero column
  for (int j = L_start; j < col_num; ++j) {
    nonzero_det = false;
    for (int i = 0.2 * row_num; i < row_num; i += 0.2 * row_num) {
      nonzero_det |= (img_data_ptr[i * col_num * 2 + j * 2 + 1] > 0);
    }
    if (nonzero_det) {
      if (*xp_shift_num == 0) {
        break;
      } else {
        if (frame_cout < 3) {
          XP_LOG_INFO("L2R_shift_num = " << *xp_shift_num);
        }
        return true;
      }
    } else {
      *xp_shift_num = -(j + 1);
    }
  }
  *xp_shift_num = 0;
  return false;
}

bool XpSensorMultithread::sensor_MT9V_image_separate(const uint8_t* img_data_ptr,
                                                     cv::Mat* img_l_ptr,
                                                     cv::Mat* img_r_ptr) {
  const int row_num = sensor_resolution_.RowNum;
  const int col_num = sensor_resolution_.ColNum;
  cv::Mat img_l_mono(row_num, col_num, CV_8UC1);
  cv::Mat img_r_mono(row_num, col_num, CV_8UC1);

  const int total_size = row_num * col_num * 2;
  uint8_t* l_row_ptr = img_l_mono.ptr();
  uint8_t* r_row_ptr = img_r_mono.ptr();
  const uint8_t* ptr_to_raw_image_data = img_data_ptr;
#ifndef __ARM_NEON__
  for (int i = 0; i < total_size; i += 2) {
    *l_row_ptr = ptr_to_raw_image_data[i];
    *r_row_ptr = ptr_to_raw_image_data[i + 1];
    l_row_ptr++;
    r_row_ptr++;
  }
#else
  for (int i = 0; i < total_size; i += 16) {
    uint8x8x2_t data = vld2_u8(ptr_to_raw_image_data + i);
    vst1_u8(l_row_ptr, data.val[0]);
    vst1_u8(r_row_ptr, data.val[1]);
    l_row_ptr += 8;
    r_row_ptr += 8;
  }
#endif
  *img_l_ptr = img_l_mono;
  *img_r_ptr = img_r_mono;
}

void XpSensorMultithread::handle_col_shift_case(uint8_t* img_data_ptr, int col_shift) {
  // the normal case
  if (col_shift == 0) {
    return;
  }

  // shift raises up
  const int row_num = sensor_resolution_.RowNum;
  const int col_num = sensor_resolution_.ColNum;

  if (col_shift < 0) {
    col_shift = -col_shift;
    for (int r = 0; r < row_num; ++r) {
      uint8_t* row_ptr = img_data_ptr + col_num * r * 2;
      int c = col_shift;
      for (; c < col_num - 8; c += 8) {
        row_ptr[2 * (c - col_shift + 0) + 1] = row_ptr[2 * (c + 0) + 1];
        row_ptr[2 * (c - col_shift + 1) + 1] = row_ptr[2 * (c + 1) + 1];
        row_ptr[2 * (c - col_shift + 2) + 1] = row_ptr[2 * (c + 2) + 1];
        row_ptr[2 * (c - col_shift + 3) + 1] = row_ptr[2 * (c + 3) + 1];
        row_ptr[2 * (c - col_shift + 4) + 1] = row_ptr[2 * (c + 4) + 1];
        row_ptr[2 * (c - col_shift + 5) + 1] = row_ptr[2 * (c + 5) + 1];
        row_ptr[2 * (c - col_shift + 6) + 1] = row_ptr[2 * (c + 6) + 1];
        row_ptr[2 * (c - col_shift + 7) + 1] = row_ptr[2 * (c + 7) + 1];
      }
      for (; c < col_num; ++c) {
        row_ptr[2 * (c - col_shift) + 1] = row_ptr[2 * c + 1];
      }

      for (c = c - col_shift; c < col_num; ++c) {
        row_ptr[2 * c + 1] = 255;
      }
    }
  } else {
    for (int r = 0; r < row_num; ++r) {
      uint8_t* row_ptr = img_data_ptr + col_num * r * 2;
      int c = col_num - 1;
      uint8_t img_data_tmp[8];
      for (; c >= col_shift + 8; c -= 8) {
        img_data_tmp[0] = row_ptr[2 * (c - col_shift - 0) + 1];
        img_data_tmp[1] = row_ptr[2 * (c - col_shift - 1) + 1];
        img_data_tmp[2] = row_ptr[2 * (c - col_shift - 2) + 1];
        img_data_tmp[3] = row_ptr[2 * (c - col_shift - 3) + 1];
        img_data_tmp[4] = row_ptr[2 * (c - col_shift - 4) + 1];
        img_data_tmp[5] = row_ptr[2 * (c - col_shift - 5) + 1];
        img_data_tmp[6] = row_ptr[2 * (c - col_shift - 6) + 1];
        img_data_tmp[7] = row_ptr[2 * (c - col_shift - 7) + 1];
        row_ptr[2 * (c - 0) + 1] = img_data_tmp[0];
        row_ptr[2 * (c - 1) + 1] = img_data_tmp[1];
        row_ptr[2 * (c - 2) + 1] = img_data_tmp[2];
        row_ptr[2 * (c - 3) + 1] = img_data_tmp[3];
        row_ptr[2 * (c - 4) + 1] = img_data_tmp[4];
        row_ptr[2 * (c - 5) + 1] = img_data_tmp[5];
        row_ptr[2 * (c - 6) + 1] = img_data_tmp[6];
        row_ptr[2 * (c - 7) + 1] = img_data_tmp[7];
      }
      for (; c >= col_shift; --c) {
        row_ptr[2 * c + 1] = row_ptr[2 * (c - col_shift) + 1];
      }
      for (; c >= 0; --c) {
        row_ptr[2 * c + 1] = 255;
      }
    }
  }
}
#endif  // __linux__
}  // namespace XPDRIVER
