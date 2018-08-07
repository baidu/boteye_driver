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
#include <driver/helper/shared_queue.h>
#include <driver/xp_aec_table.h>
#include <driver/AR0141_aec_table.h>
#include <driver/XP_sensor_driver.h>
#include <XP/helper/param.h>
#include <XP/depth/depth_utils.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>
#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#ifdef __linux__
#include <sys/stat.h>
#endif

using std::cout;
using std::endl;
using std::vector;
using XPDRIVER::XpSensorMultithread;
using std::chrono::steady_clock;
using XPDRIVER::SensorType;
DEFINE_bool(auto_gain, false, "turn on auto gain");
DEFINE_string(wb_mode, "preset", "white balance mode: auto, disabled, preset");
DEFINE_string(dev_name, "", "which video dev name to open. Empty enables auto mode");
DEFINE_string(calib_yaml, "", "load calib file");
DEFINE_string(ir_calib_yaml, "", "load IR calib file");
DEFINE_string(depth_param_yaml, "", "load depth config file");
DEFINE_bool(depth, false, "whether or not show depth image");
DEFINE_bool(ir_depth, false, "whether or not show ir depth image");
DEFINE_string(sensor_type, "", "XP or XP2 or XP3 or FACE or XPIRL or XPIRL2, XPIRL3");
DEFINE_bool(spacebar_mode, false, "only save img when press space bar");
DEFINE_string(record_path, "", "path to save images. Set empty to disable saving");
DEFINE_int32(ir_period, 2, "One IR image in every ir_period frames. 0: all RGBs, 1: all IRs,"
            " 2: RGB-IR, 3: RGB-RGB-IR");
#ifdef __ARM_NEON__
DEFINE_int32(cpu_core, 4, "bind program to run on specific core[0 ~ 7],"
             "being out-of-range indicates no binding, only valid on ARM platform");
#endif

struct V4l2BufferData {
  int counter;
  std::shared_ptr<vector<uint8_t>> img_data_ptr;
  V4l2BufferData() {
    img_data_ptr.reset(new vector<uint8_t>);
  }
};
struct ImgForSave {
  cv::Mat l;
  cv::Mat r;
  cv::Mat xyz;
  std::string name;
};
struct StereoImage {
  cv::Mat l;
  cv::Mat r;
  float ts_100us;
};
struct ImgForShow {
  cv::Mat image;
  std::string image_name;
  std::mutex image_show_mutex;
};
XPDRIVER::shared_queue<ImgForSave> imgs_for_saving_queue("imgs_for_saving_queue");
XPDRIVER::shared_queue<ImgForSave> IR_imgs_for_saving_queue("IR_imgs_for_saving_queue");
XPDRIVER::shared_queue<StereoImage> IR_depth_queue("IR_depth_queue");
XPDRIVER::shared_queue<StereoImage> stereo_image_queue("stereo_image_queue");
XPDRIVER::shared_queue<StereoImage> IR_image_queue("IR_image_queue");
std::atomic<bool> run_flag;
std::atomic<bool> save_img, save_ir_img;
SensorType XP_sensor_type;
// we use the first imu to approx img time based on img counter
std::atomic<bool> g_auto_gain;
XPDRIVER::XP_SENSOR::infrared_mode_t g_infrared_mode;
int g_aec_index;  // signed int as the index may go to negative while calculation
int g_infrared_index;
cv::Size g_img_size;
ImgForShow g_img_lr_display, g_img_lr_IR_display;
ImgForShow g_depth_canvas;
bool g_has_IR;

// The unique instance of XpSensorMultithread
std::unique_ptr<XPDRIVER::XpSensorMultithread> g_xp_sensor_ptr;

void image_thread_safe_copy(ImgForShow* image_show, const cv::Mat& raw_image) {
  std::lock_guard<std::mutex> lock(image_show->image_show_mutex);
  raw_image.copyTo(image_show->image);
}

void image_thread_safe_show(ImgForShow* image_show) {
  std::lock_guard<std::mutex> lock(image_show->image_show_mutex);
  imshow(image_show->image_name, image_show->image);
}

bool check_file_exist(const std::string file_name) {
#ifdef __linux__
  struct stat buf;
  return (stat(file_name.c_str(), &buf) == 0);
#else
  std::cout << "Function check_file_exist not implemented under this environment!\n";
  return false;
#endif
}

bool create_directory(const std::string dir_name) {
#ifdef __linux__
  const int dir_err = mkdir(dir_name.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if (-1 == dir_err) {
      std::cout << "Error creating directory!\n";
      return false;
  }
  return true;
#else
  std::cout << "Function create_directory not implemented under this environment!\n";
  return false;
#endif
}

// Callback functions for XpSensorMultithread
// [NOTE] These callback functions have to be light-weight as it *WILL* block XpSensorMultithread
void image_data_callback(const cv::Mat& img_l, const cv::Mat& img_r, const float ts_100us,
                         const std::chrono::time_point<std::chrono::steady_clock>& sys_time) {
  if (run_flag) {
    StereoImage stereo_img;
    stereo_img.l = img_l;
    stereo_img.r = img_r;
    stereo_img.ts_100us = ts_100us;
    stereo_image_queue.push_back(stereo_img);
  }
}

void IR_data_callback(const cv::Mat& img_l, const cv::Mat& img_r, const float ts_100us,
                      const std::chrono::time_point<std::chrono::steady_clock>& sys_time) {
  if (run_flag) {
    StereoImage IR_img;
    IR_img.l = img_l;
    IR_img.r = img_r;
    IR_img.ts_100us = ts_100us;
    IR_image_queue.push_back(IR_img);
    if (FLAGS_ir_depth &&
        (XP_sensor_type == SensorType::XPIRL2 || XP_sensor_type == SensorType::XPIRL3)) {
      IR_depth_queue.push_back(IR_img);
    }
  }
}

cv::Mat_<cv::Vec3f> g_depth_xyz_img;
cv::Mat g_disparity_img;
cv::Mat g_disparity_buf;  // for filterSpeckles
vector<cv::Mat> g_disparity_ml;

void visualize_depth(const XP::DuoCalibParam& calib_param,
                     bool save_img,
                     cv::Mat* depth_canvas) {
  CHECK_EQ(g_disparity_img.type(), CV_16SC1);
  CHECK_GT(g_disparity_img.rows, 0);
  CHECK_GT(g_disparity_img.cols, 0);
  // compute xyz img for display or saving
  if (save_img) {
    if (g_depth_xyz_img.rows == 0) {
      g_depth_xyz_img.create(g_disparity_img.rows, g_disparity_img.cols);
    }
    g_depth_xyz_img.setTo(cv::Vec3f(0, 0, 0));
    for (int y = 0; y < g_disparity_img.rows; ++y) {
      for (int x = 0; x < g_disparity_img.cols; ++x) {
        int16_t disp = g_disparity_img.at<int16_t>(y, x);
        if (disp <= 0) continue;
        // http://docs.opencv.org/3.0.0/d9/d0c/group__calib3d.html#ga1bc1152bd57d63bc524204f21fde6e02
        // [XYZW]T=𝚀∗[x y 𝚍𝚒𝚜𝚙𝚊𝚛𝚒𝚝𝚢(x,y) 1]T
        cv::Vec4f xyz_homo;
        if (FLAGS_depth)
          xyz_homo = calib_param.Camera.Q *
                     cv::Vec4f(x, y, static_cast<float>(disp) / 16.f, 1);
        cv::Vec3f xyz_C(xyz_homo[0] / xyz_homo[3],
                        xyz_homo[1] / xyz_homo[3],
                        xyz_homo[2] / xyz_homo[3]);
        g_depth_xyz_img.at<cv::Vec3f>(y, x) = xyz_C;
      }
    }
  }
  if (depth_canvas->size() != g_disparity_img.size()) {
    depth_canvas->create(g_disparity_img.size(), CV_8UC3);
  }
  for (int i = 0; i < g_disparity_img.rows; ++i) {
    for (int j = 0; j < g_disparity_img.cols; ++j) {
      depth_canvas->at<cv::Vec3b>(i, j) = XP::depth16S2color(g_disparity_img.at<int16_t>(i, j));
    }
  }
}

void process_stereo_depth(const XP::DuoCalibParam& calib_param,
                          const cv::Mat& img_l_mono,
                          const cv::Mat& img_r_mono,
                          const bool save_img,
                          cv::Mat* depth_canvas) {
  // Sanity check
  CHECK_EQ(img_l_mono.channels(), 1);
  CHECK_EQ(img_r_mono.channels(), 1);
  CHECK_EQ(img_l_mono.type(), CV_8U);
  CHECK_EQ(img_r_mono.type(), CV_8U);
  XP::multilevel_stereoBM(calib_param,
                          img_l_mono,
                          img_r_mono,
                          &g_disparity_img,
                          &g_disparity_ml,
                          0,
                          2,
                          &g_disparity_buf);
  visualize_depth(calib_param, save_img, depth_canvas);
}

void process_stereo_ir_depth(const XP::DuoCalibParam& rgb_calib_param,
                             const XP::DuoCalibParam& ir_calib_param,
                             const cv::Mat& img_l_ir,
                             const cv::Mat& img_r_ir,
                             const cv::Mat& img_l_rgb,
                             const bool save_img,
                             cv::Mat* depth_canvas) {
  // Sanity check
  CHECK_EQ(img_l_rgb.channels(), 1);
  CHECK_EQ(img_l_rgb.type(), CV_8U);
  CHECK_EQ(img_l_ir.channels(), 1);
  CHECK_EQ(img_r_ir.channels(), 1);
  CHECK_EQ(img_l_ir.type(), CV_8U);
  CHECK_EQ(img_r_ir.type(), CV_8U);

  XP::ir_census_stereo(rgb_calib_param,
                       ir_calib_param,
                       img_l_ir,
                       img_r_ir,
                       img_l_rgb,
                       FLAGS_depth_param_yaml,
                       &g_disparity_img);
  visualize_depth(ir_calib_param, save_img, depth_canvas);
}

bool kill_all_shared_queues() {
  imgs_for_saving_queue.kill();
  IR_imgs_for_saving_queue.kill();
  stereo_image_queue.kill();
  IR_image_queue.kill();
  if (FLAGS_ir_depth &&
      (XP_sensor_type == SensorType::XPIRL2 || XP_sensor_type == SensorType::XPIRL3)) {
    IR_depth_queue.kill();
  }
  return true;
}

bool process_ir_control(char keypressed) {
  if (g_xp_sensor_ptr == nullptr) {
    return false;
  }
  using XPDRIVER::XP_SENSOR::infrared_pwm_max;
  // -1 means no key is pressed
  if (g_infrared_mode != XPDRIVER::XP_SENSOR::OFF && keypressed != -1) {
    int index_old = g_infrared_index;
    switch (keypressed) {
      case '6':
        g_infrared_index = 1;
        break;
      case '7':
        g_infrared_index = infrared_pwm_max * 0.2;
        break;
      case '8':
        g_infrared_index = infrared_pwm_max * 0.6;
        break;
      case '9':
        g_infrared_index = infrared_pwm_max - 10;
        break;
      case '.':
        ++g_infrared_index;
        if (g_infrared_index >= infrared_pwm_max) g_infrared_index = infrared_pwm_max -1;
        break;
      case '>':
        g_infrared_index += 5;
        if (g_infrared_index >= infrared_pwm_max) g_infrared_index = infrared_pwm_max -1;
        break;
      case ',':
        --g_infrared_index;
        if (g_infrared_index < 0) g_infrared_index = 0;
        break;
      case '<':
        g_infrared_index -= 5;
        if (g_infrared_index < 0) g_infrared_index = 0;
        break;
      default:
        break;
    }
    if (index_old != g_infrared_index)
      g_xp_sensor_ptr->set_infrared_param(g_infrared_mode, g_infrared_index, FLAGS_ir_period);
  }
  // By pressing "i" or "I", we circle around the following IR mode:
  // OFF -> STRUCTURED -> INFRARED -> ALL_LIGHT -> OFF
  if ((keypressed == 'i' || keypressed == 'I')) {
    if (XP_sensor_type == SensorType::XPIRL2 || XP_sensor_type == SensorType::XPIRL3) {
      if (g_infrared_mode == XPDRIVER::XP_SENSOR::OFF) {
        g_infrared_mode = XPDRIVER::XP_SENSOR::STRUCTURED;
        cv::namedWindow("img_lr_IR");
        cv::moveWindow("img_lr_IR", 10, 10);
      } else if (g_infrared_mode == XPDRIVER::XP_SENSOR::STRUCTURED) {
        g_infrared_mode = XPDRIVER::XP_SENSOR::INFRARED;
      } else if (g_infrared_mode == XPDRIVER::XP_SENSOR::INFRARED) {
        g_infrared_mode = XPDRIVER::XP_SENSOR::ALL_LIGHT;
      } else if (g_infrared_mode == XPDRIVER::XP_SENSOR::ALL_LIGHT) {
        g_infrared_mode = XPDRIVER::XP_SENSOR::OFF;
      } else {
        g_infrared_mode = XPDRIVER::XP_SENSOR::OFF;
      }
      g_xp_sensor_ptr->set_infrared_param(g_infrared_mode, g_infrared_index, FLAGS_ir_period);
      if (g_infrared_mode == XPDRIVER::XP_SENSOR::OFF) {
        // close IR show window
        cv::destroyWindow("img_lr_IR");
      }
    } else {
      std::cout << "Only XPIRL2 support IR mode, Please check sensor type!"<< std::endl;
    }
  }
  return true;
}

bool process_gain_control(char keypressed) {
  if (g_xp_sensor_ptr == nullptr) {
    return false;
  }
  using XPDRIVER::XP_SENSOR::kAEC_steps;
  using XPDRIVER::XP_SENSOR::kAR0141_AEC_steps;
  const bool is_ir_sensor = (XP_sensor_type == SensorType::XPIRL2 ||\
                       XP_sensor_type == SensorType::XPIRL3);
  uint32_t AEC_steps = (is_ir_sensor ? kAR0141_AEC_steps : kAEC_steps);
  if (!g_auto_gain && keypressed != -1) {  // -1 means no key is pressed
    switch (keypressed) {
      case '1':
        // The lowest brightness possible
        g_aec_index = 0;
        g_xp_sensor_ptr->set_aec_index(g_aec_index);
        break;
      case '2':
        // 20% of max brightness
        g_aec_index = AEC_steps * 0.2;
        g_xp_sensor_ptr->set_aec_index(g_aec_index);
        break;
      case '3':
        // 60% of max brightness
        g_aec_index = AEC_steps * 0.6;
        g_xp_sensor_ptr->set_aec_index(g_aec_index);
        break;
      case '4':
        // max brightness
        g_aec_index = AEC_steps - 1;
        g_xp_sensor_ptr->set_aec_index(g_aec_index);
        break;
      case '+':
      case '=':
        ++g_aec_index;
        if (g_aec_index >= AEC_steps) g_aec_index = AEC_steps - 1;
        g_xp_sensor_ptr->set_aec_index(g_aec_index);
        break;
      case ']':
        g_aec_index += 5;
        if (g_aec_index >= AEC_steps) g_aec_index = AEC_steps - 1;
        g_xp_sensor_ptr->set_aec_index(g_aec_index);
        break;
      case '-':
        --g_aec_index;
        if (g_aec_index < 0) g_aec_index = 0;
        g_xp_sensor_ptr->set_aec_index(g_aec_index);
        break;
      case '[':
        g_aec_index -= 5;
        if (g_aec_index < 0) g_aec_index = 0;
        g_xp_sensor_ptr->set_aec_index(g_aec_index);
        break;
      default:
        break;
    }
  }
  if (keypressed == 'a' || keypressed == 'A') {
    g_auto_gain = !g_auto_gain;
    g_xp_sensor_ptr->set_auto_gain(g_auto_gain);
  }
  return true;
}

// Threads
void thread_proc_img() {
  VLOG(1) << "========= thread_proc_img thread starts";
  // mode compatibility is done in main
  cv::Mat img_lr_display;

  // cv::namedWindow has to be used in a single place
  img_lr_display.create(g_img_size.height, g_img_size.width * 2, CV_8UC3);

  const bool is_color = g_xp_sensor_ptr->is_color();
  XP::DuoCalibParam calib_param;
  if (!FLAGS_calib_yaml.empty()) {
    if (!calib_param.LoadCamCalibFromYaml(FLAGS_calib_yaml)) {
      LOG(ERROR) << FLAGS_calib_yaml << " cannot be loaded";
      run_flag = false;
      return;
    }
    if (g_img_size != calib_param.Camera.img_size) {
      LOG(ERROR) << "g_img_size = " << g_img_size
                 << " != calib info" << calib_param.Camera.img_size;
      run_flag = false;
      return;
    }
  }
  XP::DuoCalibParam ir_calib_param;
  if (FLAGS_ir_depth && !FLAGS_ir_calib_yaml.empty()) {
    if (!ir_calib_param.LoadCamCalibFromYaml(FLAGS_ir_calib_yaml)) {
      LOG(ERROR) << FLAGS_ir_calib_yaml << " cannot be loaded";
      run_flag = false;
      return;
    }
  }

  // These image Mat will be assigned properly according to the sensor type
  cv::Mat img_l_mono, img_r_mono, img_l_color, img_r_color, depth_canvas;
  if (FLAGS_depth || FLAGS_ir_depth) {
    depth_canvas.create(g_img_size.height, g_img_size.width, CV_8UC3);
  }

  size_t frame_counter = 0;
  std::chrono::time_point<steady_clock> pre_proc_time = steady_clock::now();
  float thread_proc_img_rate = 0.f;
  while (run_flag) {
    VLOG(1) << "========= thread_proc_img loop starts";
    // check if the imgs queue is too long
    bool pop_to_back = false;
    if (stereo_image_queue.size() > 10) {
      pop_to_back = true;
      if (!FLAGS_depth && !FLAGS_ir_depth) {
        // only show error if no additional computation is needed
        LOG(ERROR) << "stereo_image_queue too long (" << stereo_image_queue.size()
                   << "). Pop to back";
      }
    }
    StereoImage stereo_img;
    if (pop_to_back) {
      // record and calib_verify cannot be set at the same time
      if (!stereo_image_queue.wait_and_pop_to_back(&stereo_img)) {
        break;
      }
      VLOG(1) << "stereo_image_queue.wait_and_pop_front done";
    } else {
      if (!stereo_image_queue.wait_and_pop_front(&stereo_img)) {
        break;
      }
      VLOG(1) << "stereo_image_queue.wait_and_pop_front done";
    }

    // Compute the processing rate
    if (frame_counter % 10 == 0) {
      const int ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          steady_clock::now() - pre_proc_time).count();
      pre_proc_time = steady_clock::now();
      if (ms > 0) {
        thread_proc_img_rate = 10 * 1000  / ms;
      }
    }
    if (!FLAGS_spacebar_mode && !FLAGS_record_path.empty()) {
      // always true
      save_img = true;
    }
    // Get the mono/color image Mats properly
    // Sanity check first
    if (is_color) {
      CHECK_EQ(stereo_img.l.type(), CV_8UC3);  // sanity check
      img_l_color = stereo_img.l;
      img_r_color = stereo_img.r;
      cv::cvtColor(img_l_color, img_l_mono, cv::COLOR_BGR2GRAY);
      cv::cvtColor(img_r_color, img_r_mono, cv::COLOR_BGR2GRAY);
    } else {
      CHECK_EQ(stereo_img.l.type(), CV_8UC1);  // sanity check
      img_l_mono = stereo_img.l;
      img_r_mono = stereo_img.r;
      cv::cvtColor(img_l_mono, img_l_color, cv::COLOR_GRAY2BGR);
      cv::cvtColor(img_r_mono, img_r_color, cv::COLOR_GRAY2BGR);
    }

    const int rows = img_l_color.rows;
    const int cols = img_r_color.cols;
    for (int r = 0; r < rows; ++r) {
      uchar* display_lr_img_ptr = img_lr_display.ptr(r + 0);
      memmove(display_lr_img_ptr, img_l_color.ptr(r), cols * 3);
      memmove(display_lr_img_ptr + cols * 3, img_r_color.ptr(r), cols * 3);
    }

    cv::Mat img_l_ir, img_r_ir;
    if (FLAGS_ir_depth) {
      // [NOTE]:Only when camera puts IR images can we wait for the IR_depth_queue,
      // or sensor would not stop putting RGB images into stereo_image_queue which would
      // casue memory explosion.
      if (g_infrared_mode != XPDRIVER::XP_SENSOR::OFF) {
        StereoImage IR_latest_img;
        // Pop latest IR image for depth process
        if (!IR_depth_queue.wait_and_pop_to_back(&IR_latest_img)) {
          break;
        }
        img_l_ir = IR_latest_img.l;
        img_r_ir = IR_latest_img.r;
      }
    }
    if (FLAGS_depth) {
      process_stereo_depth(calib_param,
                           img_l_mono,
                           img_r_mono,
                           save_img,
                           &depth_canvas);
      image_thread_safe_copy(&g_depth_canvas, depth_canvas);
    }
    if (FLAGS_ir_depth) {
      // [NOTE]:There is no need to compute ir depth image if no new ir images received.
      if (g_infrared_mode != XPDRIVER::XP_SENSOR::OFF) {
        process_stereo_ir_depth(calib_param,
                                ir_calib_param,
                                img_l_ir,
                                img_r_ir,
                                img_l_mono,
                                save_img,
                                &depth_canvas);
        image_thread_safe_copy(&g_depth_canvas, depth_canvas);
      }
    }
    // show some debug info
    std::string debug_string;
    float img_rate = g_xp_sensor_ptr->get_image_rate();
    float imu_rate = g_xp_sensor_ptr->get_imu_rate();
    char buf[100];
    snprintf(buf, sizeof(buf),
             "img %4.1f Hz imu %5.1f Hz proc %4.1f Hz time %.2f sec",
             img_rate, imu_rate, thread_proc_img_rate, stereo_img.ts_100us * 1e-4);
    debug_string = std::string(buf);
    cv::putText(img_lr_display, debug_string,
                cv::Point(15, 15), cv::FONT_HERSHEY_COMPLEX, 0.5,
                cv::Scalar(255, 0, 255), 1);
    image_thread_safe_copy(&g_img_lr_display, img_lr_display);

    if (save_img) {
      uint64_t img_time_100us = static_cast<uint64_t>(stereo_img.ts_100us);
      std::ostringstream ss;
      ss << std::setfill('0') << std::setw(10) << img_time_100us;
      ImgForSave img_for_save;
      img_for_save.name = ss.str();
      img_for_save.l = stereo_img.l.clone();  // The channels are mono: 1, color: 3
      img_for_save.r = stereo_img.r.clone();  // The channels are mono: 1, color: 3
      if (FLAGS_depth || FLAGS_ir_depth) {
          img_for_save.xyz = g_depth_xyz_img.clone();
      }
      imgs_for_saving_queue.push_back(img_for_save);
      save_img = false;  // reset
    }
    ++frame_counter;
    VLOG(1) << "========= thread_proc_img loop ends";
  }
  VLOG(1) << "========= thread_proc_img stops";
}

// Threads
void thread_proc_ir_img() {
  VLOG(1) << "========= thread_proc_ir_img thread starts";
  // mode compatibility is done in main
  cv::Mat img_l_IR_display, img_r_IR_display, img_lr_IR_display;
  img_lr_IR_display.create(g_img_size.height / 2, g_img_size.width, CV_8UC1);
  img_l_IR_display = img_lr_IR_display(cv::Rect(0, 0, g_img_size.width / 2,
                                                  g_img_size.height / 2));
  img_r_IR_display = img_lr_IR_display(cv::Rect(g_img_size.width / 2, 0,
                                                  g_img_size.width / 2, g_img_size.height / 2));

  size_t frame_counter = 0;
  std::chrono::time_point<steady_clock> pre_proc_time = steady_clock::now();
  float thread_proc_img_rate = 0.f;
  while (run_flag) {
    VLOG(1) << "========= thread_proc_img loop starts";
    // check if the ir imgs queue is too long
    bool IR_pop_to_back = false;
    if (IR_image_queue.size() > 10) {
      IR_pop_to_back = true;
      LOG(ERROR) << "IR_image_queue too long (" << IR_image_queue.size()
                 << "). Pop to back";
    }
    StereoImage IR_img;
    if (IR_pop_to_back) {
      // record and calib_verify cannot be set at the same time
      if (!IR_image_queue.wait_and_pop_to_back(&IR_img)) {
        break;
      }
      VLOG(1) << "IR_image_queue.wait_and_pop_front done";
    } else {
      if (!IR_image_queue.wait_and_pop_front(&IR_img)) {
        break;
      }
      VLOG(1) << "IR_image_queue.wait_and_pop_front done";
    }
    // Compute the processing rate
    if (frame_counter % 10 == 0) {
      const int ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          steady_clock::now() - pre_proc_time).count();
      pre_proc_time = steady_clock::now();
      thread_proc_img_rate = 10 * 1000  / ms;
    }
    IR_img.l.copyTo(img_l_IR_display);
    IR_img.r.copyTo(img_r_IR_display);

    if (!FLAGS_spacebar_mode && !FLAGS_record_path.empty()) {
      // always true
      save_ir_img = true;
    }

    // show some debug info
    // TODO(zhourenyi): Implement get_ir_image_rate
    float ir_img_rate = g_xp_sensor_ptr->get_ir_image_rate();
    char buf[100];
    snprintf(buf, sizeof(buf),
             "img %4.1f Hz proc %4.1f Hz time %.2f sec",
             ir_img_rate, thread_proc_img_rate, IR_img.ts_100us * 1e-4);
    std::string debug_string = std::string(buf);
    cv::putText(img_lr_IR_display, debug_string,
                cv::Point(15, 15), cv::FONT_HERSHEY_COMPLEX, 0.5,
                cv::Scalar(255, 0, 255), 1);
    image_thread_safe_copy(&g_img_lr_IR_display, img_lr_IR_display);

    if (save_ir_img) {
      ImgForSave IR_img_for_save;
      uint64_t IR_img_time_100us = static_cast<uint64_t>(IR_img.ts_100us);
      std::ostringstream ss_IR;
      ss_IR << std::setfill('0') << std::setw(10) << IR_img_time_100us;
      IR_img_for_save.name = ss_IR.str();
      IR_img_for_save.l = IR_img.l.clone();
      IR_img_for_save.r = IR_img.r.clone();
      IR_imgs_for_saving_queue.push_back(IR_img_for_save);
      save_ir_img = false;  // reset
    }
    ++frame_counter;
    usleep(1000);  // sleep for 1ms
    VLOG(1) << "========= thread_proc_ir_img loop ends";
  }
  VLOG(1) << "========= thread_proc_ir_img stops";
}

void thread_save_img() {
  save_img = false;  // reset
  if (FLAGS_record_path.empty()) {
    return;
  }

  while (run_flag) {
    ImgForSave img_for_save;
    if (!imgs_for_saving_queue.wait_and_pop_to_back(&img_for_save)) {
      break;
    }
    cv::imwrite(FLAGS_record_path + "/l/" + img_for_save.name + ".png", img_for_save.l);
    cv::imwrite(FLAGS_record_path + "/r/" + img_for_save.name + ".png", img_for_save.r);
    if (img_for_save.xyz.rows > 0) {
      // save Z val
      cv::Mat_<uint16_t> z_img(img_for_save.xyz.size());
      for (int i = 0; i < img_for_save.xyz.rows; ++i) {
        for (int j = 0; j < img_for_save.xyz.cols; ++j) {
          if (img_for_save.xyz.at<cv::Vec3f>(i, j)[2] < 1e-5) {
            z_img(i, j) = 0;
          } else {
            // convert to mm
            z_img(i, j) = img_for_save.xyz.at<cv::Vec3f>(i, j)[2] * 1000;
          }
        }
      }
      cv::imwrite(FLAGS_record_path + "/Z/" + img_for_save.name + ".png", z_img);
    }
    VLOG(1) << "========= thread_save_img loop ends";
  }
  VLOG(1) << "========= thread_save_img thread stops";
}

void thread_save_ir_img() {
  VLOG(1) << "========= thread_save_ir_img thread starts";
  save_ir_img = false;  // reset
  if (FLAGS_record_path.empty()) {
    return;
  }
  while (run_flag) {
    ImgForSave img_for_save;
    if (!IR_imgs_for_saving_queue.wait_and_pop_to_back(&img_for_save)) {
      break;
    }
    cv::imwrite(FLAGS_record_path + "/l_IR/" + img_for_save.name + ".png", img_for_save.l);
    cv::imwrite(FLAGS_record_path + "/r_IR/" + img_for_save.name + ".png", img_for_save.r);
    VLOG(1) << "========= thread_save_ir_img loop ends";
  }
  VLOG(1) << "========= thread_save_ir_img thread stops";
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

#ifdef __ARM_NEON__
  if (FLAGS_cpu_core >= 0 && FLAGS_cpu_core < 8) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(FLAGS_cpu_core, &set);

    if (0 != sched_setaffinity(getpid(), sizeof(cpu_set_t), &set))
      exit(1);
    std::cout << "RUN ON CORE [" << FLAGS_cpu_core << "]" << std::endl;
  }
#endif  // __ARM_NEON__
  if (FLAGS_calib_yaml.empty()) {
    // some mode requires FLAGS_calib_yaml
    if (FLAGS_depth) {
      LOG(ERROR) << "You must set calib_yaml to enable undistort";
      return -1;
    }
  }
  int is_ir_depth_mode = 0;
  if (FLAGS_ir_depth && (FLAGS_ir_calib_yaml.empty() || FLAGS_calib_yaml.empty() ||
      FLAGS_depth_param_yaml.empty())) {
    LOG(ERROR) << "You must set ir_calib_yaml, calib_yaml and depth_param_yaml to enable ir_depth";
    return -1;
  } else if (FLAGS_ir_depth) {
    // Auto open IR infrared light in ir_depth.
    is_ir_depth_mode = 1;
  }

  g_auto_gain = FLAGS_auto_gain;
  run_flag = true;
  g_infrared_mode = XPDRIVER::XP_SENSOR::OFF;
  g_infrared_index = 200;
  g_xp_sensor_ptr.reset(new XpSensorMultithread(FLAGS_sensor_type,
                                                g_auto_gain,
                                                true,
                                                FLAGS_dev_name,
                                                FLAGS_wb_mode));
  // If input sensor_type is not supported, init will fail
  if (g_xp_sensor_ptr->init()) {
    VLOG(1) << "XpSensorMultithread init succeeeded!";
  } else {
    LOG(ERROR) << "XpSensorMultithread failed to init";
    return -1;
  }
  // check sensor_type after driver match
  g_xp_sensor_ptr->get_sensor_type(&XP_sensor_type);
  uint16_t width, height;
  // default param. Will be changed if calib yaml file is provided
  if (!(g_xp_sensor_ptr->get_sensor_resolution(&width, &height))) {
    LOG(ERROR) << "XpSensorMultithread failed to get sensor resolution";
    return -1;
  }
  std::string deviceID;
  if (!(g_xp_sensor_ptr->get_sensor_deviceid(&deviceID))) {
    LOG(ERROR) << "XpSensorMultithread failed to get sensor deviceID";
  }
  g_img_size.width = width;
  g_img_size.height = height;
  // FACE is a special XP3
  if (XP_sensor_type == SensorType::FACE) {
    g_img_size.height = width;
    g_img_size.width = height;
  }

  g_has_IR = (XP_sensor_type == SensorType::XPIRL2 || XP_sensor_type == SensorType::XPIRL3);
  if (!FLAGS_record_path.empty()) {
    // First make sure record_path exists
    if (!check_file_exist(FLAGS_record_path)) {
      create_directory(FLAGS_record_path);
      cout << "Created " << FLAGS_record_path << "\n";
    }
    // If record_path already exists, we will append time at the end of
    // the record path and hopefully it will have no collision, except the special
    // case of spacebar_mode, as we may intend to continue saving images in the same
    // record path.
    create_directory(FLAGS_record_path + "/l");
    create_directory(FLAGS_record_path + "/r");
    if (FLAGS_depth || FLAGS_ir_depth) {
      create_directory(FLAGS_record_path + "/Z");
    }
    if (g_has_IR) {
      create_directory(FLAGS_record_path + "/l_IR");
      create_directory(FLAGS_record_path + "/r_IR");
    }
  }

  // cv::namedWindow has to be used in a single place
  cv::namedWindow("img_lr");
  cv::moveWindow("img_lr", 1, 1);
  g_img_lr_display.image.create(g_img_size.height, g_img_size.width * 2, CV_8UC3);
  g_img_lr_display.image_name = "img_lr";
  if (g_has_IR) {
    g_img_lr_IR_display.image.create(g_img_size.height / 2, g_img_size.width, CV_8UC1);
    g_img_lr_IR_display.image_name = "img_lr_IR";
  } else {
    FLAGS_ir_depth = false;  // only IR sensor can output IR depth
  }
  if (FLAGS_depth && FLAGS_ir_depth) {
    LOG(ERROR) << "Only able to output depth from either RGB or IR images";
    return -1;
  } else if (FLAGS_depth) {
    cv::namedWindow("depth_canvas");
    cv::moveWindow("depth_canvas", 1, 1);
    g_depth_canvas.image.create(g_img_size.height, g_img_size.width, CV_8UC3);
    g_depth_canvas.image_name = "depth_canvas";
  } else if (FLAGS_ir_depth) {
    cv::namedWindow("depth_canvas");
    cv::moveWindow("depth_canvas", 1, 1);
    g_depth_canvas.image.create(g_img_size.height / 2, g_img_size.width / 2, CV_8UC3);
    g_depth_canvas.image_name = "depth_canvas";
  }

  // Prepare the thread pool to handle the data from XpSensorMultithread
  vector<std::thread> thread_pool;
  thread_pool.push_back(std::thread(thread_proc_img));
  if (g_has_IR) {
    thread_pool.push_back(std::thread(thread_proc_ir_img));
  }

  if (!FLAGS_record_path.empty()) {
    g_xp_sensor_ptr->set_imu_data_callback(nullptr);
    thread_pool.push_back(std::thread(thread_save_img));
    if (g_has_IR) {
      thread_pool.push_back(std::thread(thread_save_ir_img));
    }
  }

  // Register callback functions and let XpSensorMultithread spin
  CHECK(g_xp_sensor_ptr);
  g_xp_sensor_ptr->set_image_data_callback(image_data_callback);
  if (g_has_IR) {
    g_xp_sensor_ptr->set_IR_data_callback(IR_data_callback);
  }
  g_xp_sensor_ptr->run();

  size_t frame_counter = 0;
  if (is_ir_depth_mode) {
    if (!process_ir_control('I')) {
        LOG(ERROR) << "cannot process infrared control";
      }
  }

  while (run_flag) {
#ifdef __ARM_NEON__
    if (frame_counter % 2 == 0) // NOLINT
#endif
    {
      image_thread_safe_show(&g_img_lr_display);
      if (g_has_IR && g_infrared_mode != XPDRIVER::XP_SENSOR::OFF) {
        image_thread_safe_show(&g_img_lr_IR_display);
      }
      if (FLAGS_depth || FLAGS_ir_depth) {
        image_thread_safe_show(&g_depth_canvas);
      }
    }
    ++frame_counter;
    char keypressed = cv::waitKey(20);
    if (keypressed == 27) {
      // ESC
      run_flag = false;
      break;
    } else if (keypressed == 32 && !FLAGS_record_path.empty()) {
      // space
      save_img = true;
      save_ir_img = true;
    } else if (keypressed != -1) {
      if (!process_gain_control(keypressed)) {
        LOG(ERROR) << "cannot process gain control";
      }
      if (!process_ir_control(keypressed)) {
        LOG(ERROR) << "cannot process infrared control";
      }
    }
    // TODO(huyuexiang): if there is no usleep(1000),
    // it will cause images are used too slow very frequently in ir_depth mode.
    // waitKey will be used in the next loop
    usleep(1000);  // sleep for 1ms
  }

  kill_all_shared_queues();
  for (auto& t : thread_pool) {
    t.join();
  }
  if (!g_xp_sensor_ptr->stop()) {
    LOG(ERROR) << "XpSensorMultithread failed to stop properly!";
  }
  cout << "finished safely" << std::endl;

  // Release global cv::Mat before exist to avoid core dump
  g_img_lr_display.image.release();
  if (g_has_IR) {
    g_img_lr_IR_display.image.release();
  }
  if (FLAGS_depth) {
    g_depth_canvas.image.release();
  }
  if (FLAGS_depth || FLAGS_ir_depth) {
    g_depth_xyz_img.release();
    g_disparity_img.release();
    g_disparity_buf.release();
    for (int i = 0; i < g_disparity_ml.size(); ++i) {
      g_disparity_ml[i].release();
    }
  }
  return 0;
}
