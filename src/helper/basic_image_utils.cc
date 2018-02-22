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
#include <driver/helper/basic_image_utils.h>
#include <driver/xp_aec_table.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#ifndef __DEVELOPMENT_DEBUG_MODE__
#define __IMAGE_UTILS_NO_DEBUG__
#endif

namespace XPDRIVER {

// only use central area in the image
constexpr int kMarginRow = 50;
constexpr int kMarginCol = 100;
constexpr int kPixelStep = 2;

// Compute the histogram of a sampled area of the input image and return the number of
// sampled pixels
int sampleBrightnessHistogram(const cv::Mat& raw_img,
                              std::vector<int>* histogram,
                              int* avg_pixel_val_ptr) {
  const int end_row = raw_img.rows - kMarginRow;
  const int end_col = raw_img.cols - kMarginCol;

  // Given the current algorithm, collecting histogram is not
  // necessary. But we still do so in case later we switch to a better
  // algorithm
  int pixel_num = 0;
  int avg_pixel_val = 0;
  histogram->clear();
  histogram->resize(256, 0);
  int over_exposure_pixel_num = 0;
  for (int i = kMarginRow; i < end_row; i += kPixelStep) {
    for (int j = kMarginCol; j < end_col; j += kPixelStep) {
      const uint8_t pixel_val = raw_img.data[i * raw_img.cols + j];
      avg_pixel_val += pixel_val;
      (*histogram)[pixel_val]++;
      ++pixel_num;
    }
  }
  if (avg_pixel_val_ptr) {
    *avg_pixel_val_ptr = avg_pixel_val / pixel_num;
  }
  return pixel_num;
}

void gridBrightDarkAdjustBrightness(const cv::Mat& raw_img,
                                    int* adjusted_pixel_val_ptr) {
  // Bright / dark region settings
  constexpr int kBrightRegionThres = 240;
  constexpr int kDarkRegionThres = 25;
  constexpr float kBrightRegionWeight = 1.2f;
  constexpr float kDarkRegionWeight = 0.75f;

  // Grid settings
  constexpr int kGridSize = 10;
  constexpr int kPixelsPerGrid = kGridSize * kGridSize / kPixelStep / kPixelStep;

  const int grid_rows = (raw_img.rows - 2 * kMarginRow) / kGridSize;
  const int grid_cols = (raw_img.cols - 2 * kMarginCol) / kGridSize;
  int adjusted_pixel_val = 0;
  for (int grid_r = 0; grid_r < grid_rows; ++grid_r) {
    for (int grid_c = 0; grid_c < grid_cols; ++grid_c) {
      int start_row = grid_r * kGridSize + kMarginRow;
      int end_row = start_row + kGridSize;
      int start_col = grid_c * kGridSize + kMarginCol;
      int end_col = start_col + kGridSize;
      int grid_pixel_val = 0;

      for (int i = start_row; i < end_row; i += kPixelStep) {
        for (int j = start_col; j < end_col; j += kPixelStep) {
          grid_pixel_val += raw_img.data[i * raw_img.cols + j];
        }
      }
      grid_pixel_val /= kPixelsPerGrid;
      if (grid_pixel_val > kBrightRegionThres) {
        int tmp = grid_pixel_val * kBrightRegionWeight;
        grid_pixel_val *= kBrightRegionWeight;
        XP_CHECK_EQ(tmp, grid_pixel_val);
      } else if (grid_pixel_val < kDarkRegionThres) {
        int tmp = grid_pixel_val * kDarkRegionWeight;
        grid_pixel_val *= kDarkRegionWeight;
        XP_CHECK_EQ(tmp, grid_pixel_val);
      }
      adjusted_pixel_val += grid_pixel_val;
    }
  }
  *adjusted_pixel_val_ptr = adjusted_pixel_val / (grid_rows * grid_cols);
}

// return true if new aec_index is found
bool computeNewAecTableIndex(const cv::Mat& raw_img,
                             const bool smooth_aec,
                             int* aec_index_ptr) {
  using XPDRIVER::XP_SENSOR::kAEC_steps;
  using XPDRIVER::XP_SENSOR::kAEC_LUT;
  XP_CHECK_NOTNULL(aec_index_ptr);
  int& aec_index = *aec_index_ptr;
  XP_CHECK_LT(aec_index, kAEC_steps);
  XP_CHECK_GE(aec_index, 0);

  cv::Mat mono_img;
  if (raw_img.channels() == 1) {
    mono_img = raw_img;
  } else if (raw_img.channels() == 3) {
    cv::cvtColor(raw_img, mono_img, cv::COLOR_BGR2GRAY);
  } else {
    XP_LOG_INFO("Unsupported raw_img channels: " << raw_img.channels());
    return false;
  }

  std::vector<int> histogram;
  int avg_pixel_val = 0;
  int pixel_num = sampleBrightnessHistogram(mono_img, &histogram, &avg_pixel_val);
  if (pixel_num == 0) {
    // Nothing is sampled.  Something is wrong with raw_image
    return false;
  }

  int adjusted_pixel_val = 0;
  gridBrightDarkAdjustBrightness(mono_img, &adjusted_pixel_val);

#ifndef __IMAGE_UTILS_NO_DEBUG__
  int acc_pixel_counts = 0;
  int median_pixel_val = 0;
  for (int i = 0; i < 256; ++i) {
    acc_pixel_counts += histogram[i];
    if (acc_pixel_counts >= pixel_num / 2) {
      median_pixel_val = i;
      break;
    }
  }

  int saturate_pixel_counts = 0;
  for (int i = 253; i < 256; ++i) {
    saturate_pixel_counts += histogram[i];
  }
  float saturate_ratio = static_cast<float>(saturate_pixel_counts) / pixel_num;
  XP_VLOG(1, " pixel_val avg = " << avg_pixel_val
          << " adj_avg = " << adjusted_pixel_val
          << " median = " << median_pixel_val
          << " sat_ratio = " << saturate_ratio);
#endif

  // Heuristically adjust AEC table index
  // [NOTE] a step in AEC table is in average ~4% brightness change.  We calculate a rough
  // step number that will drag the adjusted avg_pixel_val close to 128.
  // We simply use add/minus instead multiply/divide here
  // [NOTE] Due to mono aec table, the brightness changes in the first few rows of
  // are very abrupt, e.g., index 0 -> index 1, ratio = 100%
  constexpr float kStepRatioNormal = 0.05f;
  constexpr float kStepRatioBright = 0.10f;
  constexpr float kStepRatioVeryBright = 0.20f;
  constexpr int kMaxStepNumNormal = 5;
  constexpr int kMaxStepNumBright = 2;
  constexpr int kMaxStepNumVeryBright = 1;
  float step_ratio;
  int max_step_num;
  if (aec_index < 16) {
    max_step_num = kMaxStepNumVeryBright;
    step_ratio = kStepRatioVeryBright;
  } else if (aec_index < 32) {
    max_step_num = kMaxStepNumBright;
    step_ratio = kStepRatioBright;
  } else {
    max_step_num = kMaxStepNumNormal;
    step_ratio = kStepRatioNormal;
  }

  // [NOTE] We need to hand tune the target brightness to avoid saturation
  // e.g., 128 can be too high
  constexpr float target_brightness = 100.f;
  float brightness_ratio = adjusted_pixel_val / target_brightness;
  int rough_step_num = (brightness_ratio - 1.f) / step_ratio;

  // If smooth_aec is true, clip the step number to avoid sudden jump in brightness.
  // Otherwise, try to jump directly to the target aec index.
  int actual_step_num;
  if (!smooth_aec) {
    actual_step_num = rough_step_num;
  } else  if (rough_step_num > max_step_num) {
    actual_step_num = max_step_num;
  } else if (rough_step_num < -max_step_num) {
    actual_step_num = -max_step_num;
  } else {
    actual_step_num = rough_step_num;
  }

#ifndef __IMAGE_UTILS_NO_DEBUG__
  XP_VLOG(1, " brightness_ratio = " << brightness_ratio
          << " rough_step_num = " << rough_step_num
          << " actual_step_num = " << actual_step_num);
#endif

  // Compute the new aec_index
  constexpr int kLowestAecIndex = 1;
  aec_index -= actual_step_num;
  if (aec_index < kLowestAecIndex) {
    aec_index = kLowestAecIndex;
  } else if (aec_index > kAEC_steps - 1) {
    aec_index = kAEC_steps - 1;
  }
  return true;
}

#ifdef __ARM_NEON__
void AutoWhiteBalance::compute_RGB_mean_neon(const cv::Mat& rgb_img_,
                                             uint32_t* ptr_r_mean,
                                             uint32_t* ptr_g_mean,
                                             uint32_t* ptr_b_mean) const {
  int width = rgb_img_.cols;
  int height = rgb_img_.rows;
  uint32x4_t r_mean = vmovq_n_u32(0);
  uint32x4_t g_mean = vmovq_n_u32(0);
  uint32x4_t b_mean = vmovq_n_u32(0);
  uint16x8_t vr, vg, vb;
  for (int r = 0; r < height; r += 8) {
    for (int c = 0; c < width; c += 16) {
      uint8x16x3_t raw_data = vld3q_u8(rgb_img_.ptr(r) + 3 * c);
      __builtin_prefetch(rgb_img_.ptr(r) + 3 * (c + 16), 0, 1);
      vr = vaddl_u8(vget_low_u8(raw_data.val[0]), vget_high_u8(raw_data.val[0]));
      vg = vaddl_u8(vget_low_u8(raw_data.val[1]), vget_high_u8(raw_data.val[1]));
      vb = vaddl_u8(vget_low_u8(raw_data.val[2]), vget_high_u8(raw_data.val[2]));
      r_mean = vaddq_u32(r_mean, vaddl_u16(vget_low_u16(vr), vget_high_u16(vr)));
      g_mean = vaddq_u32(g_mean, vaddl_u16(vget_low_u16(vg), vget_high_u16(vg)));
      b_mean = vaddq_u32(b_mean, vaddl_u16(vget_low_u16(vb), vget_high_u16(vb)));
    }
  }
  *ptr_r_mean = vgetq_lane_u32(r_mean, 0) + vgetq_lane_u32(r_mean, 1) +
                vgetq_lane_u32(r_mean, 2) + vgetq_lane_u32(r_mean, 3);
  *ptr_g_mean = vgetq_lane_u32(g_mean, 0) + vgetq_lane_u32(g_mean, 1) +
                vgetq_lane_u32(g_mean, 2) + vgetq_lane_u32(g_mean, 3);
  *ptr_b_mean = vgetq_lane_u32(b_mean, 0) + vgetq_lane_u32(b_mean, 1) +
                vgetq_lane_u32(b_mean, 2) + vgetq_lane_u32(b_mean, 3);
}
#endif  // __ARM_NEON__

inline void AutoWhiteBalance::compute_RGB_mean(const cv::Mat &rgb_img_,
                                               uint32_t *ptr_r_mean,
                                               uint32_t *ptr_g_mean,
                                               uint32_t *ptr_b_mean) const {
#ifdef __ARM_NEON__
    return compute_RGB_mean_neon(rgb_img_, ptr_r_mean, ptr_g_mean, ptr_b_mean);
#else
  int width = rgb_img_.cols;
  int height = rgb_img_.rows;
  uint32_t mr = 0, mg = 0, mb = 0;
  for (int r = 0; r < height; r += 8) {
    const uint8_t* r_img_data_ptr = rgb_img_.ptr(r);
    const uint8_t* g_img_data_ptr = r_img_data_ptr + 1;
    const uint8_t* b_img_data_ptr = r_img_data_ptr + 2;
    for (int c = 0; c < width; ++c) {
      mr += *r_img_data_ptr;
      mg += *g_img_data_ptr;
      mb += *b_img_data_ptr;
      r_img_data_ptr += 3;
      g_img_data_ptr += 3;
      b_img_data_ptr += 3;
    }
  }
  *ptr_r_mean = mr;
  *ptr_g_mean = mg;
  *ptr_b_mean = mb;
#endif  // __ARM_NEON__
}

void AutoWhiteBalance::compute_AWB_coefficients(const cv::Mat& rgb_img_) {
  uint32_t r_mean = 0, g_mean = 0, b_mean = 0;
  compute_RGB_mean(rgb_img_, &r_mean, &g_mean, &b_mean);

  if (g_mean > r_mean && g_mean > b_mean) {
    XP_VLOG(1, "Green channel based.");
    m_coeff_g_ = 1.f;
    m_coeff_r_ = g_mean / static_cast<float>(r_mean);
    m_coeff_b_ = g_mean / static_cast<float>(b_mean);
  } else if (r_mean > g_mean && r_mean > b_mean) {
    XP_VLOG(1, "Red channel based.");
    m_coeff_g_ = r_mean / static_cast<float>(g_mean);
    m_coeff_r_ = 1.f;
    m_coeff_b_ = r_mean / static_cast<float>(b_mean);
  } else {
    XP_VLOG(1, "Blue channel based.");
    m_coeff_g_ = b_mean / static_cast<float>(g_mean);
    m_coeff_r_ = b_mean / static_cast<float>(r_mean);
    m_coeff_b_ = 1.f;
  }
}

#ifdef __ARM_NEON__
void AutoWhiteBalance::correct_white_balance_coefficients_neon(cv::Mat* rgb_img_ptr) {
  uint8x16x3_t src_raw_data, dst_data;
  int width = rgb_img_ptr->cols;
  int height = rgb_img_ptr->rows;
  float v_coeffs[3];  // r, g, b
  v_coeffs[0] = m_coeff_r_;
  v_coeffs[1] = m_coeff_g_;
  v_coeffs[2] = m_coeff_b_;
  for (int r = 0; r < height; ++r) {
    uint8_t* rgb_img_row_ptr = rgb_img_ptr->ptr(r);
    for (int c = 0; c < width; c += 16) {
      src_raw_data = vld3q_u8(rgb_img_row_ptr + c * 3);
      for (int ch = 0; ch < 3; ++ch) {
        uint16x8_t low8 = vmovl_u8(vget_low_u8(src_raw_data.val[ch]));
        uint16x8_t high8 = vmovl_u8(vget_high_u8(src_raw_data.val[ch]));
        uint32x4_t low80 = vmovl_u16(vget_low_u16(low8));
        uint32x4_t low81 = vmovl_u16(vget_high_u16(low8));
        uint32x4_t high80 = vmovl_u16(vget_low_u16(high8));
        uint32x4_t high81 = vmovl_u16(vget_high_u16(high8));
        uint32x4_t rlow80 = vcvtq_u32_f32(vmulq_n_f32(vcvtq_f32_u32(low80), v_coeffs[ch]));
        uint32x4_t rlow81 = vcvtq_u32_f32(vmulq_n_f32(vcvtq_f32_u32(low81), v_coeffs[ch]));
        uint32x4_t rhigh80 = vcvtq_u32_f32(vmulq_n_f32(vcvtq_f32_u32(high80), v_coeffs[ch]));
        uint32x4_t rhigh81 = vcvtq_u32_f32(vmulq_n_f32(vcvtq_f32_u32(high81), v_coeffs[ch]));
        dst_data.val[ch] = vcombine_u8(
                    vqmovn_u16(vcombine_u16(vqmovn_u32(rlow80), vqmovn_u32(rlow81))),
                    vqmovn_u16(vcombine_u16(vqmovn_u32(rhigh80), vqmovn_u32(rhigh81))));
      }
      vst3q_u8(rgb_img_row_ptr + c * 3, dst_data);
    }
  }
}
#endif  // __ARM_NEON__

void AutoWhiteBalance::correct_white_balance_coefficients(cv::Mat* rgb_img_ptr) {
#ifdef __ARM_NEON__
  return correct_white_balance_coefficients_neon(rgb_img_ptr);
#else
  cv::Mat& rgb_img_ = *rgb_img_ptr;
  int width = rgb_img_.cols;
  int height = rgb_img_.rows;
  int cr, cg, cb;

  for (int r = 0; r < height; ++r) {
    uint8_t* rgb_img_row_ptr = rgb_img_.ptr(r);
    for (int c = 0; c < width; ++c) {
      cr = *(rgb_img_row_ptr + 0) * m_coeff_r_;
      cg = *(rgb_img_row_ptr + 1) * m_coeff_g_;
      cb = *(rgb_img_row_ptr + 2) * m_coeff_b_;
      *(rgb_img_row_ptr + 0) = (cr <= 255 ? cr : 255);
      *(rgb_img_row_ptr + 1) = (cg <= 255 ? cg : 255);
      *(rgb_img_row_ptr + 2) = (cb <= 255 ? cb : 255);
      rgb_img_row_ptr += 3;
    }
  }
#endif  // __ARM_NEON__
}
}  // namespace XPDRIVER
