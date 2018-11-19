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
// [NOTE] For now, XP sensor only works for Linux
#include <driver/helper/xp_logging.h>
#include <driver/XP_sensor.h>
#include <driver/xp_aec_table.h>
#include <driver/AR0141_aec_table.h>
#include <driver/v4l2.h>
#include <driver/firmware_config.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <limits.h>
#include <ctype.h>
#include <unistd.h>
#include <stdarg.h>
#ifdef __linux__  // predefined by gcc
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/usb/video.h>
#include <errno.h>
#include <iconv.h>
#include <linux/uvcvideo.h>
#include <fcntl.h>
#include <time.h>
#endif  // __linux__
#include <iostream>
#include <fstream>

// Uncomment for quick outdoor setting hack
// #define OUTDOOR_SETTING

/* extention unit control selector */
#define CY_FX_UVC_XU_SPI_FLASH                              (uint16_t)(0x0a00)
#define CY_FX_UVC_XU_CAM_REG                                (uint16_t)(0x0b00)
#define CY_FX_UVC_XU_REG_BURST                              (uint16_t)(0x0c00)
#define CY_FX_UVC_XU_SVER_RW                                (uint16_t)(0x0d00)
#define CY_FX_UVC_XU_REG_RW                                 (uint16_t)(0x0e00)
#define CY_FX_UVC_XU_HVER_RW                                (uint16_t)(0x0f00)
#define CY_FX_UVC_XU_FLAG_RW                                (uint16_t)(0x1000)
#define CY_FX_UVC_XU_IR_RW                                  (uint16_t)(0x1100)
#define CY_FX_UVC_XU_SPLAH_RW                               (uint16_t)(0x1200)
#define CY_FX_UVC_XU_DEBUG_RW                               (uint16_t)(0x1300)
#define CY_FX_UVC_XU_CALIB_RW                               (uint16_t)(0x1400)

// firmeware flag Bit
#define DEBUG_DBG_BIT           (0x01 << 0)
#define DEBUG_INFO_BIT          (0x01 << 1)
#define DEBUG_DUMP_BIT          (0x01 << 2)
#define IMU_FROM_IMAGE_BIT      (0x01 << 3)
// See MTV9V024-D.pdf Table 2: Frame Time for details
// row_time = HORZ_BLANK_CONTEXTA_REG + COL_WINDOW_SIZE_CONTEXTA_REG
// It seems V024 fixes the row_time to 846 (need to confirm)
// Assume SYSCLK is 27 MHz (need to confirm)
// V_Blank calculation based on target frame rate:
// 60 Hz: V_Blank = (27000000 * 1/60 - 4 - 846 * 480) / 846 = 51.91
// 50 Hz: V_Blank = (27000000 * 1/50 - 4 - 846 * 480) / 846 = 158.29
// 30 Hz: V_Blank = (27000000 * 1/30 - 4 - 846 * 480) / 846 = 583.82
// 25 Hz: V_Blank = (27000000 * 1/25 - 4 - 846 * 480) / 846 = 796.59
#ifdef XP_IMG_WVGA
// WVGA is still not working yet
#define XP_IMG_WIDTH 752
#define XP_START_COL 4
#else
#define XP_IMG_WIDTH  640
#define XP_START_COL ((752 - 640)/ 2)
#endif
#define XP_IMG_HEIGHT 480
#define FLASH_RW_LEN  255
#define CALIB_RW_LEN  255
#define PAYLOAD_LEN   (CALIB_RW_LEN - 7)

#define XP_H_BLANK 206
#define XP_ROW_TIME (XP_IMG_WIDTH + XP_H_BLANK)  // min val 704.  Fixed at 846
#define XP_IMG_FRAMERATE 25
#define XP_OSC_FREQ 27000000
#define XP_V_BLANK (XP_OSC_FREQ * 1 / XP_IMG_FRAMERATE - 4 \
                    - XP_ROW_TIME * XP_IMG_HEIGHT) / XP_ROW_TIME

#define XP_BOARD_MAX_GAIN 0x40  // 64
#define XP_BOARD_MIN_GAIN 0x10  // 16
#define XP_BOARD_MAX_EXP (XP_IMG_HEIGHT + XP_V_BLANK - 2)  // 1274 for VGA @ 25 Hz

// TODO(huyuexiang) take care of alignment if using other kind of member!
typedef struct {
  uint8_t header[2];                 // header[0]=0xAA, header[1]=0x55
  uint8_t packet_total;              // indicates how many packets
  uint8_t packet_len;                // length of packet in bytes, including check_sum
  uint8_t id;                        // start from 0 to 255
  uint8_t data[PAYLOAD_LEN];         // payload
  uint8_t check_sum[2];              // check sum from header[0] to data[PAYLOAD_LEN-1],
                                     // LSB first, MSB last
} calib_struct_t;
calib_struct_t device_data;
static const uint16_t wCRC16Table[256] = {
  0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
  0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
  0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
  0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
  0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
  0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
  0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
  0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
  0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
  0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
  0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
  0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
  0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
  0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
  0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
  0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
  0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
  0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
  0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
  0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
  0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};
namespace XPDRIVER {
namespace XP_SENSOR {

uint64_t get_timestamp_in_img(const uint8_t* data) {
  uint64_t clock_count_with_overflow;
  XP_CHECK_NOTNULL(data);
  clock_count_with_overflow = static_cast<uint64_t>(data[12]) << 24;
  clock_count_with_overflow |= static_cast<uint64_t>(data[13]) << 16;
  clock_count_with_overflow |= static_cast<uint64_t>(data[14]) << 8;
  // this produces wrong result if uint32_t is used
  clock_count_with_overflow |= static_cast<uint64_t>(data[15]) << 0;
  return clock_count_with_overflow;
}
bool stamp_timestamp_in_img(uint8_t* data, uint64_t time) {
  XP_CHECK_NOTNULL(data);
  data[12] = (time >> 24) & 0xff;
  data[13] = (time >> 16) & 0xff;
  data[14] = (time >> 8) & 0xff;
  data[15] = (time >> 0) & 0xff;
  return true;
}

#ifdef __linux__  // predefined by gcc
// [NOTE] We make sure the firmware version is readable and matches the minimum requirement.
bool get_XP_sensor_spec(int fd, XPSensorSpec* XP_sensor_spec_ptr) {
  // resolution
  get_v4l2_resolution(fd, &(XP_sensor_spec_ptr->ColNum), &(XP_sensor_spec_ptr->RowNum));

  // read sensor type (hardware version)
  XP_sensor_spec_ptr->sensor_type = read_hard_version(fd);

  // read soft version and check for minimum firmware version requirement
  bool ok = (read_soft_version(fd, XP_sensor_spec_ptr->Soft_ver_string) &&
             check_min_soft_version(XP_sensor_spec_ptr->Soft_ver_string,
                                    &(XP_sensor_spec_ptr->firmware_soft_version)));
  if (!ok) {
    printf("*** Please update firmware ***\n");
  }

  // read device ID
  read_deviceID(fd, XP_sensor_spec_ptr->dev_id);

  return ok;
}

static uint8_t value[20];
struct uvc_xu_control_query xu_query_imu_read = {
  .unit       = 3,  // has to be unit 3
  .selector   = CY_FX_UVC_XU_REG_BURST >> 8,
  .query      = UVC_GET_CUR,
  .size       = 17,
  .data       = value,
};

bool IMU_DataAccess(int fd, XP_20608_data* data_ptr) {
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query_imu_read) < 0) {
    return false;
  }
  int16_t raw_readings[6];
  for (int k = 0; k < 6; k++) {
    // pure bit operation
    raw_readings[k] = (value[k * 2] << 8) | value[k * 2 + 1];
  }
  // Convert IMU readings
  // Assume the scale is default +-2g for accel and 250 dps for gyro
  constexpr float accel_scale = XP_BOARD_ACCEL_SCALE;  // +-2g
  constexpr float gyro_scale = XP_BOARD_GYRO_SCALE;  // 1000 dps
  XP_20608_data& data = *data_ptr;
  for (int k = 0; k < 3; k++) {
    data.accel[k] = accel_scale * raw_readings[k + 3] / 32768.f;
  }
  data.temp = 99.f;
  for (int k = 0; k < 3; k++) {
    data.gyro[k] = gyro_scale * raw_readings[k] / 32768.f;
  }
  // ts
  data.clock_count = static_cast<uint64_t>(value[15]);
  data.clock_count |= static_cast<uint64_t>(value[14]) << 8;
  data.clock_count |= static_cast<uint64_t>(value[13]) << 16;
  data.clock_count |= static_cast<uint64_t>(value[12]) << 24;
  // sanity check
  if (value[16] != 0) {
    XP_LOG_ERROR("XP IMU data incomplete value[16] \'" << value[16] << "\'");
    return false;
  }
  return true;
}

void error_handle(const std::string ns = "") {
  int res = errno;
  const char *err;
  switch (res) {
    case ENOENT:    err = "Extension unit or control not found"; break;
    case ENOBUFS:   err = "Buffer size does not match control size"; break;
    case EINVAL:    err = "Invalid request code"; break;
    case EBADRQC:   err = "Request not supported by control"; break;
    default:        err = strerror(res); break;
  }
  XP_LOG_ERROR(ns << " ioctl failed code " << res << " err " << err);
  return;
}

bool read_register(int fd, int16_t regaddr, int16_t* regval) {
  uint8_t value[4];
  struct uvc_xu_control_query xu_query;

  // We need to *write* first to let cypress knows which register we want to read
  xu_query.unit = 3;  // has to be unit 3
  xu_query.selector = CY_FX_UVC_XU_CAM_REG >> 8;
  xu_query.query = UVC_SET_CUR;
  xu_query.size = 4;
  xu_query.data = value;
  value[0] = (regaddr >> 8) | 0x80;     /* higher address part*/
  value[1] = regaddr & 0xFF;   /* lower address part*/
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle("read_register:set");
    return false;
  }

  // delay 1ms for I2C transfers time
  usleep(1000);

  xu_query.query = UVC_GET_CUR;
  value[0] = regaddr >> 8;     /* higher address part*/
  value[1] = regaddr & 0xFF;   /* lower address part*/
  value[2] = 0;                /* higher byte */
  value[3] = 0;                /* lower byte */
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle("read_register:get");
    return false;
  }
  *regval = (value[2] << 8) + value[3];
  return true;
}

bool set_register(int fd, int16_t regaddr, int16_t regval) {
  uint8_t value[4];
  struct uvc_xu_control_query xu_query;
  xu_query.unit = 3;  // has to be unit 3
  xu_query.selector = CY_FX_UVC_XU_CAM_REG >> 8;
  xu_query.query = UVC_SET_CUR;
  xu_query.size = 4;
  xu_query.data = value;
  value[0] = regaddr >> 8;     /* higher address part*/
  value[1] = regaddr & 0xFF;   /* lower address part*/
  value[2] = regval >> 8;      /* higher byte */
  value[3] = regval & 0xFF;    /* lower byte */
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle("set_register:set");
    return false;
  }
  return true;
}
// debug control
bool xp_set_debug_variable(int fd, uint8_t* debug_ptr, uint32_t len) {
  uint8_t value[255] = {0};
  struct uvc_xu_control_query xu_query;
  memcpy(value, debug_ptr, len);
  xu_query.unit = 3;  // has to be unit 3
  xu_query.selector = CY_FX_UVC_XU_DEBUG_RW >> 8;
  xu_query.query = UVC_SET_CUR;
  xu_query.size = 255;
  xu_query.data = value;
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle("xp_set_firmware_debug:set");
    return false;
  }
  return true;
}
// control infrared light brightness function of firmware with TLC59116 chip.
bool xp_infrared_ctl(int fd, infrared_mode_t infrared_mode, uint8_t pwm_value, uint8_t period) {
  IR_ctl_t XPIRLx_IR_ctrl;
  uint8_t value[4] = {0};
  struct uvc_xu_control_query xu_query;
  if (infrared_mode == OFF) {
    XPIRLx_IR_ctrl.Set_infrared_mode = 0;
    XPIRLx_IR_ctrl.Set_structured_mode = 0;
    XPIRLx_IR_ctrl.RGB_IR_period = 0;
    printf("IR control: close Infrared and structured light\n");
  } else if (infrared_mode == INFRARED) {
    XPIRLx_IR_ctrl.Set_infrared_mode = 1;
    XPIRLx_IR_ctrl.Set_structured_mode = 0;
    XPIRLx_IR_ctrl.pwm_value = pwm_value;
    if (period > 0) {
      XPIRLx_IR_ctrl.RGB_IR_period = period;
    } else {
      printf("RGB_IR_period can't be zero and have reset to 1 when open infrared light\n");
      XPIRLx_IR_ctrl.RGB_IR_period = 1;
    }
    printf("IR control: Only open Infrared light, pwm_value = %d\n", pwm_value);
  } else if (infrared_mode == STRUCTURED) {
    XPIRLx_IR_ctrl.Set_infrared_mode = 0;
    XPIRLx_IR_ctrl.Set_structured_mode = 1;
    XPIRLx_IR_ctrl.pwm_value = pwm_value;
    if (period > 0) {
      XPIRLx_IR_ctrl.RGB_IR_period = period;
    } else {
      printf("RGB_IR_period can't be zero and have reset to 1 when open structured light\n");
      XPIRLx_IR_ctrl.RGB_IR_period = 1;
    }
    printf("IR control: Only open structured light, pwm_value = %d\n", pwm_value);
  } else if (infrared_mode == ALL_LIGHT) {
    XPIRLx_IR_ctrl.Set_infrared_mode = 1;
    XPIRLx_IR_ctrl.Set_structured_mode = 1;
    XPIRLx_IR_ctrl.pwm_value = pwm_value;
    if (period > 0) {
      XPIRLx_IR_ctrl.RGB_IR_period = period;
    } else {
      printf("RGB_IR_period can't be zero and have reset to 1 when open all light\n");
      XPIRLx_IR_ctrl.RGB_IR_period = 1;
    }
    printf("IR control: open Both infrared and structured light, pwm_value = %d\n", pwm_value);
  }
  XPIRLx_IR_ctrl.UpdateBit = 1;

  uint32_t* XPIRLx_IR_ctrl_p = reinterpret_cast<uint32_t *> (&XPIRLx_IR_ctrl);
  value[0] = *XPIRLx_IR_ctrl_p >> 24;
  value[1] = *XPIRLx_IR_ctrl_p >> 16;
  value[2] = *XPIRLx_IR_ctrl_p >> 8;
  value[3] = *XPIRLx_IR_ctrl_p >> 0;
  xu_query.unit = 3;  // has to be unit 3
  xu_query.selector = CY_FX_UVC_XU_IR_RW >> 8;
  xu_query.query = UVC_SET_CUR;
  xu_query.size = 4;
  xu_query.data = value;
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle("xp_infrared_ctl:set");
    return false;
  }
  return true;
}

// enable or disable imu embed img function of firmware, default is enable.
bool xp_imu_embed_img(int fd, bool enable) {
  struct firmware_ctl_t* firmware_ctrl_flag;
  uint8_t value[4] = {0};
  struct uvc_xu_control_query xu_query;

  xu_query.unit = 3;  // has to be unit 3
  xu_query.selector = CY_FX_UVC_XU_FLAG_RW >> 8;
  xu_query.query = UVC_GET_CUR;
  xu_query.size = 4;
  xu_query.data = value;
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle("xp_imu_embed_img:get");
    return false;
  }
  uint32_t ctl_tmp = (value[0] << 24 | value[1] << 16 | value[2] << 8 | value[3]);
  firmware_ctrl_flag = reinterpret_cast<struct firmware_ctl_t* >(&ctl_tmp);
  usleep(1000);
  xu_query.selector = CY_FX_UVC_XU_FLAG_RW >> 8;
  xu_query.query = UVC_SET_CUR;
  if (enable) {
    firmware_ctrl_flag->imu_from_image = 1;
  } else {
    firmware_ctrl_flag->imu_from_image = 0;
  }
  value[0] = ctl_tmp >> 24;
  value[1] = ctl_tmp >> 16;
  value[2] = ctl_tmp >> 8;
  value[3] = ctl_tmp >> 0;
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle("xp_imu_embed_img:set");
    return false;
  }
  return true;
}

/**
 *  @brief      read software vesion (string).
 *  @param[out] NULL.
 *  @return     NULL.
 */
bool read_soft_version(int fd, char* soft_ver_ptr) {
  uint8_t value[64] = {0};
  uint8_t length = 0;
  struct uvc_xu_control_query xu_query;
  // Firstly, get version length
  xu_query.unit = 3;  // has to be unit 3
  xu_query.selector = CY_FX_UVC_XU_SVER_RW >> 8;
  xu_query.query = UVC_GET_LEN;
  xu_query.size = 2;
  xu_query.data = value;
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle("read_soft_version_length:get");
    return false;
  }
  length = value[0];

  // Secondly, obtain version
  xu_query.unit = 3;  // has to be unit 3
  xu_query.selector = CY_FX_UVC_XU_SVER_RW >> 8;
  xu_query.query = UVC_GET_CUR;
  xu_query.size = length;
  xu_query.data = value;
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle("read_soft_version:get");
    return false;
  }
  memcpy(soft_ver_ptr, value, length);
  return true;
}

bool convert_soft_version(const char* soft_ver, struct XpSoftVersion* ver_unit) {
  int ret = std::sscanf(soft_ver, "V%d.%d.%d",
                        &(ver_unit->soft_ver_major),
                        &(ver_unit->soft_ver_minor),
                        &(ver_unit->soft_ver_patch));
  return (ret == 3);
}

bool check_min_soft_version(const char* soft_ver, XpSoftVersion* firmware_soft_ver) {
  // get current firmware soft version
  if (!convert_soft_version(soft_ver, firmware_soft_ver)) {
    XP_LOG_ERROR("Incorrect firmware version format!");
    return false;
  }
  XpSoftVersion& firmware_ver = *firmware_soft_ver;
  bool ver_ok = false;
  if (firmware_ver.soft_ver_major < MIN_FIRMWARE_VERSION_MAJOR) {
    ver_ok = false;
  } else if (firmware_ver.soft_ver_major > MIN_FIRMWARE_VERSION_MAJOR) {
    ver_ok = true;
  } else {
    // Check minor version
    if (firmware_ver.soft_ver_minor < MIN_FIRMWARE_VERSION_MINOR) {
      ver_ok = false;
    } else if (firmware_ver.soft_ver_minor > MIN_FIRMWARE_VERSION_MINOR) {
      ver_ok = true;
    } else {
      // Check patch version
      ver_ok = (firmware_ver.soft_ver_patch >= MIN_FIRMWARE_VERSION_PATCH);
    }
  }

  printf("Current firmware version: %d.%d.%d\n",
         firmware_ver.soft_ver_major,
         firmware_ver.soft_ver_minor,
         firmware_ver.soft_ver_patch);
  if (!ver_ok) {
    printf("Required firmware version: %d.%d.%d\n",
           MIN_FIRMWARE_VERSION_MAJOR,
           MIN_FIRMWARE_VERSION_MINOR,
           MIN_FIRMWARE_VERSION_PATCH);
  }
  return ver_ok;
}

/**
 *  @brief      read hardware version info.
 *  @param[out] NULL.
 *  @return     NULL.
 */
SensorType read_hard_version(int fd) {
  uint8_t value[64] = {0};
  struct uvc_xu_control_query xu_query;
  SensorType sensor_type;

  xu_query.unit = 3;  // has to be unit 3
  xu_query.selector = CY_FX_UVC_XU_HVER_RW >> 8;
  xu_query.query = UVC_GET_CUR;
  xu_query.size = 4;
  xu_query.data = value;

  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle("read_hard_version:get");
    return SensorType::Unkown_sensor;
  }
  int hardware_version_num = value[0] << 24 | value[1] << 16 | value[2] << 8 | value[3];
  switch (hardware_version_num) {
    case 0:
      // Maybe read 0 from some XP2 sensor, because without hardware version control design.
      sensor_type = SensorType::XP;
      break;
    case 1:
      // only read 1 from some XP2s.
      sensor_type = SensorType::XP2;
      break;
    case 2:
      sensor_type = SensorType::XP3;
      break;
    case 3:
      // driver don't support XP3s
      sensor_type = SensorType::Unkown_sensor;
      break;
    case 4:
      sensor_type = SensorType::XPIRL;
      break;
    case 5:
      sensor_type = SensorType::XPIRL2;
      break;
    case 6:
      sensor_type = SensorType::XPIRL3;
      break;
    case 7:
      sensor_type = SensorType::XPIRL3_A;
      break;
    default:
      sensor_type = SensorType::Unkown_sensor;
      break;
  }

  return sensor_type;
}

/**
 *  @brief      read device ID from sensor flash.
 *  @param[in]  NULL.
 *  @return     NULL.
 */
void read_deviceID(int fd, char* device_id) {
  uint8_t value[255] = {0};
  struct uvc_xu_control_query xu_query;
  char *p;

  xu_query.unit =  3;
  xu_query.selector = CY_FX_UVC_XU_SPLAH_RW >> 8;
  xu_query.query = UVC_GET_CUR;
  xu_query.size = FLASH_RW_LEN;
  xu_query.data = value;

  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle("read device_id");
  } else {
    // TODO(renyi): Check for unset device ID in firmware and replaced with "unknown" here
    memcpy(device_id, xu_query.data, 100);
    // The way to check device ID:
    // 1. If there is no '\0', the device ID is "unknown" obviously.
    // 2. If the characters before '\0' are all among 32 and 126, the device ID should be legal.
    p = strchr(device_id, '\0');
    if (p) {
      for (int i = 0; i < p - device_id; i++) {
        // illegal character, print "unknown"
        if (device_id[i] < 32 || device_id[i] > 126) {
          device_id[0] = '\0';
          printf("device ID: unknown\n");
          return;
        }
      }
      printf("device ID: %s\n", device_id);
    } else {
      // no '\0' means invalid ID, just print "unknown"
      device_id[0] = '\0';
      printf("device ID: unknown\n");
    }
  }
}

void make_crc16(const uint8_t* pDataIn, int iLenIn, uint16_t* pCRCOut) {
  uint16_t wResult = 0;
  uint16_t wTableNo = 0;
  int i = 0;
  for ( i = 0; i < iLenIn; i++ ) {
    wTableNo = ((wResult & 0xff) ^ (pDataIn[i] & 0xff));
    wResult = ((wResult >> 8) & 0xff) ^ wCRC16Table[wTableNo];
  }
  *pCRCOut = wResult;
}
bool verify_crc16(const uint8_t* pDataIn, int iLenIn) {
  uint8_t crc_buf[2] = {0};
  uint16_t check_sum = 0;
  make_crc16(pDataIn, iLenIn - 2, &check_sum);
  crc_buf[0] = check_sum & 0xFF;
  crc_buf[1] = (check_sum >> 8) & 0xFF;
  if (pDataIn[iLenIn - 2] == crc_buf[0]) {
    if (pDataIn[iLenIn - 1] == crc_buf[1]) {
      return true;
    }
  }
  return false;
}
/**
 *  @brief      read calibration file from sensor flash.
 *  @param[in]  dest_buffer: useful data from sensor.
 *  @param[out] size: useful data length.
 *  @return     true means success, false means fail.
 */
bool read_calib_from_device(int fd, uint8_t *dest_buffer, int *size) {
  uint8_t packet_num = 1;
  uint8_t packet_id = 0;
  int read_len = 0;
  struct uvc_xu_control_query xu_query;

  xu_query.unit =  3;
  xu_query.selector = CY_FX_UVC_XU_CALIB_RW >> 8;
  xu_query.query = UVC_GET_CUR;
  xu_query.size = CALIB_RW_LEN;
  xu_query.data = reinterpret_cast<uint8_t *>(&device_data);

  memset(reinterpret_cast<uint8_t *>(&device_data), 0, sizeof(calib_struct_t));

  do {
    if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
      error_handle();
      *size = 0;
      return false;
    }
    if ((device_data.header[0] != 0xAA) || (device_data.header[1] != 0x55)) {
      printf("wrong packet header from device: [0x%x] [0x%x]\n", \
            device_data.header[0], device_data.header[1]);
      *size = 0;
      return false;
    }
    if (verify_crc16(reinterpret_cast<uint8_t *>(&device_data),
                      device_data.packet_len) == false) {
      uint16_t check_sum;
      printf("%dth packet CRC check failed!\n", device_data.id);
      *size = 0;
      return false;
    }
    packet_num = device_data.packet_total;
    packet_id = device_data.id;
    memcpy(&dest_buffer[packet_id * PAYLOAD_LEN], device_data.data, PAYLOAD_LEN);
    read_len += PAYLOAD_LEN;
    usleep(300000);
  } while (packet_id < packet_num - 1);
  printf("read %d bytes from device\n", read_len);

  dest_buffer[read_len] = '\0';
  *size = read_len;
  return true;
}

bool read_calib_str_from_device(int fd, std::string *calib_str) {
  int size;
  bool ret = false;
  char *buffer = new char[XP_SECTOR_MAX_LEN];
  if (read_calib_from_device(fd, reinterpret_cast<uint8_t *>(buffer), &size)) {
    *calib_str = buffer;
    ret = true;
  }
  delete [] buffer;
  return ret;
}
/**
 *  @brief      write calibration file to sensor flash.
 *  @param[in]  buffer: data to write.
 *  @param[out] size: data length.
 *  @return     NULL.
 */
void write_calib_to_device(int fd, uint8_t *buffer, int size) {
  uint8_t i, j;
  uint8_t packet_num;
  uint8_t packet_id;
  uint16_t check_sum;
  struct uvc_xu_control_query xu_query;

  // TODO(huyuexiang): the size MUST be smaller than sector size 64KB, here defined 60KB.
  assert(size <= XP_CALIB_MAX_LEN);

  packet_num = size /PAYLOAD_LEN + (size % PAYLOAD_LEN ? 1 : 0);
  packet_id = 0;
  device_data.header[0] = 0xAA;
  device_data.header[1] = 0x55;
  device_data.packet_len = CALIB_RW_LEN;

  xu_query.unit = 3;
  xu_query.selector = CY_FX_UVC_XU_CALIB_RW >> 8;
  xu_query.query = UVC_SET_CUR;
  xu_query.size = CALIB_RW_LEN;
  xu_query.data = reinterpret_cast<uint8_t *>(&device_data);

  for (i = 0; i < packet_num; i++) {
    packet_id = i;
    device_data.packet_total = packet_num;
    device_data.id = packet_id;

    // the last packet, we should take care that the length maybe less than PAYLOAD_LEN.
    // in that case, device_data.data[] must be init to '\0' first
    if (i == packet_num -1) {
      memset(device_data.data, '\0', PAYLOAD_LEN);
      memcpy(device_data.data, &buffer[i * PAYLOAD_LEN], size - i * PAYLOAD_LEN);
    } else {
      memcpy(device_data.data, &buffer[i * PAYLOAD_LEN], PAYLOAD_LEN);
    }
    make_crc16(reinterpret_cast<uint8_t *>(&device_data),
               device_data.packet_len - 2,
               &check_sum);
    device_data.check_sum[0] = check_sum & 0xFF;
    device_data.check_sum[1] = (check_sum >> 8) & 0xFF;
    if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
      error_handle();
    }
    printf("write %dth packet to device\n", device_data.id);
    if (packet_id == 0) {
      // The first packet write should wait for a long time because of spiflash erase.
      sleep(3);
    } else {
      usleep(100000);
    }
  }
}

void write_calib_str_to_device(int fd, const std::string &calib_str) {
  int size = calib_str.length();
  char *buffer = new char[size];
  calib_str.copy(buffer, size, 0);
  write_calib_to_device(fd, reinterpret_cast<uint8_t *>(buffer), size);
  delete [] buffer;
}

void read_calib_from_file(const std::string &filename, uint8_t *buffer) {
  std::cout << filename << std::endl;
  std::ifstream in(filename);
  int length;
  if (!in.is_open()) {
    XP_LOG_ERROR("Could not load calib file");
    return;
  }
  in.seekg(0, std::ios::end);
  length = in.tellg();
  // TODO(huyuexiang): the size MUST be smaller than 1/2 sector size 64KB, here defined 30KB.
  assert(length <= XP_CALIB_MAX_LEN / 2);
  in.seekg(0, std::ios::beg);
  in.read(reinterpret_cast<char *>(buffer), length);
  in.close();
}

bool set_registers_to_default(int v4l2_dev, SensorType sensor_type, int aec_index,
                              bool verbose, uint32_t* exp_ptr, uint32_t* gain_ptr) {
  if (sensor_type == SensorType::XPIRL2 ||
      sensor_type == SensorType::XPIRL3 ||
      sensor_type == SensorType::XPIRL3_A) {
    set_register(v4l2_dev, 0x3060, kAR0141_AEC_LUT[aec_index][0]);  // gain
    set_register(v4l2_dev, 0x3012, kAR0141_AEC_LUT[aec_index][1]);  // exposure
  } else {
    set_register(v4l2_dev, 0x0B, kAEC_LUT[aec_index][1]);  // COARSE_SHUTTER_WIDTH_TOTAL_CONTEXTA
    set_register(v4l2_dev, 0x35, kAEC_LUT[aec_index][0]);  // GLOBAL_GAIN_CONTEXTA_REG
  }
  // TODO(zhoury): Figure out why we cannot use set_aec_index here.
  // If not set registers in the order above, sometimes the image can be very dark
  if (gain_ptr != nullptr) {
    // convert from [16 - 64] to [0 - 100]
    int16_t gain_reg_val = kAEC_LUT[aec_index][0];
    uint32_t gain_percentage = 100 * (gain_reg_val - XP_BOARD_MIN_GAIN) /
        (XP_BOARD_MAX_GAIN - XP_BOARD_MIN_GAIN);
    *gain_ptr = gain_percentage;
  }
  if (exp_ptr != nullptr) {
    // convert from [0 - XP_BOARD_MAX_EXP] to [0 - 100]
    int16_t exp_reg_val = kAEC_LUT[aec_index][1];
    uint32_t exp_percentage = 100 * exp_reg_val / XP_BOARD_MAX_EXP;
    *exp_ptr = exp_percentage;
  }

  if (verbose) {
    int16_t regval;
    if (read_register(v4l2_dev, 0x06, &regval)) {
      printf(" current v_blank = %d\n", regval);
    }
    if (read_register(v4l2_dev, 0x35, &regval)) {  // GLOBAL_GAIN_CONTEXTA_REG
      printf(" current gain regval = %d\n", regval);
    }
    if (read_register(v4l2_dev, 0x0B, &regval)) {  // COARSE_SHUTTER_WIDTH_TOTAL_CONTEXTA
      printf(" current exp regval = %d\n", regval);
    }

    // V4L2 operation reads from the UVC descriptions, which is only set once by the firmware
    // and will NOT reflect later changes.
    /*
    struct v4l2_format format;
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int r = ioctl(v4l2_dev, VIDIOC_G_FMT, &format);
    if (r < 0) {
      XP_LOG_ERROR("ioctl(fd, VIDIOC_G_FMT, &format) failed. code " << r
                 << " dev " << v4l2_dev);
    } else {
      XP_LOG_ERROR(" current width x height = "
                 << format.fmt.pix.width << " x " << format.fmt.pix.height);
    }
    */
  }
  return true;
}

bool set_aec_index(int fd, uint32_t aec_index, SensorType sensor_type, bool verbose) {
  XP_CHECK_GE(aec_index, 0);
  int16_t gain_reg_val;
  int16_t exp_reg_val;
  if (sensor_type == SensorType::XPIRL2 ||
      sensor_type == SensorType::XPIRL3 ||
      sensor_type == SensorType::XPIRL3_A) {
    XP_CHECK_LT(aec_index, kAR0141_AEC_steps);
    gain_reg_val = kAR0141_AEC_LUT[aec_index][0];
    exp_reg_val =  kAR0141_AEC_LUT[aec_index][1];
    set_register(fd, 0x3060, gain_reg_val);  // gain
    set_register(fd, 0x3012, exp_reg_val);   // exposure
  } else {
    XP_CHECK_LT(aec_index, kAEC_steps);
    gain_reg_val = kAEC_LUT[aec_index][0];
    exp_reg_val =  kAEC_LUT[aec_index][1];
    set_register(fd, 0x0B, exp_reg_val);   // COARSE_SHUTTER_WIDTH_TOTAL_CONTEXTA
    set_register(fd, 0x35, gain_reg_val);  // GLOBAL_GAIN_CONTEXTA_REG
  }
  if (verbose) {
    printf("aec index %d, reg val gain = %d  exp = %d\n", aec_index, gain_reg_val, exp_reg_val);
  }
  return false;
}

bool set_exp_percentage(int fd, int16_t val, bool verbose) {
  if (val > 100 || val < 0) {
    XP_LOG_ERROR("exp val = " << val);
    return false;
  }
  // set HDR
  // not working yet
  // set_register(fd, 0x08, val * XP_BOARD_MAX_EXP / 100);
  // set_register(fd, 0x09, val * XP_BOARD_MAX_EXP / 100);
#ifdef OUTDOOR_SETTING
  int16_t regval = val * XP_BOARD_MAX_EXP / 512;
#else
  int16_t regval = val * XP_BOARD_MAX_EXP / 100;
#endif
  set_register(fd, 0x0B, regval);
  if (verbose) {
    printf("exp  reg value = 0x%04x = %d (%d%%)\n", regval, regval, val);
  }
  return true;
}

bool set_gain_percentage(int fd, int16_t val, bool verbose) {
  if (val > 100 || val < 0) {
    XP_LOG_ERROR("gain val = " << val);
    return false;
  }
  // The gain register value has to be in range 16 to 64 (1x to 4x)
  // We linearly map percentage value of 0-100 to register value 16-64
  // See V024 spec sheet for register 0x35 Analog Gain Control
  // http://www.onsemi.com/pub/Collateral/MT9V024-D.PDF
  int16_t regval = (XP_BOARD_MAX_GAIN - XP_BOARD_MIN_GAIN) * val / 100 + XP_BOARD_MIN_GAIN;

  // gain in context A
  set_register(fd, 0x35, regval);

  if (verbose) {
    float true_gain = static_cast<float>(regval) / XP_BOARD_MIN_GAIN;
    printf("gain reg value = 0x%04x (%d%%) -> %6.4fx gain\n", regval, val, true_gain);
  }
  return true;
}
#endif  // __linux__

#ifdef  __CYGWIN__
SharedMemoryReader::SharedMemoryReader(const std::string& file) {
  HANDLE hMapFile = OpenFileMapping(FILE_MAP_ALL_ACCESS,   // read/write access
                    FALSE,                 // do not inherit the name
                    file.c_str());         // name of mapping object
  if (hMapFile == NULL) {
    XP_LOG_ERROR("Could not open file mapping object" << file
             << " error " <<  GetLastError());
    return;
  }
  Buf_ptr_ = (LPTSTR)MapViewOfFile(hMapFile,   // handle to map object
                   FILE_MAP_ALL_ACCESS,  // read/write permission
                   0, 0, 0);
  if (Buf_ptr_ == NULL) {
    XP_LOG_ERROR("Could not map view of file error" << GetLastError());
  }
}
SharedMemoryReader::~SharedMemoryReader() {}
bool SharedMemoryReader::read(SharedMemoryReader::ImageData* data) {
  CHECK_NOTNULL(data);
  constexpr int data_len = XP_IMG_WIDTH * XP_IMG_HEIGHT * 2;
  if (data->data->size() != data_len) {
    data->data->resize(data_len);
  }
  char io_flag = 1;
  while (io_flag != 0) {
  CopyMemory(&io_flag, (PVOID)Buf_ptr_, 1);
  }
  io_flag = 2;  // reading
  CopyMemory((PVOID)Buf_ptr_, &io_flag, 1);
  CopyMemory(&(data->data->at(0)), (PVOID)(Buf_ptr_ + 1), data_len);
  io_flag = 0;  // idle
  CopyMemory((PVOID)Buf_ptr_, &io_flag, 1);
  return true;
}
#endif
ImuReader::ImuReader() :
    idx_(0),
    imu_rate_(0),
    imu_sample_count_(0),
    imu_sample_count_for_rate_(-1),
    first_imu_clock_count_(0) {
  buf_ = new uint8_t[100];
  counter32To64_ptr_.reset(
      new Counter32To64(XP_CLOCK_32BIT_MAX_COUNT));
}
ImuReader::~ImuReader() {
  delete [] buf_;
}
int ImuReader::imu_rate() const {
  return imu_rate_;
}
int ImuReader::imu_sample_count() const {
  return imu_sample_count_;
}
// clock_count_ unit: ms
uint64_t ImuReader::first_imu_clock_count() const {
  return first_imu_clock_count_;
}
#ifdef __linux__  // predefined by gcc
bool ImuReader::read(int fd, XP_20608_data* imu_data_ptr) {
  uint8_t received;
  int rdlen = ::read(fd, &received, 1);
  if (rdlen == 0) {
    XP_LOG_ERROR("rdlen == 0");
    return false;
  }
  XP_CHECK_EQ(rdlen, 1);
  // end of line
  if (received != 0xEF) {
    buf_[idx_] = received;
    // printf("%X\n", buf_[idx_] + '0');
    idx_++;
    if (idx_ > 50) {
      XP_LOG_ERROR("idx_ too long " << idx_);
      idx_ = 0;
    }
    return false;
  }
  // do the work
  // printf("Correct end character received! and length is %d\n", idx_);
  if (idx_ != 32) {
     // purge this line since something is wrong and clean the buffer
     XP_LOG_ERROR("Error from line: more or less character in the line! idx_ = " << idx_);
     idx_ = 0;
     return false;
  }
  XP_CHECK_NOTNULL(imu_data_ptr);
  XP_20608_data& imu_data = *imu_data_ptr;
  this->get_vec3f_from_sensor_data(buf_, imu_data.gyro);
  this->get_vec3f_from_sensor_data(buf_ + 12, imu_data.accel);
  for (int i = 0; i < 3; ++i) {
    imu_data.gyro[i] *= gyro_scale_;
    imu_data.accel[i] *= accel_scale_;
  }
  uint64_t time_unsign =   static_cast<uint64_t>(buf_[24] << 4 | buf_[25]) << 24
                         | static_cast<uint64_t>(buf_[26] << 4 | buf_[27]) << 16
                         | static_cast<uint64_t>(buf_[28] << 4 | buf_[29]) << 8
                         | static_cast<uint64_t>(buf_[30] << 4 | buf_[31]);
  idx_ = 0;
  // increment counter
  ++imu_sample_count_;
  if (imu_sample_count_ < 10) {
    // skip the first few imu samples
    // garbage data
    return false;
  }
  uint64_t clock_count_wo_overflow = counter32To64_ptr_->convertNewCount32(time_unsign);
  if (first_imu_clock_count_ == 0) {
    first_imu_clock_count_ = clock_count_wo_overflow;
  }
  uint64_t time_100us = clock_count_wo_overflow * 10;
  imu_data.clock_count = time_100us;
  if (imu_sample_count_for_rate_ < 0) {
    // init
    imu_sample_start_tp_ = std::chrono::steady_clock::now();
  }
  ++imu_sample_count_for_rate_;
  if (imu_sample_count_for_rate_ > 20) {
    const int time_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - imu_sample_start_tp_).count();
    imu_rate_ = 1000000 / (time_us / imu_sample_count_for_rate_);
    imu_sample_start_tp_ = std::chrono::steady_clock::now();
    imu_sample_count_for_rate_ = 0;
  }
  return true;
}
#endif  // __linux__
bool ImuReader::get_imu_from_img(const uint8_t* data,
                                 XP_20608_data* imu_data_ptr,
                                 const bool use_100us) {
  XP_CHECK_NOTNULL(imu_data_ptr);
  XP_20608_data& imu_data = *imu_data_ptr;
  this->get_vec3f_from_img_data(data, imu_data.gyro);
  this->get_vec3f_from_img_data(data + 6, imu_data.accel);
  for (int i = 0; i < 3; ++i) {
    imu_data.gyro[i] *= gyro_scale_;
    imu_data.accel[i] *= accel_scale_;
  }
  const uint64_t clock_count_with_overflow = get_timestamp_in_img(data);
  // increment counter
  ++imu_sample_count_;
  if (imu_sample_count_ < 10) {
    XP_LOG_INFO("imu_sample_count_ = " << imu_sample_count_);
    // skip the first few imu samples
    // garbage data
    return false;
  }
  if (first_imu_clock_count_ == 0) {
    first_imu_clock_count_ = clock_count_with_overflow;
  }
  if (use_100us) {
    // convert to 100us
    uint64_t time_100us = clock_count_with_overflow * 10;
    imu_data.clock_count = time_100us;
  } else {
    // The clock count is stored in the unit of ms
    imu_data.clock_count = clock_count_with_overflow;
  }
  if (imu_sample_count_for_rate_ < 0) {
    // init
    imu_sample_start_tp_ = std::chrono::steady_clock::now();
  }
  ++imu_sample_count_for_rate_;
  if (imu_sample_count_for_rate_ > 20) {
    const int time_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - imu_sample_start_tp_).count();
    if (time_us > 0) {
      imu_rate_ = (1000000 * imu_sample_count_for_rate_) / time_us;
    }
    imu_sample_start_tp_ = std::chrono::steady_clock::now();
    imu_sample_count_for_rate_ = 0;
  }
  return true;
}
bool ImuReader::get_vec3f_from_sensor_data(const uint8_t* data, float* v) {
  for (int xyz = 0; xyz < 3; ++xyz) {
    uint16_t unsigned_v = (data[0 + xyz * 4] << 4 | data[1 + xyz * 4]) << 8
                           | (data[2 + xyz * 4] << 4 | data[3 + xyz * 4]);
    int16_t signed_v = static_cast<int16_t>(unsigned_v);
    if (unsigned_v & 0x8000) {
       unsigned_v = 65536 - unsigned_v;
       signed_v = static_cast<int16_t>(unsigned_v) * (-1);
    }
    v[xyz] = static_cast<float>(signed_v) / 32768.f;
  }
  return true;
}
bool ImuReader::get_vec3f_from_img_data(const uint8_t* data, float* v) {
  for (int xyz = 0; xyz < 3; ++xyz) {
    uint16_t unsigned_v = (data[0 + xyz * 2] << 8) | (data[1 + xyz * 2]);
    int16_t signed_v = static_cast<int16_t>(unsigned_v);
    if (unsigned_v & 0x8000) {
       unsigned_v = 65536 - unsigned_v;
       signed_v = static_cast<int16_t>(unsigned_v) * (-1);
    }
    v[xyz] = static_cast<float>(signed_v) / 32768.f;
  }
  return true;
}
}  // namespace XP_SENSOR
}  // namespace XPDRIVER
