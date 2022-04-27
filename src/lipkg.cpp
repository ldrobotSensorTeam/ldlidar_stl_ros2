/**
 * @file lipkg.cpp
 * @author LDRobot (marketing1@ldrobot.com)
 * @brief  LiDAR data protocol processing App
 *         This code is only applicable to LDROBOT LiDAR LD06 products 
 * sold by Shenzhen LDROBOT Co., LTD  
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lipkg.h"

#include <math.h>
#include <string.h>

#include <algorithm>

#include "tofbf.h"

static const uint8_t CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8};

uint8_t CalCRC8(const uint8_t *data, uint16_t data_len) {
  uint8_t crc = 0;
  while (data_len--) {
    crc = CrcTable[(crc ^ *data) & 0xff];
    data++;
  }
  return crc;
}

LiPkg::LiPkg(rclcpp::Node::SharedPtr& node,std::string frame_id, bool laser_scan_dir, bool enable_angle_crop_func,
  double angle_crop_min, double angle_crop_max)
    : node_(node),
      frame_id_(frame_id),
      timestamp_(0),
      speed_(0),
      error_times_(0),
      is_frame_ready_(false),
      is_pkg_ready_(false),
      laser_scan_dir_(laser_scan_dir),
      enable_angle_crop_func_(enable_angle_crop_func),
      angle_crop_min_(angle_crop_min),
      angle_crop_max_(angle_crop_max) {

}

double LiPkg::GetSpeed(void) { 
  return speed_ / 360.0; // unit is Hz 
}

bool LiPkg::AnalysisOne(uint8_t byte) {
  static enum {
    HEADER,
    VER_LEN,
    DATA,
  } state = HEADER;
  static uint16_t count = 0;
  static uint8_t tmp[128] = {0};
  static uint16_t pkg_count = sizeof(LiDARFrameTypeDef);

  switch (state) {
    case HEADER:
      if (byte == PKG_HEADER) {
        tmp[count++] = byte;
        state = VER_LEN;
      }
      break;
    case VER_LEN:
      if (byte == PKG_VER_LEN) {
        tmp[count++] = byte;
        state = DATA;
      } else {
        state = HEADER;
        count = 0;
        return false;
      }
      break;
    case DATA:
      tmp[count++] = byte;
      if (count >= pkg_count) {
        memcpy((uint8_t *)&pkg, tmp, pkg_count);
        uint8_t crc = CalCRC8((uint8_t *)&pkg, pkg_count - 1);
        state = HEADER;
        count = 0;
        if (crc == pkg.crc8) {
          return true;
        } else {
          error_times_++;
          return false;
        }
      }
      break;
    default:
      break;
  }

  return false;  
}

bool LiPkg::Parse(const uint8_t *data, long len) {
  for (int i = 0; i < len; i++) {
    if (AnalysisOne(data[i])) {
      // parse a package is success
      double diff = (pkg.end_angle / 100 - pkg.start_angle / 100 + 360) % 360;
      if (diff > (double)pkg.speed * POINT_PER_PACK / kPointFrequence * 3 / 2) {
        error_times_++;
      } else {
        speed_ = pkg.speed; // Degrees per second
        timestamp_ = pkg.timestamp; // In milliseconds
        uint32_t diff = ((uint32_t)pkg.end_angle + 36000 - (uint32_t)pkg.start_angle) % 36000;
        float step = diff / (POINT_PER_PACK - 1) / 100.0;
        float start = (double)pkg.start_angle / 100.0;
        float end = (double)(pkg.end_angle % 36000) / 100.0;
        PointData data;
        for (int i = 0; i < POINT_PER_PACK; i++) {
          data.distance = pkg.point[i].distance;
          data.angle = start + i * step;
          if (data.angle >= 360.0) {
            data.angle -= 360.0;
          }
          data.intensity = pkg.point[i].intensity;
          one_pkg_[i] = data;
          frame_tmp_.push_back(PointData(data.angle, data.distance, data.intensity));
        }
        // prevent angle invert
        one_pkg_.back().angle = end;
        is_pkg_ready_ = true;
      }
    }
  }

  return true;
}

bool LiPkg::AssemblePacket() {
  float last_angle = 0;
  Points2D tmp, data;
  int count = 0;

  for (auto n : frame_tmp_) {
    // wait for enough data, need enough data to show a circle
    // enough data has been obtained
    if ((n.angle < 20.0) && (last_angle > 340.0)) {
      // std::cout << "count: " << count << std::endl;
      if ((count * GetSpeed()) > (kPointFrequence * 1.4)) {
        frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.begin() + count - 1);
        // RCLCPP_INFO_STREAM(node_->get_logger()," [ldrobot] frame error,count num: " << count);
        return false;
      }
      data.insert(data.begin(), frame_tmp_.begin(), frame_tmp_.begin() + count);
      // std::cout << "data.size() " << data.size() << std::endl;
      Tofbf tofbfLd06(speed_);
      tmp = tofbfLd06.NearFilter(data);

      std::sort(tmp.begin(), tmp.end(), [](PointData a, PointData b) { return a.angle < b.angle; });
      if (tmp.size() > 0) {
        ToLaserscan(tmp);
        is_frame_ready_ = true;
        frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.begin() + count);
        return true;
      }
    }
    count++;
    last_angle = n.angle;
  }

  return false;
}

const std::array<PointData, POINT_PER_PACK>& LiPkg::GetPkgData(void) {
  is_pkg_ready_ = false;
  return one_pkg_;
}

void LiPkg::ToLaserscan(std::vector<PointData> src) {
  float angle_min, angle_max, range_min, range_max, angle_increment, scan_time_interval;

  // Adjust the parameters according to the demand
  angle_min = ANGLE_TO_RADIAN(src.front().angle);
  angle_max = ANGLE_TO_RADIAN(src.back().angle);
  range_min = 0.02;
  range_max = 12;
  angle_increment = ANGLE_TO_RADIAN(speed_ / kPointFrequence);

  static uint16_t last_times_stamp = 0;
  uint16_t tmp_times_stamp = GetTimestamp();
  if (tmp_times_stamp - last_times_stamp < 0) {
    scan_time_interval = (tmp_times_stamp - last_times_stamp + 30000) / 1000.f; // timers uint is Seconds
  } else {
    scan_time_interval = (tmp_times_stamp - last_times_stamp) / 1000.f;
  }
  last_times_stamp = tmp_times_stamp;

  // Calculate the number of scanning points
  if (speed_ > 0) {
    unsigned int beam_size = static_cast<unsigned int>(ceil((angle_max - angle_min) / angle_increment));
    output_.header.stamp = node_->now();
    output_.header.frame_id = frame_id_;
    output_.angle_min = angle_min;
    output_.angle_max = angle_max;
    output_.range_min = range_min;
    output_.range_max = range_max;
    output_.angle_increment = angle_increment;
    output_.scan_time = scan_time_interval;
    if (beam_size <= 1) {
      output_.time_increment = 0.0;
    } else {
      output_.time_increment = scan_time_interval / static_cast<float>(beam_size - 1);
    }
    // First fill all the data with Nan
    output_.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output_.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

    unsigned int last_index = 0;
    for (auto point : src) {
      float range = point.distance / 1000.f;  // distance unit transform to meters
      float intensity = point.intensity;      // laser receive intensity
      float dir_angle;
      
      if (laser_scan_dir_) {
        dir_angle = static_cast<float>(360.f - point.angle); // lidar rotation data flow changed from clockwise to counterclockwise
      } else {
        dir_angle = point.angle;
      }
      
      if (enable_angle_crop_func_) { // Angle crop setting, Mask data within the set angle range
        if ((dir_angle >= angle_crop_min_) && (dir_angle <= angle_crop_max_)) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      float angle = ANGLE_TO_RADIAN(dir_angle); // lidar angle unit form degree transform to radian
      unsigned int index = static_cast<unsigned int>((angle - output_.angle_min) / output_.angle_increment);
      if (index < beam_size) {
        // If the current content is Nan, it is assigned directly
        if (std::isnan(output_.ranges[index])) {
          output_.ranges[index] = range;
          unsigned int err = index - last_index;
          if (err == 2){
            output_.ranges[index - 1] = range;
            output_.intensities[index - 1] = intensity;
          }
        } else { // Otherwise, only when the distance is less than the current
                //   value, it can be re assigned
          if (range < output_.ranges[index]) {
            output_.ranges[index] = range;
          }
        }
        output_.intensities[index] = intensity;
        last_index = index;
      }
    }
  }
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/