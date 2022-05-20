/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
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

#include "ros2_api.h"
#include "lipkg.h"

void  ToLaserscanMessagePublish(ldlidar::Points2D& src, ldlidar::LiPkg* commpkg, LaserScanSetting& setting,
 rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub);

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("ldlidar_published"); // create a ROS2 Node

  std::string product_name;
	std::string topic_name;
	std::string port_name;
  LaserScanSetting setting;
	setting.frame_id = "base_laser";
  setting.laser_scan_dir = true;
  setting.enable_angle_crop_func = false;
  setting.angle_crop_min = 0.0;
  setting.angle_crop_max = 0.0;
  
  // declare ros2 param
  node->declare_parameter<std::string>("product_name", product_name);
  node->declare_parameter<std::string>("topic_name", topic_name);
  node->declare_parameter<std::string>("port_name", port_name);
  node->declare_parameter<std::string>("frame_id", setting.frame_id);
  node->declare_parameter<bool>("laser_scan_dir", setting.laser_scan_dir);
  node->declare_parameter<bool>("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->declare_parameter<double>("angle_crop_min", setting.angle_crop_min);
  node->declare_parameter<double>("angle_crop_max", setting.angle_crop_max);

  // get ros2 param
  node->get_parameter("product_name", product_name);
  node->get_parameter("topic_name", topic_name);
  node->get_parameter("port_name", port_name);
  node->get_parameter("frame_id", setting.frame_id);
  node->get_parameter("laser_scan_dir", setting.laser_scan_dir);
  node->get_parameter("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->get_parameter("angle_crop_min", setting.angle_crop_min);
  node->get_parameter("angle_crop_max", setting.angle_crop_max);

  ldlidar::LiPkg *lidar_pkg = new ldlidar::LiPkg();
  ldlidar::CmdInterfaceLinux *cmd_port = new ldlidar::CmdInterfaceLinux();

  RCLCPP_INFO_STREAM(node->get_logger(), " [ldrobot] SDK Pack Version is " << lidar_pkg->GetSdkPackVersionNum());
  RCLCPP_INFO(node->get_logger(), " [ldrobot] <product_name>: %s ,<topic_name>: %s ,<port_name>: %s ,<frame_id>: %s", 
              product_name.c_str(), topic_name.c_str(), port_name.c_str(), setting.frame_id.c_str());

  RCLCPP_INFO(node->get_logger(), "[ldrobot] <laser_scan_dir>: %s,<enable_angle_crop_func>: %s,<angle_crop_min>: %f,<angle_crop_max>: %f",
   (setting.laser_scan_dir?"Counterclockwise":"Clockwise"), (setting.enable_angle_crop_func?"true":"false"), setting.angle_crop_min, setting.angle_crop_max);

  if (port_name.empty()) {
    RCLCPP_ERROR(node->get_logger(), " [ldrobot] input <port_name> param is null");
    exit(EXIT_FAILURE);
  }

  cmd_port->SetReadCallback(std::bind(&ldlidar::LiPkg::CommReadCallback, lidar_pkg, std::placeholders::_1, std::placeholders::_2));

  if (cmd_port->Open(port_name)) {
    RCLCPP_INFO(node->get_logger(), " [ldrobot] open %s device %s success!", product_name.c_str(), port_name.c_str());
  }else {
    RCLCPP_ERROR(node->get_logger(), " [ldrobot] open %s device %s fail!", product_name.c_str(), port_name.c_str());
    exit(EXIT_FAILURE);
  }

  // create ldlidar data topic and publisher
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);
  
  rclcpp::WallRate r(10); //10hz

  auto last_time = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    if (lidar_pkg->IsFrameReady()) {
      lidar_pkg->ResetFrameReady();
      last_time = std::chrono::steady_clock::now();
      ldlidar::Points2D laserscandata = lidar_pkg->GetLaserScanData();
      ToLaserscanMessagePublish(laserscandata, lidar_pkg, setting, node, publisher);
    }

    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-last_time).count() > 1000) { 
			RCLCPP_ERROR(node->get_logger(),"[ldrobot] lidar pub data is time out, please check lidar device");
			exit(EXIT_FAILURE);
		}

    r.sleep();
  }

  cmd_port->Close();

  delete lidar_pkg;
  lidar_pkg = nullptr;
  delete cmd_port;
  cmd_port = nullptr;
  
  RCLCPP_INFO(node->get_logger(), "[ldrobot] this node of ldlidar_published is end");
  rclcpp::shutdown();

  return 0;
}

void  ToLaserscanMessagePublish(ldlidar::Points2D& src, ldlidar::LiPkg* commpkg, LaserScanSetting& setting,
 rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub) {
  float angle_min, angle_max, range_min, range_max, angle_increment;
  double scan_time;
  rclcpp::Time start_scan_time;
  static rclcpp::Time end_scan_time;

  start_scan_time = node->now();
  scan_time = (start_scan_time.seconds() - end_scan_time.seconds());

  // Adjust the parameters according to the demand
  angle_min = ANGLE_TO_RADIAN(src.front().angle);
  angle_max = ANGLE_TO_RADIAN(src.back().angle);
  range_min = 0.02;
  range_max = 12;
  float spin_speed = static_cast<float>(commpkg->GetSpeedOrigin());
  float scan_freq = static_cast<float>(commpkg->kPointFrequence);
  angle_increment = ANGLE_TO_RADIAN(spin_speed / scan_freq);
  // Calculate the number of scanning points
  if (commpkg->GetSpeedOrigin() > 0) {
    int beam_size = static_cast<int>(ceil((angle_max - angle_min) / angle_increment));
    if (beam_size < 0) {
      RCLCPP_ERROR(node->get_logger(), "[ldrobot] error beam_size < 0");
    }
    sensor_msgs::msg::LaserScan output;
    output.header.stamp = start_scan_time;
    output.header.frame_id = setting.frame_id;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;
    if (beam_size <= 1) {
      output.time_increment = 0;
    } else {
      output.time_increment = static_cast<float>(scan_time / (double)(beam_size - 1));
    }
    output.scan_time = scan_time;
    // First fill all the data with Nan
    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

    for (auto point : src) {
      float range = point.distance / 1000.f;  // distance unit transform to meters
      float intensity = point.intensity;      // laser receive intensity 
      float dir_angle = point.angle;

      if ((point.distance == 0) && (point.intensity == 0)) { // filter is handled to  0, Nan will be assigned variable.
        range = std::numeric_limits<float>::quiet_NaN(); 
        intensity = std::numeric_limits<float>::quiet_NaN();
      }

      if (setting.enable_angle_crop_func) { // Angle crop setting, Mask data within the set angle range
        if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max)) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      float angle = ANGLE_TO_RADIAN(dir_angle); // Lidar angle unit form degree transform to radian
      int index = static_cast<int>((angle - output.angle_min) / output.angle_increment);
      if (index < beam_size) {
        if (index < 0) {
          RCLCPP_ERROR(node->get_logger(), "[ldrobot] error index: %d, beam_size: %d, angle: %f, output.angle_min: %f, output.angle_increment: %f", 
                     index, beam_size, angle, output.angle_min, output.angle_increment);
        }

        if (setting.laser_scan_dir) {
          int index_anticlockwise = beam_size - index - 1;
          // If the current content is Nan, it is assigned directly
          if (std::isnan(output.ranges[index_anticlockwise])) {
            output.ranges[index_anticlockwise] = range;
          } else { // Otherwise, only when the distance is less than the current
                    //   value, it can be re assigned
            if (range < output.ranges[index_anticlockwise]) {
                output.ranges[index_anticlockwise] = range;
            }
          }
          output.intensities[index_anticlockwise] = intensity;
        } else {
          // If the current content is Nan, it is assigned directly
          if (std::isnan(output.ranges[index])) {
            output.ranges[index] = range;
          } else { // Otherwise, only when the distance is less than the current
                  //   value, it can be re assigned
            if (range < output.ranges[index]) {
              output.ranges[index] = range;
            }
          }
          output.intensities[index] = intensity;
        }
      }
    }
    lidarpub->publish(output);
    end_scan_time = start_scan_time;
    RCLCPP_INFO(node->get_logger(), "[ldrobot] pub lidar data");
  } 
}


/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
