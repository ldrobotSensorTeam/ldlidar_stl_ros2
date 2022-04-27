/**
 * @file main.cpp
 * @author LDRobot (marketing1@ldrobot.com)
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

#include <rclcpp/rclcpp.hpp>
#include <stdio.h>

#include <iostream>
#include <string>

#include "cmd_interface_linux.h"
#include "lipkg.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // create a ROS2 Node
  auto node = std::make_shared<rclcpp::Node>("ldlidar_published"); 

  std::string product_name;
	std::string topic_name;
	std::string port_name;
	std::string frame_id;
  bool laser_scan_dir = true;
  bool enable_angle_crop_func = false;
  double angle_crop_min = 0.0;
  double angle_crop_max = 0.0;
  
  // declare ros2 param
  node->declare_parameter<std::string>("product_name", product_name);
  node->declare_parameter<std::string>("topic_name", topic_name);
  node->declare_parameter<std::string>("port_name", port_name);
  node->declare_parameter<std::string>("frame_id", frame_id);
  node->declare_parameter<bool>("laser_scan_dir", laser_scan_dir);
  node->declare_parameter<bool>("enable_angle_crop_func", enable_angle_crop_func);
  node->declare_parameter<double>("angle_crop_min", angle_crop_min);
  node->declare_parameter<double>("angle_crop_max", angle_crop_max);

  // get ros2 param
  node->get_parameter("product_name", product_name);
  node->get_parameter("topic_name", topic_name);
  node->get_parameter("port_name", port_name);
  node->get_parameter("frame_id", frame_id);
  node->get_parameter("laser_scan_dir", laser_scan_dir);
  node->get_parameter("enable_angle_crop_func", enable_angle_crop_func);
  node->get_parameter("angle_crop_min", angle_crop_min);
  node->get_parameter("angle_crop_max", angle_crop_max);

  RCLCPP_INFO(node->get_logger(), " [ldrobot] SDK Pack Version is v2.2.11");
  RCLCPP_INFO(node->get_logger(), " [ldrobot] <product_name>: %s ,<topic_name>: %s ,<port_name>: %s ,<frame_id>: %s", 
              product_name.c_str(), topic_name.c_str(), port_name.c_str(), frame_id.c_str());

  RCLCPP_INFO(node->get_logger(), "[ldrobot] <laser_scan_dir>: %s,<enable_angle_crop_func>: %s,<angle_crop_min>: %f,<angle_crop_max>: %f",
   (laser_scan_dir?"Counterclockwise":"Clockwise"), (enable_angle_crop_func?"true":"false"), angle_crop_min, angle_crop_max);

  LiPkg *lidar = new LiPkg(node, frame_id, laser_scan_dir, enable_angle_crop_func, angle_crop_min, angle_crop_max);
  CmdInterfaceLinux cmd_port;

  if (port_name.empty()) {
    RCLCPP_ERROR(node->get_logger(), " [ldrobot] Can't find %s device.", product_name.c_str());
    exit(EXIT_FAILURE);
  }else {
    RCLCPP_INFO(node->get_logger(), " [ldrobot] Found %s device.", product_name.c_str());
  }

  cmd_port.SetReadCallback([&lidar](const char *byte, size_t len) {
    if (lidar->Parse((uint8_t *)byte, len)) {
      lidar->AssemblePacket();
    }
  });

  if (cmd_port.Open(port_name)) {
    RCLCPP_INFO(node->get_logger(), " [ldrobot] open %s device %s success!", product_name.c_str(), port_name.c_str());
  }else {
    RCLCPP_ERROR(node->get_logger(), " [ldrobot] open %s device %s fail!", product_name.c_str(), port_name.c_str());
    exit(EXIT_FAILURE);
  }

  // create ldlidar data topic and publisher
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
  publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);
  
  rclcpp::WallRate r(10); //10hz
  while (rclcpp::ok()) {
    if (lidar->IsFrameReady()) {
      lidar->ResetFrameReady();
      // publish ldlidar frame data topic message
      publisher->publish(lidar->GetLaserScan());
    }
    r.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "this node of ldlidar_published is end");
  rclcpp::shutdown();

  return 0;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
