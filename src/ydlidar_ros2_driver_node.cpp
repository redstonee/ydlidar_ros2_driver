/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

#define ROS2Verision "1.0.1"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());

  CYdLidar laser;
  auto lidarPort = node->declare_parameter("lidarPort", "/dev/ydlidar"); // Serial port
  auto ignoreArray = node->declare_parameter("ignore_array", "");        // IDK what this is
  auto frame_id = node->declare_parameter("frame_id", "laser_frame");

  auto baudrate = node->declare_parameter("baudrate", 230400);
  auto lidarType = node->declare_parameter("lidar_type", (int)TYPE_TRIANGLE);              // TOF lidar
  auto deviceInterface = node->declare_parameter("device_type", (int)YDLIDAR_TYPE_SERIAL); // Serial/TCP/UDP
  auto sampleRate = node->declare_parameter("sample_rate", 9);
  auto abnormalCheckCnt = node->declare_parameter("abnormal_check_count", 4);
  auto intensityBit = node->declare_parameter("intensity_bit", 8); // Intensity bit count

  auto fixedResolution = node->declare_parameter("fixed_resolution", false); // Fixed angle resolution
  auto propReversion = node->declare_parameter("reversion", true);           // Rotate 180
  auto inverted = node->declare_parameter("inverted", true);                 // Counterclockwise
  auto autoReconn = node->declare_parameter("auto_reconnect", true);         // Auto reconnect
  auto singleChan = node->declare_parameter("isSingleChannel", false);       // One-way communication
  auto intensity = node->declare_parameter("intensity", false);              // Intensity
  auto supportMotorDTR = node->declare_parameter("support_motor_dtr", false);
  auto angleMax = node->declare_parameter("angle_max", 180.f);  // Maximum angle in degrees
  auto angleMin = node->declare_parameter("angle_min", -180.f); // Minimum angle in degrees
  auto rangeMax = node->declare_parameter("range_max", 64.f);   // Maximum range in meters
  auto rangeMin = node->declare_parameter("range_min", 0.01f);   // Minimum range in meters
  auto scanFreq = node->declare_parameter("frequency", 10.f);   // Scan frequency

  laser.setlidaropt(LidarPropSerialPort, lidarPort.c_str(), lidarPort.size());
  laser.setlidaropt(LidarPropIgnoreArray, ignoreArray.c_str(), ignoreArray.size());
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
  laser.setlidaropt(LidarPropLidarType, &lidarType, sizeof(int));
  laser.setlidaropt(LidarPropDeviceType, &deviceInterface, sizeof(int));
  laser.setlidaropt(LidarPropSampleRate, &sampleRate, sizeof(int));
  laser.setlidaropt(LidarPropAbnormalCheckCount, &abnormalCheckCnt, sizeof(int));
  laser.setlidaropt(LidarPropIntenstiyBit, &intensityBit, sizeof(int));
  laser.setlidaropt(LidarPropFixedResolution, &fixedResolution, sizeof(bool));
  laser.setlidaropt(LidarPropReversion, &propReversion, sizeof(bool));
  laser.setlidaropt(LidarPropInverted, &inverted, sizeof(bool));
  laser.setlidaropt(LidarPropAutoReconnect, &autoReconn, sizeof(bool));
  laser.setlidaropt(LidarPropSingleChannel, &singleChan, sizeof(bool));
  laser.setlidaropt(LidarPropIntenstiy, &intensity, sizeof(bool));
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &supportMotorDTR, sizeof(bool));
  laser.setlidaropt(LidarPropMaxAngle, &angleMax, sizeof(float));
  laser.setlidaropt(LidarPropMinAngle, &angleMin, sizeof(float));
  laser.setlidaropt(LidarPropMaxRange, &rangeMax, sizeof(float));
  laser.setlidaropt(LidarPropMinRange, &rangeMin, sizeof(float));
  laser.setlidaropt(LidarPropScanFrequency, &scanFreq, sizeof(float));

  bool ret = laser.initialize();
  if (ret)
  {
    ret = laser.turnOn();
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "%s\n", laser.DescribeError());
  }

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  auto stop_scan_service =
      [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<std_srvs::srv::Empty::Request> req,
               std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    UNUSED(request_header);
    UNUSED(req);
    UNUSED(response);
    return laser.turnOff();
  };

  auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan", stop_scan_service);

  auto start_scan_service =
      [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<std_srvs::srv::Empty::Request> req,
               std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    UNUSED(request_header);
    UNUSED(req);
    UNUSED(response);
    return laser.turnOn();
  };

  auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan", start_scan_service);

  rclcpp::WallRate loop_rate(20);

  while (ret && rclcpp::ok())
  {

    LaserScan scan; //

    if (laser.doProcessSimple(scan))
    {

      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_msg->header.stamp.nanosec = scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;

      int size = (scan.config.max_angle - scan.config.min_angle) / scan.config.angle_increment + 1;
      scan_msg->ranges.resize(size);
      scan_msg->intensities.resize(size);
      for (size_t i = 0; i < scan.points.size(); i++)
      {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle) / scan.config.angle_increment);
        if (index >= 0 && index < size)
        {
          scan_msg->ranges[index] = scan.points[i].range;
          scan_msg->intensities[index] = scan.points[i].intensity;
        }
      }

      laser_pub->publish(*scan_msg);
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }
    if (!rclcpp::ok())
    {
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}
