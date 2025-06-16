#ifndef WB_SENSOR_PUBLISHER_SCAN_PUB_HPP
#define WB_SENSOR_PUBLISHER_SCAN_PUB_HPP

#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots_ros2_driver/plugins/Ros2SensorPlugin.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <webots/device.h>
#include <webots/lidar.h>
#include <webots/robot.h>

namespace webots_scan_publisher {

class scan_publisher : public webots_ros2_driver::Ros2SensorPlugin {
public:
  void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
  void step() override;


private:
  void readLidarData();
  void enable();
  void disable();
  WbDeviceTag lidar_ = 0;
  int width_ = 0;
  int height_ = 0;
  double fov_ = 0.0;
  double vertical_fov_ = 0.0;

  bool mIsEnabled = false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_publisher_;
  sensor_msgs::msg::PointCloud2 cloud_msg_;
};

}  // namespace webots_scan_publisher

#endif  // WB_SENSOR_PUBLISHER_SCAN_PUB_HPP
