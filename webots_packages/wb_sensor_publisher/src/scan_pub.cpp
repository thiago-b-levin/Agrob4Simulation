#include "wb_sensor_publisher/scan_pub.hpp"

namespace webots_scan_publisher {

void scan_publisher::init(webots_ros2_driver::WebotsNode *node,
                          std::unordered_map<std::string, std::string> &parameters) {
  Ros2SensorPlugin::init(node, parameters);  // sets mNode, mTopicName, mFrameName, etc.

  if (!parameters.count("lidarName"))
    throw std::runtime_error("Missing required parameter 'lidarName'");

  lidar_ = wb_robot_get_device(parameters["lidarName"].c_str());
  if (!lidar_ || wb_device_get_node_type(lidar_) != WB_NODE_LIDAR)
    throw std::runtime_error("Invalid or missing Lidar device");

  width_ = wb_lidar_get_horizontal_resolution(lidar_);
  height_ = wb_lidar_get_number_of_layers(lidar_);
  fov_ = wb_lidar_get_fov(lidar_);
  vertical_fov_ = wb_lidar_get_vertical_fov(lidar_);

  lidar_publisher_ = mNode->create_publisher<sensor_msgs::msg::PointCloud2>(
      mTopicName, rclcpp::SensorDataQoS().reliable());

  cloud_msg_.header.frame_id = mFrameName;

  if (mAlwaysOn) {
    enable();
    mIsEnabled = true;
  }

  RCLCPP_INFO(mNode->get_logger(), "Lidar Publisher Initialized! (%dx%d, FOV: %.2f)", width_, height_, fov_);
}

void scan_publisher::enable() {
  wb_lidar_enable(lidar_, mPublishTimestepSyncedMs);
  wb_lidar_enable_point_cloud(lidar_);
}

void scan_publisher::disable() {
  wb_lidar_disable(lidar_);
}

void scan_publisher::step() {
  if (!preStep())
    return;

  if (mIsEnabled)
    readLidarData();

  if (mAlwaysOn)
    return;

  enable();
  mIsEnabled = true;
}

void scan_publisher::readLidarData() {
  const WbLidarPoint *point_cloud = wb_lidar_get_point_cloud(lidar_);
  if (!point_cloud)
    return;

  cloud_msg_.header.stamp = mNode->get_clock()->now();
  cloud_msg_.header.frame_id = mFrameName;
  cloud_msg_.height = 1;
  cloud_msg_.width = width_ * height_;
  cloud_msg_.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg_);
  modifier.setPointCloud2Fields(4,  // Removed "ring"
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg_, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg_, "intensity");

  for (int i = 0; i < width_ * height_; ++i) {
    *iter_x = point_cloud[i].x;
    *iter_y = point_cloud[i].y;
    *iter_z = point_cloud[i].z;

    float distance = std::sqrt(
        point_cloud[i].x * point_cloud[i].x + 
        point_cloud[i].y * point_cloud[i].y + 
        point_cloud[i].z * point_cloud[i].z);

    float max_range = wb_lidar_get_max_range(lidar_);
    *iter_intensity = 255.0f * (1.0f - std::min(distance / max_range, 1.0f));

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }

  lidar_publisher_->publish(cloud_msg_);
}

}  // namespace webots_scan_publisher

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(webots_scan_publisher::scan_publisher, webots_ros2_driver::PluginInterface)
