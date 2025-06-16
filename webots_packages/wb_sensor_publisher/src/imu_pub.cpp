// imu_mag_publisher.cpp
#include "wb_sensor_publisher/imu_pub.hpp"

namespace webots_sensor_publisher {

void imupublisher::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
  mNode = node;
  mFrameName = "imu_link";
  mTopicName = "/imu/rpy";

  mPublisher = mNode->create_publisher<geometry_msgs::msg::Vector3Stamped>(
  mTopicName, rclcpp::SensorDataQoS().reliable());

  std::string imuName = parameters.count("inertialUnitName") ? parameters["inertialUnitName"] : "imu inertial";
  mInertialUnit = wb_robot_get_device(imuName.c_str());

  if (!mInertialUnit || wb_device_get_node_type(mInertialUnit) != WB_NODE_INERTIAL_UNIT)
    throw std::runtime_error("Could not find a valid InertialUnit device with name: " + imuName);

  wb_inertial_unit_enable(mInertialUnit, 32); 
}

void imupublisher::step() {
  const double *orientation = wb_inertial_unit_get_roll_pitch_yaw(mInertialUnit);

  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = mNode->get_clock()->now();
  msg.header.frame_id = mFrameName;


  msg.vector.x = orientation[0]; // roll
  msg.vector.y = orientation[1]; // pitch
  msg.vector.z = orientation[2]; // yaw

  mPublisher->publish(msg);
}

}  // namespace webots_ros2_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(webots_sensor_publisher::imupublisher, webots_ros2_driver::PluginInterface)
