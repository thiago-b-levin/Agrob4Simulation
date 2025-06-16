#include "wb_sensor_publisher/sensor_pub.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace webots_sensor_publisher {

  void sensor_publisher::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

    // Store node handle
    node_ = node;

    // Use the ROS 2 parameter system to load parameters
    wheel_base_ = parameters.count("wheel_base") ? atof(parameters["wheel_base"].c_str()) : DEFAULT_WHEEL_BASE;
    wheel_radius_ = parameters.count("wheel_radius") ? atof(parameters["wheel_radius"].c_str()) : DEFAULT_WHEEL_RADIUS;
    left_wheel_sensor_name_ = parameters.count("left_wheel_sensor") ? parameters["left_wheel_sensor"] : "FWLeft_Joint_Sensor";
    right_wheel_sensor_name_ = parameters.count("right_wheel_sensor") ? parameters["right_wheel_sensor"] : "FWRight_Joint_Sensor";
    odom_frame_ = parameters.count("odom_frame") ? parameters["odom_frame"] : "odom";
    base_frame_ = parameters.count("base_frame") ? parameters["base_frame"] : "base_link";
    RCLCPP_INFO(node_->get_logger(), "Odometry publisher initialized with parameters:");
    RCLCPP_INFO(node_->get_logger(), "  wheel_base: %f", wheel_base_);
    RCLCPP_INFO(node_->get_logger(), "  wheel_radius: %f", wheel_radius_);
    RCLCPP_INFO(node_->get_logger(), "  left_wheel_sensor: %s", left_wheel_sensor_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "  right_wheel_sensor: %s", right_wheel_sensor_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "  odom_frame: %s", odom_frame_.c_str());
    RCLCPP_INFO(node_->get_logger(), "  base_frame: %s", base_frame_.c_str());
    // Initialize odometry variables
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    last_time_ = node_->now();
    first_update_ = true;

    // Get and enable wheel position sensors
    right_sensor = wb_robot_get_device(right_wheel_sensor_name_.c_str());
    left_sensor = wb_robot_get_device(left_wheel_sensor_name_.c_str());
    wb_position_sensor_enable(right_sensor, TIME_STEP);
    wb_position_sensor_enable(left_sensor, TIME_STEP);

    // Create odometry publisher and transform broadcaster
    odom_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS().reliable());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    
  }


  // Publish odometry message
  void sensor_publisher::publishOdometry() {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = node_->now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    // Position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    
    // Convert orientation to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    // Velocity (from cmd_vel)
    odom.twist.twist.linear.x = cmd_vel_msg_.linear.x;
    odom.twist.twist.linear.y = cmd_vel_msg_.linear.y;
    odom.twist.twist.angular.z = cmd_vel_msg_.angular.z;

    // Publish message
    odom_publisher_->publish(odom);
    RCLCPP_DEBUG(node_->get_logger(), "Published odometry: x=%f, y=%f, theta=%f",
                 x_, y_, theta_);
  }

  // Update TF transform for odometry
  void sensor_publisher::updateTF() {
    geometry_msgs::msg::TransformStamped transform;
    
    transform.header.stamp = node_->now();
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    
    // Set translation
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    
    // Set rotation
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    transform.transform.rotation = tf2::toMsg(q);
    
    // Broadcast transform
    tf_broadcaster_->sendTransform(transform);
  }

  // Update odometry calculations
  void sensor_publisher::updateOdometry() {
    // Get current time
    rclcpp::Time current_time = node_->now();
    double dt = (current_time - last_time_).seconds();
    
    // Skip if time delta is too small
    if (dt < 0.001) {
      return;
    }
    
    last_time_ = current_time;

    // Get wheel positions from sensors
    double pos_left = GetSensorData(left_sensor);
    double pos_right = GetSensorData(right_sensor);
    
    // Handle first update specially
    if (first_update_) {
      prev_pos_left_ = pos_left;
      prev_pos_right_ = pos_right;
      first_update_ = false;
      return;  // Skip calculations on first update
    }
    
    // Calculate wheel velocities (change in position / time)
    double v_left = (pos_left - prev_pos_left_) / dt;
    double v_right = (pos_right - prev_pos_right_) / dt;
    
    // Store current positions for next update
    prev_pos_left_ = pos_left;
    prev_pos_right_ = pos_right;

    // Compute odometry
    computeOdometry(v_left, v_right, dt);
  }

  // Compute odometry based on wheel velocities
  void sensor_publisher::computeOdometry(double v_left, double v_right, double dt) {
    // Compute linear and angular velocity
    double v = (wheel_radius_ * v_right + wheel_radius_ * v_left) / 2.0;
    double w = (wheel_radius_ * v_right - wheel_radius_ * v_left) / wheel_base_;

    // Compute new position
    x_ += v * cos(theta_) * dt;
    y_ += v * sin(theta_) * dt;
    theta_ += w * dt;

    // Normalize angle
    if (theta_ > M_PI) {
      theta_ -= 2.0 * M_PI;
    } else if (theta_ < -M_PI) {
      theta_ += 2.0 * M_PI;
    }
  }

  // Get position sensor reading
  double sensor_publisher::GetSensorData(WbDeviceTag sensor_name) {
    return wb_position_sensor_get_value(sensor_name);
  }

  // Enable sensors (if needed)
  void sensor_publisher::enableSensors() {
    wb_position_sensor_enable(left_sensor, TIME_STEP);
    wb_position_sensor_enable(right_sensor, TIME_STEP);
  }

  // Disable sensors (if needed)
  void sensor_publisher::disableSensors() {
    wb_position_sensor_disable(left_sensor);
    wb_position_sensor_disable(right_sensor);
  }

  // Main step function called at each simulation step
  void sensor_publisher::step() {
    // Update odometry based on wheel sensors
    updateOdometry();
    
    // Publish odometry data
    publishOdometry();
    
    // Update and broadcast TF transform
    updateTF();
  }

} // namespace webots_sensor_publisher

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(webots_sensor_publisher::sensor_publisher,
                       webots_ros2_driver::PluginInterface)