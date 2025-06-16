#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

// Standard includes
#include <cmath>
#include <string>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Webots includes
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

// Default Constants
#define RAD_TO_DEGREE (180.0 / M_PI)
#define DEGREE_TO_RAD (M_PI / 180.0)
#define DEFAULT_WHEEL_BASE 0.512
#define DEFAULT_WHEEL_RADIUS 0.165
#define TIME_STEP 20  // Timestep in milliseconds

namespace webots_sensor_publisher {

class sensor_publisher: public webots_ros2_driver::PluginInterface {
public:
    // Interface methods
    void init(webots_ros2_driver::WebotsNode *node,
              std::unordered_map<std::string, std::string> &parameters) override;
    void step() override;

private:
    // ROS communication
    webots_ros2_driver::WebotsNode* node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Message storage
    geometry_msgs::msg::Twist cmd_vel_msg_;
    
    
    // Odometry functions 
    void publishOdometry();
    void updateOdometry();
    void updateTF();
    void computeOdometry(double v_left, double v_right, double dt);

    // Parameter loading
    void loadParametersFromFile(const std::string &file_path);
    
    // Sensor management
    void enableSensors();
    void disableSensors();
    double GetSensorData(WbDeviceTag sensor_name);
    
    // Odometry Variables
    double x_, y_, theta_;
    rclcpp::Time last_time_;
    double prev_pos_left_, prev_pos_right_;
    bool first_update_;
        
    // Robot Parameters
    double wheel_radius_;
    double wheel_base_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string left_wheel_sensor_name_;
    std::string right_wheel_sensor_name_;

    // Sensor devices
    // Wheel sensors
    WbDeviceTag right_sensor;
    WbDeviceTag left_sensor; 
};

} // namespace webots_sensor_publisher

#endif // WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP