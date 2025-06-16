#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

// Standard includes
#include <cmath>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Webots includes
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"
#include <webots/motor.h>
#include <webots/position_sensor.h>

// Custom includes
#include "husky_controll/differential_calculations.h"

// Constants
#define RAD_TO_DEGREE 180.0 / M_PI
#define DEGREE_TO_RAD M_PI / 180.0
#define DISTANCE_BETWEEN_WHEELS 0.512
#define WHEEL_RADIUS 0.165

namespace webots_driver {

class huskyDriver : public webots_ros2_driver::PluginInterface {
public:
    // Interface methods
    void init(webots_ros2_driver::WebotsNode *node,
              std::unordered_map<std::string, std::string> &parameters) override;
    void step() override;

private:
    // Control state enumeration
    enum ControlState {IDLE, CONTROLLING};
    
    // Callback functions
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    // Control functions
    void controller();
    // ROS communication
    webots_ros2_driver::WebotsNode* node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    
    // Message storage
    geometry_msgs::msg::Twist cmd_vel_msg;
    
    // Motor devices
    WbDeviceTag Fright_motor;
    WbDeviceTag Fleft_motor;
    WbDeviceTag Bright_motor;
    WbDeviceTag Bleft_motor;
    // WbDeviceTag connector_angle_sensor;
    
    // // Controller parameters; 
    // double front_to_joint_dist_ = 0.55;     // distance from front wheels to joint
    // double back_to_joint_dist_ = 0.55;        // distance from back wheels to joint
    // double kp_ = 1;                           // controller proportional component
    // double angle_ref_;                        // controller reference
    // double angle_err_;                        // controller error
    // double control_timeout_ = 0.05;           // timeout (secs) to disable control in case of missing data
    // double min_radius_;                       // min allowed rotation radius
    
    // // Timing
    // rclcpp::Time angle_stamp_;                // last joint angle stamp
    // rclcpp::Time cmd_vel_stamp_;              // last cmd_vel stamp
    
    // State variables
    ControlState control_state_;              // control state machine's state
    
    // Helper objects
    DifferentialCalculations* differential_calculations_;  // differential calculations object
};

} // namespace webots_driver

#endif // WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP