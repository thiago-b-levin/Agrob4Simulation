#include "husky_controll/MyRobotDriver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>
#include <cmath>

namespace webots_driver {

// Initialization
void huskyDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {
    
  this->node_ = node;
  
  // Get device handles
  Fright_motor = wb_robot_get_device("front_right_wheel_joint");
  Fleft_motor = wb_robot_get_device("front_left_wheel_joint");
  Bright_motor = wb_robot_get_device("rear_right_wheel_joint");
  Bleft_motor = wb_robot_get_device("rear_left_wheel_joint");
  

  //print controller initialized
  RCLCPP_INFO(rclcpp::get_logger("huskyDriver"), "Husky controller initialized");
  
  // Initialize motors
  wb_motor_set_position(Fleft_motor, INFINITY);
  wb_motor_set_velocity(Fleft_motor, 0.0);
  wb_motor_set_position(Fright_motor, INFINITY);
  wb_motor_set_velocity(Fright_motor, 0.0);
  wb_motor_set_position(Bleft_motor, INFINITY);
  wb_motor_set_velocity(Bleft_motor, 0.0);
  wb_motor_set_position(Bright_motor, INFINITY);
  wb_motor_set_velocity(Bright_motor, 0.0);
  
  // Initialize control variables
  control_state_ = IDLE;
  
  // Create cmd_vel subscription
  cmd_vel_subscription_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      std::bind(&huskyDriver::cmdVelCallback, this, std::placeholders::_1));
  
  // Initialize differential calculations
  differential_calculations_ = new DifferentialCalculations(WHEEL_RADIUS, DISTANCE_BETWEEN_WHEELS);
}

// Callback for cmd_vel messages
// Callback for cmd_vel messages
void huskyDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  // Directly copy the velocities without applying any limit
  cmd_vel_msg = *msg;
  RCLCPP_INFO(rclcpp::get_logger("huskyDriver"), "Received cmd_vel: linear.x=%f, angular.z=%f" 
              , cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
  // // Update timestamp when new command is received
  // cmd_vel_stamp_ = node_->now();
}

// Step function
void huskyDriver::step() {
  controller();
}

// Main controller function
void huskyDriver::controller() {
  switch (control_state_) {
  case IDLE:
    // Stop all motors
    wb_motor_set_velocity(Fleft_motor, 0.0);
    wb_motor_set_velocity(Fright_motor, 0.0);
    wb_motor_set_velocity(Bleft_motor, 0.0);
    wb_motor_set_velocity(Bright_motor, 0.0);

    if (cmd_vel_msg.linear.x != 0.0 || cmd_vel_msg.angular.z != 0.0) {
      // Reset timestamps when leaving IDLE state
      control_state_ = CONTROLLING;
    }
    break;
  
  case CONTROLLING:
    // // Calculate durations since last updates
    // rclcpp::Duration angle_duration = node_->now() - angle_stamp_;
    // rclcpp::Duration cmd_vel_duration = node_->now() - cmd_vel_stamp_;
    
    // // Check for cmd_vel timeout
    // if (cmd_vel_duration.seconds() > control_timeout_) {
    //   RCLCPP_WARN(rclcpp::get_logger("huskyDriver"), 
    //               "cmd_vel reception has timed out (%f), sending to IDLE", 
    //               cmd_vel_duration.seconds());
      
    //   // Reset velocities and go to IDLE
    //   cmd_vel_msg.linear.x = 0.0;
    //   cmd_vel_msg.angular.z = 0.0;
    //   control_state_ = IDLE;
    //   return;
    // }
    // If command is zero, go to IDLE state
    if (cmd_vel_msg.linear.x == 0.0 && cmd_vel_msg.angular.z == 0.0) {
      control_state_ = IDLE;
      return;
    }
    
    // Process the command
    geometry_msgs::msg::Twist last_cmd_vel = cmd_vel_msg;
    
    // // Check for invalid sensor data
    // if (std::isnan(angle_measured)) {
    //   RCLCPP_ERROR(rclcpp::get_logger("huskyDriver"), "ERROR: NaN detected in sensor data");
    //   cmd_vel_msg.linear.x = 0.0;
    //   cmd_vel_msg.angular.z = 0.0;
    //   control_state_ = IDLE;
    //   return;
    // }
    
    // Calculate wheel velocities
    double front_right_wheel_vel, front_left_wheel_vel;
    differential_calculations_->getWheelsSpeed(
        last_cmd_vel.linear.x, 
        last_cmd_vel.angular.z, 
        front_right_wheel_vel, 
        front_left_wheel_vel);
    
    // Ensure positive values
    //front_right_wheel_vel = std::fabs(front_right_wheel_vel);
    //front_left_wheel_vel = std::fabs(front_left_wheel_vel);
    
    // Set up motor velocities
    std::vector<double> front_motor_vel = {front_left_wheel_vel, -front_right_wheel_vel};
    std::vector<double> back_motor_vel = {front_left_wheel_vel, -front_right_wheel_vel};
    
    // Apply steering controller
    
    // Check for invalid motor velocities
    if (std::isnan(front_motor_vel[0]) || std::isnan(front_motor_vel[1]) ||
        std::isnan(back_motor_vel[0]) || std::isnan(back_motor_vel[1])) {
      RCLCPP_ERROR(rclcpp::get_logger("huskyDriver"), "ERROR: NaN detected in motor velocity");
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.0;
      control_state_ = IDLE;
      return;
    }
    

    
    // Set motor velocities
    wb_motor_set_velocity(Fleft_motor, front_motor_vel[0]);
    wb_motor_set_velocity(Fright_motor, front_motor_vel[1]);
    wb_motor_set_velocity(Bleft_motor, back_motor_vel[0]);
    wb_motor_set_velocity(Bright_motor, back_motor_vel[1]);
    // // Update timestamp to indicate we've processed this command
    // cmd_vel_stamp_ = node_->now();
    break;
  }
}



} // namespace webots_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(webots_driver::huskyDriver, webots_ros2_driver::PluginInterface)