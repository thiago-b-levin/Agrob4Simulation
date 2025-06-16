#ifndef WEBOTS_ROS2_PLUGIN_SCAN_PUB_HPP
#define WEBOTS_ROS2_PLUGIN_SCAN_PUB_HPP

// Standard includes
#include <cmath>
#include <string>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "webots_ros2_driver/WebotsNode.hpp" 
#include "webots_ros2_driver/PluginInterface.hpp" 
#include <webots/robot.h> 
#include <webots/device.h>
#include <webots/inertial_unit.h>

// Constants
#define TIME_STEP 32

namespace webots_sensor_publisher {

    class imupublisher : public webots_ros2_driver::PluginInterface {
        public:
          void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
          void step() override;
        
        private:
          webots_ros2_driver::WebotsNode *mNode;
          WbDeviceTag mInertialUnit;
          rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr mPublisher;
          std::string mFrameName = "imu_link";
          std::string mTopicName = "/imu/rpy";
        };

} // namespace webots_sensor_publisher

#endif // WEBOTS_ROS2_PLUGIN_SCAN_PUB_HPP
