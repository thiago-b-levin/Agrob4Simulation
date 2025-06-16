#ifndef WEBOTS_ROS2_PLUGIN_CAM_PUB_HPP
#define WEBOTS_ROS2_PLUGIN_CAM_PUB_HPP

// Standard includes
#include <cmath>
#include <string>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "webots_ros2_driver/WebotsNode.hpp" 
#include "webots_ros2_driver/plugins/Ros2SensorPlugin.hpp" 
#include <webots/robot.h> 
#include <webots/device.h>
#include <webots/camera.h>
#include <opencv2/opencv.hpp> 

// Constants
#define TIME_STEP 32

namespace webots_sensor_publisher {

    class camera_publisher : public webots_ros2_driver::Ros2SensorPlugin {
      public:
      void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
      void step() override;
    
    private:
      void enable();
      void disable();
      void publishImage();
    
      WbDeviceTag mCamera = 0;
      bool mIsEnabled = false;
    
      int mWidth = 0;
      int mHeight = 0;
      std::string mEncoding = "bgra8";
    
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mPublisher;
      sensor_msgs::msg::Image mImageMsg;
      // Add another publisher
      rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr mCompressedPublisher;
      sensor_msgs::msg::CompressedImage mCompressedImageMsg;
    };
    

} // namespace webots_sensor_publisher

#endif // WEBOTS_ROS2_PLUGIN_CAM_PUB_HPP
