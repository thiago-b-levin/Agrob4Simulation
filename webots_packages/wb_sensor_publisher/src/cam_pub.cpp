// imu_mag_publisher.cpp
#include "wb_sensor_publisher/cam_pub.hpp"

namespace webots_sensor_publisher {

void camera_publisher::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
  Ros2SensorPlugin::init(node, parameters);  // sets mNode, mTopicName, mFrameName, etc.

  if (!parameters.count("cameraName"))
    throw std::runtime_error("Missing required parameter 'cameraName'");

  mCamera = wb_robot_get_device(parameters["cameraName"].c_str());
  if (!mCamera || wb_device_get_node_type(mCamera) != WB_NODE_CAMERA)
    throw std::runtime_error("Invalid or missing Camera device.");

  mWidth = wb_camera_get_width(mCamera);
  mHeight = wb_camera_get_height(mCamera);
  mEncoding = "bgra8";  // Webots uses BGRA format

  mPublisher = mNode->create_publisher<sensor_msgs::msg::Image>(
    mTopicName + "/image_raw", rclcpp::SensorDataQoS().reliable());

  mCompressedPublisher = mNode->create_publisher<sensor_msgs::msg::CompressedImage>(
  mTopicName + "/compressed", rclcpp::SensorDataQoS().reliable());
    
  mCompressedImageMsg.header.frame_id = mFrameName;
  mCompressedImageMsg.format = "jpeg";  
    

  mImageMsg.header.frame_id = mFrameName;
  mImageMsg.height = mHeight;
  mImageMsg.width = mWidth;
  mImageMsg.encoding = mEncoding;
  mImageMsg.is_bigendian = false;
  mImageMsg.step = mWidth * 4;  // 4 bytes per pixel (BGRA)
  mImageMsg.data.resize(mImageMsg.step * mHeight);

  if (mAlwaysOn) {
    enable();
    mIsEnabled = true;
  }
}

void camera_publisher::enable() {
  wb_camera_enable(mCamera, mPublishTimestepSyncedMs);
}

void camera_publisher::disable() {
  wb_camera_disable(mCamera);
}

void camera_publisher::step() {
  if (!preStep())
    return;

  if (mIsEnabled)
    publishImage();

  if (mAlwaysOn)
    return;

  bool shouldBeEnabled = mPublisher->get_subscription_count() > 0 || mCompressedPublisher->get_subscription_count() > 0;
  
  if (shouldBeEnabled != mIsEnabled) {
    if (shouldBeEnabled)
      enable();
    else
      disable();
    mIsEnabled = shouldBeEnabled;
  }
}

void camera_publisher::publishImage() {
  mImageMsg.header.stamp = mNode->get_clock()->now();

  const unsigned char *image = wb_camera_get_image(mCamera);
  std::copy(image, image + mImageMsg.data.size(), mImageMsg.data.begin());

  mPublisher->publish(mImageMsg);
// Convert raw Webots BGRA image to OpenCV BGR format
cv::Mat bgra(mHeight, mWidth, CV_8UC4, (void *)image);
cv::Mat bgr;
cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);

// Encode as JPEG
std::vector<uchar> buf;
cv::imencode(".jpg", bgr, buf);

// Create CompressedImage message
sensor_msgs::msg::CompressedImage compressed_msg;
compressed_msg.header.stamp = mNode->get_clock()->now();
compressed_msg.header.frame_id = mFrameName;
compressed_msg.format = "jpeg";
compressed_msg.data = buf;

mCompressedPublisher->publish(compressed_msg);
}

}  // namespace webots_ros2_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(webots_sensor_publisher::camera_publisher, webots_ros2_driver::PluginInterface)
