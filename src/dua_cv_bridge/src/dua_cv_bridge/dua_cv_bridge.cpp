#include <dua_cv_bridge/dua_cv_bridge.hpp>

namespace dua_cv_bridge
{

Image::SharedPtr DUACVBridge::frame_to_msg(
  const cv::Mat & frame,
  const std::string & encoding)
{
  auto ros_image = std::make_shared<Image>();

  // Set frame-relevant image contents
  ros_image->set__width(frame.cols);
  ros_image->set__height(frame.rows);
  ros_image->set__encoding(encoding);
  ros_image->set__step(frame.cols * frame.elemSize());
  ros_image->set__is_bigendian(false);

  // Copy frame data (this avoids the obsolete cv_bridge)
  size_t size = ros_image->step * frame.rows;
  ros_image->data.resize(size);
  memcpy(ros_image->data.data(), frame.data, size);

  return ros_image;
}

cv::Mat DUACVBridge::msg_to_frame(Image::SharedPtr & msg)
{
  // Determine OpenCV type from ROS2 encoding
  int cv_type = 0;
  const std::string & encoding = msg->encoding;
  if (encoding == "mono8") {
    cv_type = CV_8UC1;
  } else if (encoding == "bgr8" || encoding == "rgb8") {
    cv_type = CV_8UC3;
  } else if (encoding == "bgra8" || encoding == "rgba8") {
    cv_type = CV_8UC4;
  } else {
    throw std::runtime_error(
      "DUACVBridge::msg_to_frame - Encoding not supported: " + encoding);
  }

  // Create OpenCV Mat from ROS2 Image message
  cv::Mat frame(
    msg->height,
    msg->width,
    cv_type,
    (void *)(msg->data.data()));

  return frame.clone();
}


}  // namespace dua_cv_bridge
