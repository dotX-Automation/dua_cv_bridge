/**
 * DUA cv bridge library base class.
 *
 * dotX Automation s.r.l. <info@dotxautomation.com>
 *
 * March 19, 2025
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <dua_cv_bridge/dua_cv_bridge.hpp>

namespace dua_cv_bridge
{

Image::SharedPtr dua_frame_to_msg(
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

cv::Mat dua_msg_to_frame(const Image::ConstSharedPtr & msg)
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
      "dua_cv_bridge::dua_msg_to_frame - Encoding not supported: " + encoding);
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
