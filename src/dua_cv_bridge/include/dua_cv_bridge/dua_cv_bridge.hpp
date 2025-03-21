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

#pragma once

#include <dua_cv_bridge/visibility_control.h>

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>
#include <stdexcept>

#include <opencv2/core.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace sensor_msgs::msg;
using namespace sensor_msgs::image_encodings;

namespace dua_cv_bridge
{

/**
 * @brief Converts an OpenCV cv::Mat to a ROS2 sensor_msgs::msg::Image message.
 *
 * @param frame      The cv::Mat image data to be converted.
 * @param encoding   The desired ROS2 encoding for the output Image.
 * @return           A sensor_msgs::msg::Image containing the converted data.
 */
Image::SharedPtr DUA_CV_BRIDGE_PUBLIC dua_frame_to_msg(
  const cv::Mat & frame,
  const std::string & encoding);

/**
 * @brief Converts a ROS2 sensor_msgs::msg::Image::SharedPtr message to an OpenCV cv::Mat.
 *
 * @param msg         The ROS Image message to be converted.
 * @param out_frame   The output cv::Mat where the data will be stored.
 */
void DUA_CV_BRIDGE_PUBLIC dua_msg_to_frame(const Image::ConstSharedPtr & msg, cv::Mat & out_frame);

}  // namespace dua_cv_bridge
