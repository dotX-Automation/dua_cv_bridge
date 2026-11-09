/**
 * DUA cv bridge library.
 *
 * dotX Automation s.r.l. <info@dotxautomation.com>
 *
 * March 19, 2025
 */

/**
 * Copyright 2025 dotX Automation s.r.l.
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

#include "visibility_control.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <sensor_msgs/image_encodings.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace dua_cv_bridge
{

using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;

/**
 * @brief Converts an OpenCV matrix to a sensor_msgs::msg::Image message, copying its pixels.
 *
 * @param frame The matrix to convert, of any type and layout (ROIs included).
 * @param encoding The ROS encoding of the matrix data, not checked against its type.
 * @return A new message, without header, ready to be published with std::move.
 */
[[nodiscard]] Image::UniquePtr DUA_CV_BRIDGE_PUBLIC frame_to_msg(
  const cv::Mat & frame,
  const std::string & encoding);

/**
 * @brief Compresses an OpenCV matrix to JPEG in a sensor_msgs::msg::CompressedImage message,
 *        with the format string expected by compressed_image_transport.
 *
 * @param frame The matrix to compress, with channels in the order of the encoding.
 * @param encoding The ROS encoding of the matrix data.
 * @param level The JPEG quality, in [0, 100].
 * @return A new message, without header, ready to be published with std::move.
 * @throws std::runtime_error if compression fails.
 */
[[nodiscard]] CompressedImage::UniquePtr DUA_CV_BRIDGE_PUBLIC frame_to_compressed_msg(
  const cv::Mat & frame,
  const std::string & encoding,
  int level = 60);

/**
 * @brief Converts a sensor_msgs::msg::Image message to an OpenCV matrix, copying its pixels.
 *
 * @param msg The message to convert; channels are not reordered.
 * @param out_frame The output matrix.
 * @throws std::runtime_error if the encoding is not supported or the message is malformed.
 */
void DUA_CV_BRIDGE_PUBLIC msg_to_frame(const Image::ConstSharedPtr & msg, cv::Mat & out_frame);

/**
 * @brief Decompresses a sensor_msgs::msg::CompressedImage message to an OpenCV matrix.
 *
 * @param msg The message to decompress; nothing is done if it holds no data.
 * @param out_frame The output matrix.
 * @throws std::runtime_error if decompression fails.
 */
void DUA_CV_BRIDGE_PUBLIC compressed_msg_to_frame(
  const CompressedImage::ConstSharedPtr & msg,
  cv::Mat & out_frame);

} // namespace dua_cv_bridge
