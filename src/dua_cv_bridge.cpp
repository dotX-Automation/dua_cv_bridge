/**
 * DUA cv bridge library implementation.
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

#include <dua_cv_bridge/dua_cv_bridge.hpp>

namespace dua_cv_bridge
{

Image::UniquePtr frame_to_msg(
  const cv::Mat & frame,
  const std::string & encoding)
{
  auto ros_image = std::make_unique<Image>();

  // Set frame-relevant image contents
  ros_image->set__width(frame.cols);
  ros_image->set__height(frame.rows);
  ros_image->set__encoding(encoding);
  ros_image->set__step(frame.cols * frame.elemSize());
  ros_image->set__is_bigendian(false);

  // Copy frame data through a view of the message buffer, which handles non-continuous frames
  ros_image->data.resize(ros_image->step * frame.rows);
  cv::Mat data_view(frame.rows, frame.cols, frame.type(), ros_image->data.data());
  frame.copyTo(data_view);

  return ros_image;
}

CompressedImage::UniquePtr frame_to_compressed_msg(
  const cv::Mat & frame,
  const std::string & encoding,
  int level)
{
  auto ros_image = std::make_unique<CompressedImage>();

  // compressed_image_transport subscribers read the channel order from the part after "compressed"
  ros_image->set__format(encoding + "; jpeg compressed " + encoding);

  // Compress image directly into the message
  const std::vector<int> compression_params = {
    cv::IMWRITE_JPEG_QUALITY, level,
    cv::IMWRITE_JPEG_OPTIMIZE, 1};
  if (!cv::imencode(".jpg", frame, ros_image->data, compression_params)) {
    throw std::runtime_error("dua_cv_bridge::frame_to_compressed_msg: Failed to compress image");
  }

  return ros_image;
}

void msg_to_frame(const Image::ConstSharedPtr & msg, cv::Mat & out_frame)
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
  } else if (encoding == "bgr16" || encoding == "rgb16") {
    cv_type = CV_16UC3;
  } else if (encoding == "bgra16" || encoding == "rgba16") {
    cv_type = CV_16UC4;
  } else if (encoding == "64FC1") {
    cv_type = CV_64FC1;
  } else if (encoding == "32FC1") {
    cv_type = CV_32FC1;
  } else {
    throw std::runtime_error(
      "dua_cv_bridge::msg_to_frame: Encoding not supported: " + encoding);
  }

  // Check the message layout before reading its buffer, since it comes from outside
  const size_t row_size = static_cast<size_t>(msg->width) * CV_ELEM_SIZE(cv_type);
  if (msg->step < row_size ||
    msg->step % CV_ELEM_SIZE1(cv_type) != 0 ||
    msg->data.size() < static_cast<size_t>(msg->step) * msg->height)
  {
    throw std::runtime_error(
      "dua_cv_bridge::msg_to_frame: Image size, step and data size do not match");
  }

  // Create OpenCV Mat from ROS2 Image message, honoring row padding
  cv::Mat tmp(
    msg->height,
    msg->width,
    cv_type,
    const_cast<uint8_t *>(msg->data.data()),
    msg->step);

  // Copy data to output frame
  out_frame = tmp.clone();
}

void compressed_msg_to_frame(const CompressedImage::ConstSharedPtr & msg, cv::Mat & out_frame)
{
  // Consistency check
  if (msg->data.empty()) {
    return;
  }

  // Decompress image
  out_frame = cv::imdecode(msg->data, cv::IMREAD_UNCHANGED);
  if (out_frame.empty()) {
    throw std::runtime_error("dua_cv_bridge::compressed_msg_to_frame: Failed to decompress image");
  }
}

} // namespace dua_cv_bridge
