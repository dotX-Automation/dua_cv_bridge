# dua_cv_bridge

Minimal replacement for `cv_bridge`: conversions between ROS 2 image messages and OpenCV matrices, compiled against the OpenCV that every DUA base image builds from source. Prefer it to `cv_bridge`, which links the distribution's OpenCV.

## Contents

- [x] `dua_cv_bridge::frame_to_msg`: copies a `cv::Mat` (ROIs included) into a new `sensor_msgs/msg/Image`, returned as `Image::UniquePtr`.
- [x] `dua_cv_bridge::frame_to_compressed_msg`: encodes a `cv::Mat` as JPEG (quality `level`, default `60`) into a new `sensor_msgs/msg/CompressedImage`, returned as `CompressedImage::UniquePtr`, with the `<encoding>; jpeg compressed <encoding>` format string that `compressed_image_transport` subscribers expect. Throws `std::runtime_error` if encoding fails.
- [x] `dua_cv_bridge::msg_to_frame`: copies a `sensor_msgs/msg/Image` into a `cv::Mat`, honoring row padding. Supports `mono8`, `bgr8`/`rgb8`, `bgra8`/`rgba8`, `bgr16`/`rgb16`, `bgra16`/`rgba16`, `32FC1`, `64FC1`; channels are not reordered (an `rgb8` message yields an RGB matrix). Throws `std::runtime_error` on other encodings and on messages whose size, step and data do not match.
- [x] `dua_cv_bridge::compressed_msg_to_frame`: decodes a `sensor_msgs/msg/CompressedImage` into a `cv::Mat` as stored in the compressed data, ignoring the format string. Leaves the output untouched if the message holds no data, throws `std::runtime_error` if decoding fails.

The returned messages have no header: set it before publishing.

## Usage

Add the dependency to `package.xml` and `CMakeLists.txt`:

```xml
<depend>dua_cv_bridge</depend>
```

```cmake
find_package(dua_cv_bridge REQUIRED)
ament_target_dependencies(<target> dua_cv_bridge)
```

Then include the header:

```cpp
#include <dua_cv_bridge/dua_cv_bridge.hpp>
```

The conversion functions return a `std::unique_ptr` so that the message can be published without copies, which is what intra-process communication in a component container needs:

```cpp
Image::UniquePtr image_msg = dua_cv_bridge::frame_to_msg(frame, sensor_msgs::image_encodings::BGR8);
image_msg->set__header(header);
image_pub_->publish(std::move(image_msg));
```

Assigning the result to an `Image::SharedPtr` still compiles, but publishing a `SharedPtr` copies the message: keep the `UniquePtr` up to the publisher. All functions are reentrant.

---

## Copyright and License

Copyright 2025 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
