// Copyright 2023 U Power Robotics USA, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UP_FRONTCAM_NODE_HPP_
#define UP_FRONTCAM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace upower
{

class UpFrontCamRefAppNode : public rclcpp::Node
{
public:
  UpFrontCamRefAppNode(const rclcpp::NodeOptions & options);
  void img_receive_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
};

} // namespace upower

#endif  // UP_FRONTCAM_NODE_HPP_