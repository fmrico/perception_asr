// Copyright (c) 2023 StressOverflow
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

#ifndef PERCEPTION_ASR_STRESSOVERFLOW__DARKNETDETECTIONNODE_HPP_
#define PERCEPTION_ASR_STRESSOVERFLOW__DARKNETDETECTIONNODE_HPP_

#include <memory>

#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include "rclcpp/rclcpp.hpp"

namespace perception_asr_stressoverflow
{

class DarknetDetectionNode : public rclcpp::Node
{
public:
  DarknetDetectionNode();

private:
  void detection_callback(const darknet_ros_msgs::msg::BoundingBoxes::ConstSharedPtr & msg);

  rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr detection_bbxs_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
};

}  // namespace perception_asr_stressoverflow

#endif  // PERCEPTION_ASR_STRESSOVERFLOW__DARKNETDETECTIONNODE_HPP_
