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

#ifndef PERCEPTION_ASR_STRESSOVERFLOW__DETECTED_PERSON_TF_PUB_HPP_
#define PERCEPTION_ASR_STRESSOVERFLOW__DETECTED_PERSON_TF_PUB_HPP_

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "image_geometry/pinhole_camera_model.h"
#include "cv_bridge/cv_bridge.h"
#include "depth_image_proc/depth_traits.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"

namespace perception_asr_stressoverflow
{

class DetectedPersonTfPub : public rclcpp::Node
{
public:
  DetectedPersonTfPub();

private:
  void tf_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg);

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detecction_sub;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // perception_asr_stressoverflow

#endif  // PERCEPTION_ASR_STRESSOVERFLOW__DETECTED_PERSON_TF_PUB_HPP_
