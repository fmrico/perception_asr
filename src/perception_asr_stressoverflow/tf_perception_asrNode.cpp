// Copyright 2023 (c) StressOverflow
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

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>

#include "perception_asr_stressoverflow/tf_perception_asrNode.hpp"
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
using namespace std;
namespace perception_asr_stressoverflow
{

using std::placeholders::_1;
using namespace std::chrono_literals;

tf_perception_asrNode::tf_perception_asrNode()
: Node("tf_perception_asrNode"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  detecction_sub = create_subscription<vision_msgs::msg::Detection3DArray>(
    "input_3d", rclcpp::SensorDataQoS(),
    std::bind(&tf_perception_asrNode::tf_callback, this, _1));

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
}

void
tf_perception_asrNode::tf_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg)
{
  for (const auto & detection : msg->detections) {

    if (detection.results.empty()) {
      continue;
    }

    if (detection.results[0].hypothesis.class_id == "person") {
      std::cerr << "Person detected" << std::endl;

      double object_x = detection.bbox.center.position.x;
      double object_y = detection.bbox.center.position.y;
      double object_z = detection.bbox.center.position.z;

      tf2::Transform camera2object;
      camera2object.setOrigin(tf2::Vector3(object_x, object_y, object_z));
      camera2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

      geometry_msgs::msg::TransformStamped odom2camera_msg;
      tf2::Stamped<tf2::Transform> odom2camera;
      try {
        odom2camera_msg = tf_buffer_.lookupTransform(
          "odom", "camera_depth_optical_frame",
          tf2::timeFromSec(rclcpp::Time(msg->header.stamp).seconds()));
        tf2::fromMsg(odom2camera_msg, odom2camera);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Obstacle transform not found: %s", ex.what());
        return;
      }
      tf2::Transform odom2object = odom2camera * camera2object;

      geometry_msgs::msg::TransformStamped odom2object_msg;
      odom2object_msg.transform = tf2::toMsg(odom2object);

      odom2object_msg.header.stamp = msg->header.stamp;
      odom2object_msg.header.frame_id = "odom";
      odom2object_msg.child_frame_id = "detected_obstacle";

      tf_broadcaster_->sendTransform(odom2object_msg);
    }
  }


  /*vision_msgs::msg::Detection3DArray detection_array_msg;
  size_t array_size = detection_array_msg.detections.size();
  for ( int i=0; i< array_size; i++){
    //string name = msg->detections[i].results.hypothesis.class_id;

    double object_x=msg->detections[i].bbox.center.position.x;
    double object_y=msg->detections[i].bbox.center.position.y;
    double object_z=msg->detections[i].bbox.center.position.z;

    if (!std::isinf(object_x)) {
      tf2::Transform camera2object;
      camera2object.setOrigin(tf2::Vector3(object_x, object_y, object_z));
      camera2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

      geometry_msgs::msg::TransformStamped odom2camera_msg;
      tf2::Stamped<tf2::Transform> odom2camera;
      try {
        odom2camera_msg = tf_buffer_.lookupTransform(
          "odom", "camera_link",
          tf2::timeFromSec(rclcpp::Time(msg->header.stamp).seconds() - 0.3));
        tf2::fromMsg(odom2camera_msg, odom2camera);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Obstacle transform not found: %s", ex.what());
        return;
      }

      tf2::Transform odom2object = odom2camera * camera2object;

      geometry_msgs::msg::TransformStamped odom2object_msg;
      odom2object_msg.transform = tf2::toMsg(odom2object);

      odom2object_msg.header.stamp = msg->header.stamp;
      odom2object_msg.header.frame_id = "odom";
      odom2object_msg.child_frame_id = "detected_obstacle";

      tf_broadcaster_->sendTransform(odom2object_msg);
    }
  }*/
}

}  // namespace perception_asr_stressoverflow
