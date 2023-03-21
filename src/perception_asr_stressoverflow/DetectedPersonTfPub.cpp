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

#include "perception_asr_stressoverflow/DetectedPersonTfPub.hpp"
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

DetectedPersonTfPub::DetectedPersonTfPub()
: Node("detected_person_tf_pub"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  detecction_sub = create_subscription<vision_msgs::msg::Detection3DArray>(
    "input_3d", rclcpp::SensorDataQoS(),
    std::bind(&DetectedPersonTfPub::tf_callback, this, _1));

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
}

void
DetectedPersonTfPub::tf_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg)
{
  for (const auto & detection : msg->detections) {

    if (detection.results.empty()) {
      continue;
    }

    if (detection.results[0].hypothesis.class_id == "person") {
      double person_x = detection.bbox.center.position.x;
      double person_y = detection.bbox.center.position.y;
      double person_z = detection.bbox.center.position.z;

      tf2::Transform camera2person;
      camera2person.setOrigin(tf2::Vector3(person_x, person_y, person_z));
      camera2person.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

      geometry_msgs::msg::TransformStamped odom2camera_msg;
      tf2::Stamped<tf2::Transform> odom2camera;
      try {
        odom2camera_msg = tf_buffer_.lookupTransform(
          "odom", "camera_depth_optical_frame",
          tf2::timeFromSec(rclcpp::Time(msg->header.stamp).seconds()));
        tf2::fromMsg(odom2camera_msg, odom2camera);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Person transform not found: %s", ex.what());
        return;
      }
      tf2::Transform odom2person = odom2camera * camera2person;

      geometry_msgs::msg::TransformStamped odom2person_msg;
      odom2person_msg.transform = tf2::toMsg(odom2person);

      odom2person_msg.header.stamp = msg->header.stamp;
      odom2person_msg.header.frame_id = "odom";
      odom2person_msg.child_frame_id = "detected_person";

      tf_broadcaster_->sendTransform(odom2person_msg);
    }
  }
}

}  // namespace perception_asr_stressoverflow
