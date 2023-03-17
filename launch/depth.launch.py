# Copyright (c) 2023 StressOverflow
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# import os

# from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # darknet_dir = get_package_share_directory('darknet_ros')

    # darknet_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(darknet_dir,
    #                                                'launch',
    #                                                'darknet_ros.launch.py')
    #                                  ))

    # params_file = os.path.join(
    #     get_package_share_directory('perception_asr_stressoverflow'),
    #     )

    perception_asr_cmd = Node(package='perception_asr_stressoverflow',
                              executable='darknet_detection',
                              output='screen',
                              parameters=[{
                                'use_sim_time': False
                              }],
                              remappings=[
                                ('input_bbxs_detection', '/darknet_ros/bounding_boxes'),
                                ('output_detection_2d', '/output_detection_2d'),
                              ])

    perception_asr2_cmd = Node(package='perception_asr_stressoverflow',
                               executable='darknet_detection',
                               output='screen',
                               parameters=[{
                                 'use_sim_time': False
                               }],
                               remappings=[
                                 # ('camera_info', '/camera/depth/camera_info'),
                                 ('input_depth', '/camera/depth/image_raw'),
                                 ('input_detection_2d', '/darknet_ros/bounding_boxes'),
                                 ('output_detection_2d', '/output_detection_2d'),
                               ])

    detectionTo3DfromDepth_cmd = Node(package='perception_asr_stressoverflow',
                                      executable='detection_2d_to_3d_depth',
                                      output='screen',
                                      parameters=[{
                                        'use_sim_time': False
                                      }],
                                      remappings=[
                                        ('camera_info', '/camera/depth/camera_info'),
                                        ('input_depth', '/camera/depth/image_raw'),
                                        ('input_detection_2d', '/output_detection_2d'),
                                        ('output_detection_3d', '/output_detection_3d'),
                                      ])

    tf_cmd = Node(package='perception_asr_stressoverflow',
                  executable='tf_perception_asr',
                  output='screen',
                  parameters=[{
                    'use_sim_time': False
                  }],
                  remappings=[
                    ('input_3d', '/output_detection_3d'),
                  ])

    # HSVfilter_cmd = Node(package='perception_asr_stressoverflow',
    #                           executable='hsv_filter',
    #                           output='screen',
    #                           parameters=[{
    #                             'use_sim_time': False
    #                           }],
    #                           remappings=[
    #                             ('camera_info', '/camera/depth/camera_info'),
    #                             ('input_depth', '/camera/depth/image_raw'),
    #                             ('output_detection_3d', '/darknet_ros/bounding_boxes'),
    #                           ])

    ld = LaunchDescription()
    # ld.add_action(darknet_cmd)
    ld.add_action(perception_asr_cmd)
    ld.add_action(perception_asr2_cmd)
    ld.add_action(detectionTo3DfromDepth_cmd)
    ld.add_action(tf_cmd)
    # ld.add_action(HSVfilter_cmd)

    return ld
