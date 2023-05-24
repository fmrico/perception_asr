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

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    perception_asr_stressoverflow_dir = get_package_share_directory(
                                        'perception_asr_stressoverflow')

    ir_robots_dir = get_package_share_directory(
                                        'ir_robots')

    config = os.path.join(perception_asr_stressoverflow_dir, 'config', 'params.yaml')
    ir_robots_config = os.path.join(ir_robots_dir, 'config', 'params.yaml')

    with open(config, "r") as stream:
        try:
            conf = (yaml.safe_load(stream))

        except yaml.YAMLError as exc:
            print(exc)

    with open(ir_robots_config, "r") as stream:
        try:
            ir_conf = (yaml.safe_load(stream))

        except yaml.YAMLError as exc:
            print(exc)

    launch_darknet = conf['perception_asr_stressoverflow']['launchers']['launch_darknet']
    kobuki_camera = ir_conf['ir_robots']['kobuki_camera']
    sim_kobuki = ir_conf['ir_robots']['simulation']

    if launch_darknet:
        darknet_dir = get_package_share_directory('darknet_ros')

        if sim_kobuki:
            camera_remap = 'camera/image_raw'
        else:
            if 'xtion' in kobuki_camera:
                camera_remap = 'camera/rgb/image_raw'
            elif 'astra' in kobuki_camera:
                camera_remap = 'camera/color/image_raw'
            else:
                camera_remap = 'image_raw'

        darknet_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(darknet_dir,
                                                       'launch',
                                                       'darknet_ros.launch.py')),
            launch_arguments={
              'image': camera_remap,
            }.items())

        ld.add_action(darknet_cmd)

    perception_asr_cmd = Node(package='perception_asr_stressoverflow',
                              executable='darknet_detection',
                              output='screen',
                              parameters=[{
                                'use_sim_time': sim_kobuki
                              }],
                              remappings=[
                                ('input_bbxs_detection', '/darknet_ros/bounding_boxes'),
                                ('output_detection_2d', '/output_detection_2d'),
                              ])

    ld.add_action(perception_asr_cmd)

    detectionTo3DfromDepth_cmd = Node(package='perception_asr_stressoverflow',
                                      executable='detection_2d_to_3d_depth',
                                      output='screen',
                                      parameters=[{
                                        'use_sim_time': sim_kobuki
                                      }],
                                      remappings=[
                                        ('camera_info', '/camera/depth/camera_info'),
                                        ('input_depth', '/camera/depth/image_raw'),
                                        ('input_detection_2d', '/output_detection_2d'),
                                        ('output_detection_3d', '/output_detection_3d'),
                                      ])

    ld.add_action(detectionTo3DfromDepth_cmd)

    person_tf_cmd = Node(package='perception_asr_stressoverflow',
                         executable='detected_person_tf_pub',
                         output='screen',
                         parameters=[{
                           'use_sim_time': sim_kobuki
                         }],
                         remappings=[
                           ('input_3d', '/output_detection_3d'),
                         ])

    ld.add_action(person_tf_cmd)

    person_monitor_cmd = Node(package='perception_asr_stressoverflow',
                              executable='detected_person_monitor',
                              output='screen',
                              parameters=[{
                                 'use_sim_time': sim_kobuki
                              }],)

    ld.add_action(person_monitor_cmd)

    chair_tf_cmd = Node(package='perception_asr_stressoverflow',
                        executable='detected_chair_tf_pub',
                        output='screen',
                        parameters=[{
                          'use_sim_time': sim_kobuki
                        }],
                        remappings=[
                          ('input_3d', '/output_detection_3d'),
                        ])

    ld.add_action(chair_tf_cmd)

    chair_monitor_cmd = Node(package='perception_asr_stressoverflow',
                             executable='detected_chair_monitor',
                             output='screen',
                             parameters=[{
                                 'use_sim_time': sim_kobuki
                             }],)

    ld.add_action(chair_monitor_cmd)

    bag_tf_cmd = Node(package='perception_asr_stressoverflow',
                        executable='detected_bag_tf_pub',
                        output='screen',
                        parameters=[{
                          'use_sim_time': sim_kobuki
                        }],
                        remappings=[
                          ('input_3d', '/output_detection_3d'),
                        ])

    ld.add_action(bag_tf_cmd)

    bag_monitor_cmd = Node(package='perception_asr_stressoverflow',
                             executable='detected_bag_monitor',
                             output='screen',
                             parameters=[{
                                 'use_sim_time': sim_kobuki
                             }],)

    ld.add_action(bag_monitor_cmd)

    return ld
