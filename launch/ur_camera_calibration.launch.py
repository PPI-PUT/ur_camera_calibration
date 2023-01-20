# Copyright 2023 Piotr Kicki
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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_prefix = get_package_share_directory("ur_camera_calibration")

    # configs
    config_param = DeclareLaunchArgument(
        'config_param_file',
        default_value=[pkg_prefix, '/param/defaults.param.yaml'],
        description='Node config.'
    )

    # nodes and launches
    ur_camera_calibration_node = Node(
            name='ur_camera_calibration_node',
            namespace='',
            package='ur_camera_calibration',
            executable='ur_camera_calibration_node.py',
            parameters=[
                LaunchConfiguration('config_param_file'),
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
            emulate_tty=True
    )

    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('azure_kinect_ros_driver'), 'launch'),
         '/driver.launch.py']),
        launch_arguments={
            'depth_enabled': "true",
            'depth_mode': "WFOV_UNBINNED",
            'depth_unit': "32FC1",
            'color_enabled': "true",
            'color_format': "bgra",
            'color_resolution': "720P",
            'fps': "15",
            'point_cloud': "true",
            'rgb_point_cloud': "true",
            'point_cloud_in_depth_frame': "false",
            'sensor_sn': "",
            'recording_file': "",
            'recording_loop_enabled': "false",
            'body_tracking_enabled': "false",
            'body_tracking_smoothing_factor': "0.0",
            'rescale_ir_to_mono8': "false",
            'ir_mono8_scaling_factor': "1.0",
            'imu_rate_target': "0",
            'wired_sync_mode': "0",
            'subordinate_delay_off_master_usec': "0",
        }.items()
    )

    apriltag_config_file = os.path.join(
                                get_package_share_directory('ur_camera_calibration'),
                                'param',
                                'april_tag.yaml')
    apriltag_detector_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('apriltag_ros'), 'launch'),
            '/tag_camera.launch.py']),
        launch_arguments=dict(camera_name='/rgb',
                              image_topic="image_raw",
                              config_file=apriltag_config_file,
                              ).items(),
    )

    # events and wrokflow
    calibration_finished_event = RegisterEventHandler(
        OnProcessExit(
            target_action=ur_camera_calibration_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Calibration finished'))
            ]
        )
    )

    return LaunchDescription([
        config_param,
        camera_node,
        apriltag_detector_node,
        ur_camera_calibration_node,
        #calibration_finished_event,
        ])
