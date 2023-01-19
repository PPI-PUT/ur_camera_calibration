#!/usr/bin/env python3

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

import rclpy
from rclpy.node import Node
try:
    from ur_camera_calibration.ur_camera_calibration import RobotCameraCalibration
except ImportError:
    from ur_camera_calibration import RobotCameraCalibration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import os
import yaml
from threading import Thread


class RobotCameraCalibrationNode(Node):
    def __init__(self):
        super().__init__("robot_camera_calibration_node")
        print("INIT NODE")
        # load parameters
        robot_ip = self.declare_parameter('robot_ip', "150.254.47.176").value
        self.save_dir = self.declare_parameter('save_dir', ".").value
        prefix = self.declare_parameter('prefix', "").value
        self.prefix = prefix if prefix == "" or prefix.endswith("_") else prefix + "_"

        self.robot_camera_calibration = RobotCameraCalibration(robot_ip)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.read_apriltag_transform)
        print("NODE INITIALIZED")
        Thread(target=self.calibrate).start()

    def read_apriltag_transform(self):
        #print("READ TF:")
        try:
            # todo: check frame names
            tag_tf = self.tf_buffer.lookup_transform(
                "rgb_camera_link", "tag", rclpy.time.Time())
                #"tag", "rgb_camera_link", rclpy.time.Time())
            self.robot_camera_calibration.set_tag_tf(tag_tf)
        except TransformException as e:
            self.get_logger().error("Transform exception: {}".format(e))

    def calibrate(self):
        rot, trans = self.robot_camera_calibration.callibrate()
        rot = rot.tolist()
        trans = trans.tolist()
        transform = {
            "frame_id": self.prefix + "base_link",
            "child_frame_id": "rgb_camera_link",
            "transform": {
                "translation": {"x": trans[0], "y": trans[1], "z": trans[2]},
                "rotation": {"x": rot[0], "y": rot[1], "z": rot[2], "w": rot[3]},
            }
        }
        if self.save_dir:
            os.makedirs(self.save_dir, exist_ok=True)

        save_path = os.path.join(self.save_dir, "calibration.yaml")
        with open(save_path, 'w') as yaml_file:
            yaml.dump(transform, yaml_file, default_flow_style=False)
        print("Calibration finshed!")
        print("Calibration results saved to {}".format(save_path))
        exit()


def main(args=None):
    rclpy.init(args=args)
    node = RobotCameraCalibrationNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
