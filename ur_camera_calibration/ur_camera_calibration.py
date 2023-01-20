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

from time import sleep
from copy import copy
import numpy as np
import cv2
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from scipy.spatial.transform import Rotation as R

def rotmat2rotvec(rm):
    return R.from_matrix(rm).as_rotvec(degrees=False)

def rotvec2rotmat(rotvec):
    return R.from_rotvec(rotvec).as_matrix()

def rotvec2euler(rotvec):
    return R.from_rotvec(rotvec).as_euler("xyz", degrees=False)

def euler2rotmat(euler):
    return R.from_euler("xyz", euler).as_matrix()

def quat2rotmat(quat):
    return R.from_quat(quat).as_matrix()

def rotmat2quat(rm):
    return R.from_matrix(rm).as_quat()

def modify_pose(pose, move):
    pose[:3] += move[:3]
    R = rotvec2rotmat(pose[3:])
    Re = euler2rotmat(move[3:])
    pose[3:] = rotmat2rotvec(Re @ R)
    return pose

class URCameraCalibration:
    def __init__(self, robot_ip) -> None:
        self.tag_rot = np.eye(3)
        self.tag_trans = np.array([0., 0., 0.])
        print("INIT ROBOT")
        print("ROBOT IP: ", robot_ip)
        self.robot_control = RTDEControlInterface(robot_ip)
        self.robot_receive = RTDEReceiveInterface(robot_ip)
        print("ROBOT INITIALIZED")

        self.tcp_t = []
        self.tcp_R = []
        self.marker_t = []
        self.marker_R = []

        self.tag_t = None
        self.tag_R = None

        self.image = None

        d = 0.1
        rd = 0.25
        self.moves = np.array([
            [0., 0., 0., 0., 0., 0.],
            [0., 0., d,  0., 0., 0.],
            [0., 0., -d, 0., 0., 0.],
            [0., d,  0., 0., 0., 0.],
            [0., -d, 0., 0., 0., 0.],
            [d,  0., 0., 0., 0., 0.],
            [-d, 0., 0., 0., 0., 0.],
            [0., d,   d, 0., 0., 0.],
            [0., d,  -d, 0., 0., 0.],
            [0., -d,  d, 0., 0., 0.],
            [0., -d, -d, 0., 0., 0.],
            [d,   d, 0., 0., 0., 0.],
            [d,  -d, 0., 0., 0., 0.],
            [-d,  d, 0., 0., 0., 0.],
            [-d, -d, 0., 0., 0., 0.],
            [d,  0.,  d, 0., 0., 0.],
            [d,  0., -d, 0., 0., 0.],
            [-d, 0.,  d, 0., 0., 0.],
            [-d, 0., -d, 0., 0., 0.],
            [0., 0., 0., 0.,   0.,  rd,],
            [0., 0., 0., 0.,   0., -rd,],
            [0., 0., 0., 0.,   rd,  0.,],
            [0., 0., 0., 0.,  -rd,  0.,],
            [0., 0., 0., rd,   0.,  0.,],
            [0., 0., 0., -rd,  0.,  0.,],
            [0., 0., 0., rd,   0.,  rd,],
            [0., 0., 0., rd,   0., -rd,],
            [0., 0., 0., -rd,  0.,  rd,],
            [0., 0., 0., -rd,  0., -rd,],
            [0., 0., 0., rd,   rd,  0.,],
            [0., 0., 0., rd,  -rd,  0.,],
            [0., 0., 0., -rd,  rd,  0.,],
            [0., 0., 0., -rd, -rd,  0.,],
            [0., 0., 0., rd,   0.,  rd,],
            [0., 0., 0., rd,   0., -rd,],
            [0., 0., 0., -rd,  0.,  rd,],
            [0., 0., 0., -rd,  0., -rd,],
            [0., 0., 0., 0., 0., 0.],
        ])

    def set_tag_tf(self, tag_tf):
        trans = tag_tf.transform.translation
        rot = tag_tf.transform.rotation
        self.tag_t = np.array([trans.x, trans.y, trans.z])
        self.tag_R = quat2rotmat([rot.x, rot.y, rot.z, rot.w])

    def set_camera_image(self, image):
        self.image = image
        

    def callibrate(self):
        sleep(5.)
        print("Starting calibration")
        # todo: 
        robot_pose = self.robot_receive.getActualTCPPose()
        for m in self.moves:
            desired_pose = modify_pose(copy(robot_pose), m)
            self.robot_control.moveJ_IK(desired_pose)
            print(desired_pose)
            sleep(2.)
            actual_pose = self.robot_receive.getActualTCPPose()
            self.tcp_t.append(actual_pose[:3])
            self.tcp_R.append(rotvec2rotmat(actual_pose[3:]))
            self.marker_t.append(self.tag_t)
            self.marker_R.append(self.tag_R)
            sleep(1.)
        
        self.tcp_t, self.tcp_R = np.array(self.tcp_t), np.array(self.tcp_R)
        self.marker_t, self.marker_R = np.array(self.marker_t), np.array(self.marker_R)
        calibration_data = dict(tcp_t=self.tcp_t, tcp_R=self.tcp_R,
                                marker_t=self.marker_t, marker_R=self.marker_R)
        np.save("calibration_data_12.npy", calibration_data)
        print(self.tcp_t)
        print(self.marker_t)
        invtcp_R = np.linalg.inv(self.tcp_R)
        invtcp_t = (-invtcp_R @ self.tcp_t[..., np.newaxis])[..., 0]
        R, t = cv2.calibrateHandEye(invtcp_R, invtcp_t, self.marker_R, self.marker_t)
        R = np.array([[-1., 0., 0.], [0., -1., 0.], [0., 0., 1.]]) @ R
        R = rotmat2quat(R)
        print(R, t)
        return R, t[:, 0]
