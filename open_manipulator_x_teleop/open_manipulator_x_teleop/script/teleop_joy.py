#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Will Son

from asyncio import Future
from math import fabs
import rclpy

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from sensor_msgs.msg import JointState
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Joy


class TeleopJoy(Node):
    def __init__(self):
        qos = QoSProfile(depth=10)

        self.present_joint_angle = [0.0, 0.0, 0.0, 0.0]
        self.goal_joint_angle = [0.0, 0.0, 0.0, 0.0]
        self.prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0]
        self.present_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.prev_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.TASK_POSITION_DELTA = 0.01  # meter
        self.JOINT_ANGLE_DELTA = 0.05  # radian
        self.PATH_TIME = 0.5  # second

        self.AXIS_THRESHOLD = 0.95

        super().__init__("teleop_joy")

        self.joint_state_subscription = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, qos
        )
        self.kinematics_pose_subscription = self.create_subscription(
            KinematicsPose, "kinematics_pose", self.kinematics_pose_callback, qos
        )
        self.open_manipulator_state_subscription = self.create_subscription(
            OpenManipulatorState,
            "states",
            self.open_manipulator_state_callback,
            qos,
        )
        self.joy_subscription = self.create_subscription(
            Joy, "joy", self.open_manipulator_joy_callback, qos
        )

        # Create Service Clients
        self.goal_joint_space = self.create_client(
            SetJointPosition, "goal_joint_space_path"
        )
        self.goal_tool_control = self.create_client(
            SetJointPosition, "goal_tool_control"
        )
        self.goal_task_space = self.create_client(
            SetKinematicsPose, "goal_task_space_path"
        )

        self.send_goal_task = Future()
        self.send_goal_task.set_result(True)

        self.last_action_time = self.get_clock().now()

    def open_manipulator_joy_callback(self, msg: Joy):
        # 5 - dead man switch
        if not msg.buttons[5]:
            return

        # check if execution finished
        if self.get_clock().now() - self.last_action_time < rclpy.time.Duration(
            seconds=(self.PATH_TIME * 0.4)
        ):
            return

        # Task space
        if self.send_goal_task.done():
            goal_task_space_changed = True
            if msg.axes[1] > self.AXIS_THRESHOLD:
                self.goal_kinematics_pose[0] = (
                    self.prev_goal_kinematics_pose[0] + self.TASK_POSITION_DELTA
                )
            elif msg.axes[1] < -self.AXIS_THRESHOLD:
                self.goal_kinematics_pose[0] = (
                    self.prev_goal_kinematics_pose[0] - self.TASK_POSITION_DELTA
                )
            elif msg.axes[0] > self.AXIS_THRESHOLD:
                self.goal_kinematics_pose[1] = (
                    self.prev_goal_kinematics_pose[1] + self.TASK_POSITION_DELTA
                )
            elif msg.axes[0] < -self.AXIS_THRESHOLD:
                self.goal_kinematics_pose[1] = (
                    self.prev_goal_kinematics_pose[1] - self.TASK_POSITION_DELTA
                )
            elif msg.axes[3] > self.AXIS_THRESHOLD:
                self.goal_kinematics_pose[2] = (
                    self.prev_goal_kinematics_pose[2] + self.TASK_POSITION_DELTA
                )
            elif msg.axes[3] < -self.AXIS_THRESHOLD:
                self.goal_kinematics_pose[2] = (
                    self.prev_goal_kinematics_pose[2] - self.TASK_POSITION_DELTA
                )
            else:
                goal_task_space_changed = False
            
            if goal_task_space_changed:
                self.last_action_time = self.get_clock().now()
                self.send_goal_task_space()
            
        # Joint space
        goal_joint_space_changed = True
        if msg.axes[4] > self.AXIS_THRESHOLD:
            self.goal_joint_angle[0] = (
                self.prev_goal_joint_angle[0] + self.JOINT_ANGLE_DELTA
            )
        elif msg.axes[4] < -self.AXIS_THRESHOLD:
            self.goal_joint_angle[0] = (
                self.prev_goal_joint_angle[0] - self.JOINT_ANGLE_DELTA
            )
        elif msg.axes[5] < -self.AXIS_THRESHOLD:
            self.goal_joint_angle[1] = (
                self.prev_goal_joint_angle[1] + self.JOINT_ANGLE_DELTA
            )
        elif msg.axes[5] > self.AXIS_THRESHOLD:
            self.goal_joint_angle[1] = (
                self.prev_goal_joint_angle[1] - self.JOINT_ANGLE_DELTA
            )
        elif msg.buttons[0]:
            self.goal_joint_angle[2] = (
                self.prev_goal_joint_angle[2] + self.JOINT_ANGLE_DELTA
            )
        elif msg.buttons[2]:
            self.goal_joint_angle[2] = (
                self.prev_goal_joint_angle[2] - self.JOINT_ANGLE_DELTA
            )
        elif msg.buttons[1]:
            self.goal_joint_angle[3] = (
                self.prev_goal_joint_angle[3] + self.JOINT_ANGLE_DELTA
            )
        elif msg.buttons[3]:
            self.goal_joint_angle[3] = (
                self.prev_goal_joint_angle[3] - self.JOINT_ANGLE_DELTA
            )
        else:
            goal_joint_space_changed = False
        
        if goal_joint_space_changed:
            self.last_action_time = self.get_clock().now()
            self.send_goal_joint_space()
        
        # Gripper
        if msg.buttons[6]:
            self.send_tool_control(-0.01)
            self.last_action_time = self.get_clock().now()
        elif msg.buttons[7]:
            self.send_tool_control(0.01)
            self.last_action_time = self.get_clock().now()

        # elif msg.buttons[9]:
        #     goal_joint_angle[0] = 0.0
        #     goal_joint_angle[1] = 0.0
        #     goal_joint_angle[2] = 0.0
        #     goal_joint_angle[3] = 0.0
        #     self.send_goal_joint_space()
        # elif msg.buttons[8]:
        #     goal_joint_angle[0] = 0.0
        #     goal_joint_angle[1] = -1.05
        #     goal_joint_angle[2] = 0.35
        #     goal_joint_angle[3] = 0.70
        #     self.send_goal_joint_space()

        for index in range(0, 7):
            self.prev_goal_kinematics_pose[index] = self.goal_kinematics_pose[index]
        for index in range(0, 4):
            self.prev_goal_joint_angle[index] = self.goal_joint_angle[index]

    def send_goal_task_space(self):
        goal_task_space_req = SetKinematicsPose.Request()
        goal_task_space_req.end_effector_name = "gripper"
        goal_task_space_req.kinematics_pose.pose.position.x = self.goal_kinematics_pose[
            0
        ]
        goal_task_space_req.kinematics_pose.pose.position.y = self.goal_kinematics_pose[
            1
        ]
        goal_task_space_req.kinematics_pose.pose.position.z = self.goal_kinematics_pose[
            2
        ]
        goal_task_space_req.kinematics_pose.pose.orientation.w = (
            self.goal_kinematics_pose[3]
        )
        goal_task_space_req.kinematics_pose.pose.orientation.x = (
            self.goal_kinematics_pose[4]
        )
        goal_task_space_req.kinematics_pose.pose.orientation.y = (
            self.goal_kinematics_pose[5]
        )
        goal_task_space_req.kinematics_pose.pose.orientation.z = (
            self.goal_kinematics_pose[6]
        )
        goal_task_space_req.path_time = self.PATH_TIME

        try:
            self.send_goal_task = self.goal_task_space.call_async(goal_task_space_req)
        except Exception as e:
            self.get_logger().info("Sending Goal Kinematic Pose failed %r" % (e,))

    def send_goal_joint_space(self):
        goal_joint_space_req = SetJointPosition.Request()
        goal_joint_space_req.joint_position.joint_name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
        ]
        goal_joint_space_req.joint_position.position = [
            self.goal_joint_angle[0],
            self.goal_joint_angle[1],
            self.goal_joint_angle[2],
            self.goal_joint_angle[3],
        ]
        goal_joint_space_req.path_time = self.PATH_TIME

        try:
            send_goal_joint = self.goal_joint_space.call_async(goal_joint_space_req)
        except Exception as e:
            self.get_logger().info("Sending Goal Joint failed %r" % (e,))

    def send_tool_control(self, position):
        tool_control_req = SetJointPosition.Request()
        tool_control_req.path_time = self.PATH_TIME
        tool_control_req.joint_position.joint_name = ["gripper"]
        tool_control_req.joint_position.position = [position]

        try:
            send_tool_control = self.goal_tool_control.call_async(tool_control_req)
        except Exception as e:
            self.get_logger().info("Sending tool control failed %r" % (e,))

    def kinematics_pose_callback(self, msg):
        self.present_kinematics_pose[0] = msg.pose.position.x
        self.present_kinematics_pose[1] = msg.pose.position.y
        self.present_kinematics_pose[2] = msg.pose.position.z
        self.present_kinematics_pose[3] = msg.pose.orientation.w
        self.present_kinematics_pose[4] = msg.pose.orientation.x
        self.present_kinematics_pose[5] = msg.pose.orientation.y
        self.present_kinematics_pose[6] = msg.pose.orientation.z

    def joint_state_callback(self, msg):
        self.present_joint_angle[0] = msg.position[0]
        self.present_joint_angle[1] = msg.position[1]
        self.present_joint_angle[2] = msg.position[2]
        self.present_joint_angle[3] = msg.position[3]

    def open_manipulator_state_callback(self, msg):
        if msg.open_manipulator_moving_state == "STOPPED":
            for index in range(0, 7):
                self.goal_kinematics_pose[index] = self.present_kinematics_pose[index]
            for index in range(0, 4):
                self.goal_joint_angle[index] = self.present_joint_angle[index]


def main():
    rclpy.init()
    teleop_joy = TeleopJoy()
    teleop_joy.get_logger().info("Starting")
    rclpy.spin(teleop_joy)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
