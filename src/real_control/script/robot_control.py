#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/6/7
# @Author : Zzy

from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
import rospy
import math
import numpy as np
import rtde_receive
import rtde_control



frequency = 125
PI = math.pi


class robot_client:
    def __init__(self):
        self.arm_client = SimpleActionClient("/scaled_pos_joint_traj_controller/follow_joint_trajectory",
                                             FollowJointTrajectoryAction)
        self.arm_cmd = FollowJointTrajectoryGoal()
        self.arm_state = self.arm_client.wait_for_server(rospy.Duration(3))
        if self.arm_state:
            rospy.loginfo("connected the arm server.")
        else:
            rospy.loginfo("connect the arm server failed.")
        self.arm_cmd.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                                               'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.way_point = JointTrajectoryPoint()

    def move_to_joint(self, joint_target, time1=1.0, wait=True):
        """
        @param wait: 是否阻塞
        """
        self.arm_cmd.trajectory.header = Header()
        self.arm_cmd.trajectory.header.stamp = rospy.Time.now()
        self.arm_cmd.trajectory.points = []

        self.way_point.positions = joint_target
        self.way_point.time_from_start = rospy.Duration.from_sec(time1)
        self.arm_cmd.trajectory.points.append(self.way_point)
        self.arm_client.send_goal(self.arm_cmd)
        if wait:
            duration = rospy.Duration.from_sec(time1)
            rospy.sleep(duration)

    def move_to_trajectory(self, trajectory, time=1.0, wait=True):
        self.arm_cmd.trajectory.header = Header()
        self.arm_cmd.trajectory.header.stamp = rospy.Time.now()
        self.arm_cmd.trajectory.points = []

        time_step = time / len(trajectory)
        for i in range(trajectory):
            self.way_point.positions = trajectory[i]
            self.way_point.time_from_start = rospy.Duration.from_sec(time_step)
            self.arm_cmd.trajectory.points.append(self.way_point)
        self.arm_client.send_goal(self.arm_cmd)
        if wait:
            duration = rospy.Duration.from_sec(time)
            rospy.sleep(duration)


if __name__ == '__main__':
    rospy.init_node("control_node")
    robot = robot_client()
    file_name = "../data/202207182040_robot.csv"
    data = np.loadtxt(open(file_name, "rb"), delimiter=",", skiprows=1)
    init_angle = data[0, 8:14]

    robot.move_to_joint(init_angle, time1 = 10)
