#!/usr/bin/env python2
"""
Node to find the scene
"""
from __future__ import division
import json

import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryActionFeedback
# http://docs.ros.org/fuerte/api/control_msgs/html/msg/

from devine_config import topicname
from devine_irl_control.irl_constant import ROBOT_CONTROLLER

NODE_NAME = 'SceneFinder'
ZONE_DETECTION_TOPIC = topicname('zone_detection')
NECK_TRAJ_STATE_TOPIC = '/jn0/neck_controller/follow_joint_trajectory/feedback'
NECK_CTRL_STATE_TOPIC = '/jn0/neck_controller/state'
TOPIC_HEAD_JOINT_STATE = '/head_joint_traj_point'

LIMITS = ROBOT_CONTROLLER['neck_controller']['joints_limit']

DELTA_TIME = 1
DELTA_POS = 0.05

class SceneFinder(object):
    ''' Find scene to play game '''

    def __init__(self, theta, time):
        self.current_joint_position = [0, 0]
        self.new_joint_position = [0, 0]
        self.current_error = [0, 0]
        self.theta = theta
        self.time = time
        self.direction = -1
        self.current_status = 3 # 3:SUCCEEDED, 0:PENDING
        self.current_zone = {
            "top_left_corner": [-1, -1],
            "bottom_right_corner": [-1, -1]
        }

        rospy.Subscriber(NECK_TRAJ_STATE_TOPIC, FollowJointTrajectoryActionFeedback,
                         self.traj_state_callback, queue_size=1)
        rospy.Subscriber(ZONE_DETECTION_TOPIC, String,
                         self.zone_callback, queue_size=1)
        self.sub_neck_ctrl = rospy.Subscriber(NECK_CTRL_STATE_TOPIC, JointTrajectoryControllerState,
                                              self.joint_state_callback, queue_size=1)
        self.pub_neck_ctrl = rospy.Publisher(TOPIC_HEAD_JOINT_STATE,
                                             JointTrajectoryPoint, queue_size=1)

    def traj_state_callback(self, data):
        ''' Monitor neck trajectory state '''
        self.current_status = data.status.status
        self.current_joint_position = list(data.feedback.actual.positions)

    def joint_state_callback(self, data):
        ''' Get initial neck joint state at node launch '''
        self.current_joint_position = data.actual.positions
        self.sub_neck_ctrl.unregister()

    def zone_callback(self, data):
        ''' Listen to zone_detection '''
        self.current_zone = json.loads(data.data)

    def update(self):
        ''' Move head to find scene '''
        delta_theta = self.theta * self.direction

        if self.new_joint_position[0] + delta_theta < LIMITS[0][0]:
            self.direction = 1
            delta_theta = self.theta * self.direction
        if self.new_joint_position[0] + delta_theta > LIMITS[0][1]:
            self.direction = -1
            delta_theta = self.theta * self.direction

        top_left = self.current_zone['top_left_corner']
        bottom_right = self.current_zone['bottom_right_corner']

        if top_left == [-1, -1] and bottom_right == [-1, -1]:
            self.new_joint_position[0] = self.current_joint_position[0] + delta_theta
        elif top_left == [-1, -1]:
            self.new_joint_position[0] = self.current_joint_position[0] + delta_theta
        elif bottom_right == [-1, -1]:
            self.new_joint_position[0] = self.current_joint_position[0] - delta_theta
        else:
            pass # publish found scene, ready to take picture!

        ros_packet = JointTrajectoryPoint(positions=self.new_joint_position,
                                          time_from_start=rospy.Duration(self.time))
        self.pub_neck_ctrl.publish(ros_packet)

def main():
    '''Entry point of this file'''

    rospy.init_node(NODE_NAME)
    scene_finder = SceneFinder(DELTA_POS, DELTA_TIME)
    rate = rospy.Rate(1/DELTA_TIME)
    while not rospy.is_shutdown():
        scene_finder.update()
        rate.sleep()

if __name__ == '__main__':
    main()
