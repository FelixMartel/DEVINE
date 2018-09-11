#!/usr/bin/env python

import math
import rospy
import tf
from std_msgs.msg import Float32MultiArray
from trajectory_client import TrajectoryClient
from gripper import Gripper
import ik
from movement import Movement

ROBOT = 'jn0'
TOPIC_OBJECT_LOCATION = "/object_location"
TOPIC_OBJECT_FRAME = "/object_frame"
TOPIC_ROBOT_BASE = "/base_link"
TOPIC_ROBOT_R_SHOULDER_FIXED_FRAME = "/R_shoulder_fixed_link"
TOPIC_ROBOT_L_SHOULDER_FIXED_FRAME = "/L_shoulder_fixed_link"
TOPIC_ROBOT_NECK_PAN_FRAME = "/neck_pan_link"
TOPIC_GUESSWHAT_SUCCEED = '/is_guesswhat_succeed'

class Controller(object):
    def __init__(self):
        self.object_location = None
        self.tf_listener = tf.TransformListener()
        self.gripper = Gripper(ROBOT, 'right')

        try:
            rospy.loginfo('Waiting for Arm and Head controllers')
            self.traj_arm_right = TrajectoryClient(ROBOT, 'right_arm_controller')
            self.traj_arm_left = TrajectoryClient(ROBOT, 'left_arm_controller')
            self.traj_head = TrajectoryClient(ROBOT, 'head_controller')
        except RuntimeError as err:
            rospy.logerr(err)
            rospy.signal_shutdown(err)

        try:
            rospy.loginfo('Waiting for /object_frame and /base_link...')
            self.tf_listener.waitForTransform(TOPIC_ROBOT_R_SHOULDER_FIXED_FRAME, TOPIC_OBJECT_FRAME, rospy.Time(), rospy.Duration(4))
            self.tf_listener.waitForTransform(TOPIC_ROBOT_R_SHOULDER_FIXED_FRAME, TOPIC_ROBOT_BASE, rospy.Time(), rospy.Duration(4))
            self.tf_listener.waitForTransform(TOPIC_ROBOT_L_SHOULDER_FIXED_FRAME, TOPIC_OBJECT_FRAME, rospy.Time(), rospy.Duration(4))
            self.tf_listener.waitForTransform(TOPIC_ROBOT_L_SHOULDER_FIXED_FRAME, TOPIC_ROBOT_BASE, rospy.Time(), rospy.Duration(4))
        except tf.Exception as err:
            rospy.logerr(err)
            # rospy.signal_shutdown(err)

        rospy.Subscriber(TOPIC_OBJECT_LOCATION, Float32MultiArray, self.object_location_callback)

    def object_location_callback(self, msg):
        if self.object_location != msg.data:
            rospy.loginfo(msg.data)
            self.object_location = msg.data
            self.now = rospy.Time().now()
            if msg.data != (0, 0, 0):
                self.calcul()
                self.move()
            else:
                self.move_init(10)

    def calcul(self):
        trans_r_arm = None
        trans_l_arm = None
        i = 0
        try:
            self.tf_listener.waitForTransform(TOPIC_ROBOT_R_SHOULDER_FIXED_FRAME, TOPIC_OBJECT_FRAME, self.now, rospy.Duration(4))
            (trans_r_arm, rot_arm) = self.tf_listener.lookupTransform(TOPIC_ROBOT_R_SHOULDER_FIXED_FRAME, TOPIC_OBJECT_FRAME, self.now)
            
            self.tf_listener.waitForTransform(TOPIC_ROBOT_L_SHOULDER_FIXED_FRAME, TOPIC_OBJECT_FRAME, self.now, rospy.Duration(4))
            (trans_l_arm, rot_l_arm) = self.tf_listener.lookupTransform(TOPIC_ROBOT_L_SHOULDER_FIXED_FRAME, TOPIC_OBJECT_FRAME, self.now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
            rospy.logerr(err)
            rospy.signal_shutdown(err)
       
        if not rospy.is_shutdown():
            trans_head = None
            i = 0            
            try:
                self.tf_listener.waitForTransform(TOPIC_ROBOT_NECK_PAN_FRAME, TOPIC_OBJECT_FRAME, self.now, rospy.Duration(4))
                (trans_head, rot_head) = self.tf_listener.lookupTransform(TOPIC_ROBOT_NECK_PAN_FRAME, TOPIC_OBJECT_FRAME, self.now)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
                rospy.logerr(err)
                rospy.signal_shutdown(err)

        if not rospy.is_shutdown():
            rospy.loginfo('Translation from r_shoulder_fixed_link to obj: %s', trans_r_arm)
            rospy.loginfo('Translation from l_shoulder_fixed_link to obj: %s', trans_l_arm)
            rospy.loginfo('Translation from neck_pan_link to obj: %s', trans_head)

            # Calculate inverse kinematic
            self.right_joints_position = ik.arm_pan_tilt('right', trans_r_arm[0], trans_r_arm[1], trans_r_arm[2])
            rospy.loginfo('Right Joint Position: %s', self.right_joints_position)

            self.left_joints_position = ik.arm_pan_tilt('left', trans_l_arm[0], trans_l_arm[1], trans_l_arm[2])
            rospy.loginfo('Left Arm Joint Position: %s', self.left_joints_position)

            self.head_joints_position = ik.head_pan_tilt(trans_head[0], trans_head[1], trans_head[2])
            rospy.loginfo('Head Joint Position: %s', self.head_joints_position)
          
    def move_init(self, time):
        rospy.loginfo('move_init')
        self.traj_arm_right.clear()
        self.traj_arm_left.clear()
        self.traj_head.clear()

        self.traj_arm_right.add_point([0, 0, 0, 0], time)
        self.traj_arm_left.add_point([0, 0, 0, 0], time)
        self.traj_head.add_point([0, 0], time)

        self.traj_arm_right.start()
        self.traj_arm_left.start()
        self.traj_head.start()

        self.traj_arm_right.wait(time)
        self.traj_arm_left.wait(time)
        self.traj_head.wait(time)

        rospy.loginfo('Completed')

    def move(self):
        time = 10
        
        point_with_head = True
        point_with_right = False
        point_with_left = False
        move_gripper = False

        if point_with_head:
            self.traj_head.clear()
            self.traj_head.add_point(self.head_joints_position, time)
            self.traj_head.start()
        if point_with_right:
            self.traj_arm_right.clear()
            self.traj_arm_right.add_point(self.right_joints_position, time)
            self.traj_arm_right.start()
        if point_with_left:
            self.traj_arm_left.clear()
            self.traj_arm_left.add_point(self.left_joints_position, time)
            self.traj_arm_left.start()

        if point_with_head:
            self.traj_head.wait(time)
        if point_with_right:
            self.traj_arm_right.wait(time)
        if point_with_left:
            self.traj_arm_left.wait(time)

        if move_gripper:
            i = 0
            while i < 3:
                self.gripper.open(0.3)
                rospy.sleep(0.5)
                self.gripper.open(0.1)
                rospy.sleep(0.5)
                i = i + 1

        rospy.loginfo('Completed')

def main():
    node_name = 'irl_control_control'
    rospy.init_node(node_name)
    rospy.loginfo('Running node \'' + node_name + '\'')

    controller = Controller()
    movement = Movement(controller)
    rospy.spin()

if __name__ == '__main__':
    main()
