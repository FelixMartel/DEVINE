#! /usr/bin/env python2

''' Script to point with command line '''

import argparse
import rospy
import tf

from trajectory_msgs.msg import JointTrajectoryPoint
from devine_config import topicname
from geometry_msgs.msg import PoseStamped


NODE_NAME = 'devine_irl_control_example_point'
TOPIC_OBJECT_LOCATION = topicname('guess_location_world')
TOPIC_HEAD_LOOK_AT = topicname('robot_look_at')
TOPIC_HEAD_JOINT_STATE = topicname('robot_head_joint_traj_point')
TIME = 2

def main(args):
    ''' Publish on the 3D position from /base_link to point '''
    # Parse arguments
    if args.point:
        point = [float(i) for i in args.point.split(',')]
    if args.look:
        look = [float(i) for i in args.look.split(',')]
    elif args.head_joint_position:
        head_joint_pos = [float(i) for i in args.head_joint_position.split(',')]
    if args.time:
        time = int(args.time)
    else:
        time = TIME

    rospy.init_node(NODE_NAME)
    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        if args.point:
            pub_arm = rospy.Publisher(TOPIC_OBJECT_LOCATION, PoseStamped, queue_size=1)
            pose_stamp_arm = pose_to_pose_stamped(point[0], point[1], point[2])
            pub_arm.publish(pose_stamp_arm)
        if args.look:
            pub_head = rospy.Publisher(TOPIC_HEAD_LOOK_AT, PoseStamped, queue_size=1)
            pose_stamp_head = pose_to_pose_stamped(look[0], look[1], look[2])
            pub_head.publish(pose_stamp_head)
        elif args.head_joint_position:
            pub = rospy.Publisher(TOPIC_HEAD_JOINT_STATE, JointTrajectoryPoint, queue_size=1)
            ros_packet = JointTrajectoryPoint()
            ros_packet.positions = head_joint_pos
            ros_packet.time_from_start = rospy.Duration(time)
            pub.publish(ros_packet)
        else:
            rospy.signal_shutdown('Missing arguments')
        rate.sleep()

def pose_to_pose_stamped(x, y, z, roll=0, pitch=0, yaw=0):
    ''' Convert x, y, z, roll, pitch, yaw to PoseStamp '''
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'base_link'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    return pose

def parser():
    ''' Command Line Parser'''
    arg_fmt = argparse.RawDescriptionHelpFormatter
    arg_parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    required = arg_parser.add_argument_group('required arguments')
    required.add_argument('-p', '--point', required=False, help='What 3D position to point?')
    required.add_argument('-l', '--look', required=False, help='What 3D position to look?')
    required.add_argument('-hjp', '--head_joint_position',
                          required=False, help='What joint positions?')
    required.add_argument('-t', '--time', required=False, help='Time to accomplish trajectory?')
    arguments = arg_parser.parse_args(rospy.myargv()[1:])
    return arguments

if __name__ == '__main__':
    main(parser())
