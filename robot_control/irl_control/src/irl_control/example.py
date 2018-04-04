#! /usr/bin/env python

import argparse
import rospy

from trajectory_client import TrajectoryClient
from gripper import Gripper
import ik
from marker import Markers
import tf

def main(args):
    """Example to control head, arms and gripper with command line
    """
    # Parse arguments
    robot = args.robot
    controller = args.controller
    arm = None
    if controller.split('_arm')[0]:
        arm = controller.split('_')[0]

    joints = args.joints
    if joints:
        joints = [float(i) for i in args.joints.replace('[', '').replace(']', '').split(',')]
    point = args.point
    if point:
        point = [float(i) for i in args.point.replace('[', '').replace(']', '').split(',')]
    time = float(args.time)

    # Start ROS node
    print('Init node...')
    node_name = 'irl_control' + '_' + controller
    rospy.init_node(node_name)
    print('Running node \'' + node_name + '\'...')

    # Init TrajectoryClient
    try:
        traj = TrajectoryClient(robot, controller)
        traj_head = TrajectoryClient(robot, 'head_controller')
    except RuntimeError as err:
        rospy.logerr(err)
        rospy.signal_shutdown(err)

    # Init Gripper
    gripper = Gripper(robot, arm)

    # Calculate position
    if point and arm:
        # Get tf information
        tf_listener = tf.TransformListener()

        trans_arm = None
        i = 0
        while not trans_arm:
            try:
                (trans_arm, rot) = tf_listener.lookupTransform('/' + arm[0].upper() + '_shoulder_fixed_link', '/obj', rospy.Time(0))
                (trans_neck, rot_neck) = tf_listener.lookupTransform('/neck_pan_link', '/obj', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
                rospy.sleep(0.1)
                i = i + 1
                if i == 10:
                    rospy.logerr(err)
                    rospy.signal_shutdown(err)
                else:
                    continue
        print('Translation from shoulder_fixed_link to obj:', trans_arm)
        print('Translation from neck_pan_link to obj:', trans_neck)

        # Calculate inverse kinematic
        joints_position = ik.arm_pan_tilt(arm, trans_arm[0], trans_arm[1], trans_arm[2])
        print('Arm Joint Position:', joints_position)

        neck_joints_position = ik.neck_pan_tilt(trans_neck[0], trans_neck[1], trans_neck[2])
        print('Neck Joint Position:', neck_joints_position)

        # Show markers TODO TEMP
        markers = Markers([[point[0], point[1], point[2]]])
        for i in range(0, 100):
            markers.publish()
            rospy.sleep(0.01)
    else:
        joints_position = joints

    # Accomplish trajectory
    traj.add_point(joints_position, time)
    traj_head.add_point(neck_joints_position, time)

    traj.start()
    traj_head.start()

    traj.wait(time)
    traj_head.wait(time)

    for i in range(3):
        gripper.open(0.3)
        rospy.sleep(0.5)
        gripper.open(0.1)
        rospy.sleep(0.5)

    traj.clear()

    traj.add_point([0, 0, 0, 0], time)
    traj_head.add_point([0, 0], time)

    traj.start()
    traj_head.start()

    traj.wait(time)
    traj_head.wait(time)

    print('Completed')

if __name__ == '__main__':
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument('-r', '--robot', required=True, choices=['jn0'], help='Which robot?')
    required.add_argument('-c', '--controller', required=True, choices=['head_controller', 'left_arm_controller', 'right_arm_controller'], help='Which controller?')
    required.add_argument('-j', '--joints', required=False, help='What joints positions?')
    required.add_argument('-p', '--point', required=False, help='What 3D point to point?')
    required.add_argument('-t', '--time', required=True, help='How much seconds?')
    args = parser.parse_args(rospy.myargv()[1:])

    main(args)
