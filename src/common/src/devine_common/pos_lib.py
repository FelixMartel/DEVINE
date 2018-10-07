#!/usr/bin/env python2
"""
Position library for DEVINE using openni

ROS Topics
"""
from __future__ import division #python 2 float division support
from threading import Lock
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import tf
from devine_config import topicname
import math_utils

#Topics
IMAGE_DEPTH_TOPIC = topicname('image_depth')
OBJECT_IMAGE_LOCATION_TOPIC = topicname('guess_location_image')
OBJECT_WORLD_LOCATION_TOPIC = topicname('guess_location_world')

ROS_PUBLISHER = rospy.Publisher(OBJECT_WORLD_LOCATION_TOPIC, Float32MultiArray, queue_size=10)


class PosLib(object):
    """
    Callback executed when a depth image is received from openni
    """
    def __init__(self, depth_topic, obj_pos_topic):
        self.trans_camera = 0
        self.rot_camera = 0
        self.tf_listener = tf.TransformListener()
        self.subscription = rospy.Subscriber(depth_topic,
                                             PointCloud2, self.openni_depth_callback)
        rospy.Subscriber(obj_pos_topic, Int32MultiArray, self.object_position_callback)
        self.position_to_transform = None #[320, 240] #in pixel
        self.current_point_cloud = None
        self.mutex = Lock() #Lock mutex to sync up point_cloud and 2D position calculation

    def openni_depth_callback(self, data):
        """
        Callback executed when a depth image is received from openni
        """
        #self.subscription.unregister() #only do it once
        rospy.loginfo("Received a new pointcloud, image size: %ix%i", data.width, data.height)
        self.mutex.acquire()
        self.current_point_cloud = data
        self.mutex.release()
        self.do_point_transform()
        #points = self.draw(self.deserialize(data))

    def object_position_callback(self, data):
        """
        Callback executed when an object is found by guesswhat and its position is broadcasted
        """
        rospy.loginfo("Received a new 2D point to transform")
        self.mutex.acquire()
        self.position_to_transform = data.data
        self.mutex.release()
        self.do_point_transform()

    def do_point_transform(self):
        '''Do the 2d -> 3d transformation if we got the necessary information'''
        self.mutex.acquire()
        try:
            if self.current_point_cloud is not None and self.position_to_transform is not None:
                [x, y] = self.position_to_transform
                print(x, y)
                pc = self.current_point_cloud
                center_point = math_utils.upper_left_to_zero_center(x, y, pc.width, pc.height)
                [z] = next(point_cloud2.read_points(pc, field_names='z',
                                                    skip_nans=False, uvs=[(x, y)]))
                (trans, rot) = self.get_trans()
                position = math_utils.calc_geometric_location(center_point[0], center_point[1],
                                                              z,
                                                              pc.width, pc.height,
                                                              trans, rot)
                ros_packet = Float32MultiArray()
                ros_packet.data = position
                ROS_PUBLISHER.publish(ros_packet)
                self.current_point_cloud = None
                self.position_to_transform = None
                rospy.loginfo("Object 3D position calculated: (%.2f, %.2f, %.2f)",
                              position[0], position[1], position[2])
        finally:
            self.mutex.release()

    def get_trans(self):
        '''Get translation between robot base_link and camera base_link'''
        target_frame = "/base_link"
        source_frame = "/openni_base_link"
        time = self.tf_listener.getLatestCommonTime(target_frame, source_frame)
        (trans, rot) = self.tf_listener.lookupTransform(target_frame, source_frame, time)
        return (trans, rot)

def deserialize(pcloud):
    """
    Deserialize a cloudpoit into an array of (x, y, z) tuples
    """
    return list(point_cloud2.read_points(pcloud, field_names=("x", "y", "z"), skip_nans=True))

def main():
    ''' Init node '''
    rospy.init_node('pos_lib')
    PosLib(IMAGE_DEPTH_TOPIC, OBJECT_IMAGE_LOCATION_TOPIC)
    rospy.loginfo("Node initialized")
    rospy.spin()

if __name__ == '__main__':
    main()
