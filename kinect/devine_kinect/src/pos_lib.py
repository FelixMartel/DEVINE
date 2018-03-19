#!/usr/bin/env python2
"""
Position library for DEVINE using openni

ROS Topics
"""
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

IMAGE_DEPTH_TOPIC = "/camera/depth_registered/points"

class PosLib(object):
    """
    Callback executed when a depth image is received from openni
    """
    def __init__(self, subscription_topic):
        self.subscription = rospy.Subscriber(subscription_topic, 
                                            PointCloud2, self.openni_depth_callback)
        self.current_2d_position = [0, 0]

    def openni_depth_callback(self, data):
        """
        Callback executed when a depth image is received from openni
        """
        #self.subscription.unregister() #only do it once
        #points = self.deserialize(data)
        #self.draw(points)
        val = next(point_cloud2.read_points(data, field_names='z', skip_nans=False, uvs=[tuple(self.current_2d_position)]))
        rospy.loginfo("Object position found: (%i, %i, %i)",
        self.current_2d_position[0], self.current_2d_position[1], val[0])

        
    def deserialize(self, pcloud):
        """
        Deserialize a cloudpoit into an array of (x, y, z) tuples
        """
        return list(point_cloud2.read_points(pcloud, field_names = ("x", "y", "z"), skip_nans=True))


    def draw(self, points):
        """
        Draw an array of (x, y, z) tuples with matplotlib
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        vects = zip(*points)
        xdata = list(vects[0])
        ydata = list(vects[1])
        zdata = list(vects[2])
        ax.scatter(xdata, ydata, zdata, c=zdata, cmap='Greens')
        plt.show()


if __name__ == '__main__':
    rospy.init_node('devine_kinect')
    PosLib(IMAGE_DEPTH_TOPIC)
    rospy.loginfo("Node initiated")
    rospy.spin()