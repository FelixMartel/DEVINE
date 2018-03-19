#!/usr/bin/env python2
"""
Position library for DEVINE using openni

ROS Topics
"""
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

IMAGE_DEPTH_TOPIC = "/camera/depth_registered/points"

class PosLib(object):
    """
    Callback executed when a depth image is received from openni
    """
    def __init__(self, subscription_topic):
        self.subscription = rospy.Subscriber(subscription_topic, PointCloud2, self.openni_depth_callback)
        self.current_2d_position = [0, 0]

    def openni_depth_callback(self, data):
        """
        Callback executed when a depth image is received from openni
        """
        self.subscription.unregister() #only do it once, could use msg = rospy.wait_for_message("my_topic", MyType)
        points = self.deserialize(data)
        #self.draw(points)


    def deserialize(self, pcloud):
        return list(point_cloud2.read_points(pcloud, field_names = ("x", "y", "z"), skip_nans=True))

    def draw(self, points):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        vects = zip(*points)
        xdata = list(vects[0])
        ydata = list(vects[1])
        zdata = list(vects[2])
        ax.scatter(xdata, ydata, zdata, c=zdata, cmap='Greens')
        plt.show()

if __name__ == '__main__':
    rospy.init_node('devine_kinect') #This could probably be a simple ros library
    PosLib(IMAGE_DEPTH_TOPIC)
    rospy.loginfo("Node initiated")
    rospy.spin()