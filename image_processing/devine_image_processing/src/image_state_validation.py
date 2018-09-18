#!/usr/bin/env python3
'''ROS module for image'''

from queue import Queue, Empty
from std_msgs.msg import Float64MultiArray
import rospy
import rosbag
from std_msgs.msg import String
from DEVINEParameters import ConfigSectionMap

#topics
IMAGE_TOPIC = ConfigSectionMap("TOPICS")['RawImage']
VALID_IMAGE_TOPIC = ConfigSectionMap("TOPICS")['ValidatedImage']

def validate_image():
    """
    Method that validates images received from the kinect before sending it to other modules
    """
    #TODO: Some processing to discriminate good and bad images
    rospy.Publisher(VALID_IMAGE_TOPIC, Float64MultiArray, queue_size=1)

if __name__ == '__main__':
    #rospy.init_node('image_state')
    validate_image()
    print(VALID_IMAGE_TOPIC)
    #rospy.spin()
