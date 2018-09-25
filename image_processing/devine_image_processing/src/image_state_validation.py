#!/usr/bin/env python3
'''ROS module for image'''

from Queue import Queue, Empty
import rospy
import rosbag
from std_msgs.msg import Bool, Float64MultiArray
from sensor_msgs.msg import CompressedImage
from DEVINEParameters import ConfigSectionMap, test

#topics
IMAGE_TOPIC = ConfigSectionMap("TOPICS")['RawImage']
VALID_IMAGE_TOPIC = ConfigSectionMap("TOPICS")['ValidatedImage']
BLUR_DETECTION_TOPIC = ConfigSectionMap("TOPICS")['BlurDetection']

#ROS PUBLISHER
ROS_PUBLISHER = rospy.Publisher(VALID_IMAGE_TOPIC, Float64MultiArray, queue_size=1)

def validate_image(data):
    if (data.data):
        valid_image = raw_image_queue.get(timeout=1)
        ROS_PUBLISHER.publish(valid_image)

def raw_image_callback(data):
        '''Callback for image topic'''
        if raw_image_queue.full():
            rospy.logwarn('Queue full, flushing 1 image')
            raw_image_queue.get()
        raw_image_queue.put(data.data)

if __name__ == '__main__':
    rospy.init_node('image_state')
    raw_image_queue = Queue(5)

    rospy.Subscriber(IMAGE_TOPIC, CompressedImage, raw_image_callback)
    rospy.Subscriber(BLUR_DETECTION_TOPIC, Bool, validate_image)
    rospy.spin()
