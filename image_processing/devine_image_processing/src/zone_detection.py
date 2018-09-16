#!/usr/bin/env python2
'''Zone detection using bright colored squares'''

import rospy
from std_msgs.msg import String

from ros_image_processor import ImageProcessor, ROSImageProcessingWrapper

import cv2
import numpy as np
from enum import Enum

#import time

IMAGE_TOPIC = '/camera/rgb/image_color/compressed' #'/devine/image/zone_detection'
ZONE_DETECTION_TOPIC = '/zone_detection'

class ZoneDetection(ImageProcessor):
    '''Zone detection'''
    
    def process(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        #plt.imshow(image)
        #time.sleep(2)
        cv2.imshow("Original image", image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit()
        return "Hello, World !"


def main():
    '''Entry point of this file'''
    processor = ROSImageProcessingWrapper(ZoneDetection, IMAGE_TOPIC)
    publisher = rospy.Publisher(ZONE_DETECTION_TOPIC, String, queue_size=10, latch=True)
    processor.loop(lambda processor_output : publisher.publish(processor_output))

if __name__ == '__main__':
    main()
