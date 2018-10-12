#!/usr/bin/env python2
'''
    Zone detection node
    Created by DEVINE
'''
import rospy
from std_msgs.msg import String
from ros_image_processor import ROSImageProcessingWrapper
from zone_detection.color_zone_detection import ColorZoneDetection
from zone_detection.apriltags_zone_detection import AprilTagsZoneDetection
from devine_config import topicname

IMAGE_TOPIC = topicname('zone_detection_image')
ZONE_DETECTION_TOPIC = topicname('zone_detection')

def main():
    '''Entry point of this file'''
    mode = rospy.get_param("/zone_detection/mode")
    processor = None
    if mode == "apriltags":
        processor = ROSImageProcessingWrapper(AprilTagsZoneDetection, IMAGE_TOPIC)
    else: #mode == "colortags" is default
        processor = ROSImageProcessingWrapper(ColorZoneDetection, IMAGE_TOPIC)
    publisher = rospy.Publisher(ZONE_DETECTION_TOPIC, String, queue_size=10, latch=True)
    processor.loop(lambda processor_output : publisher.publish(processor_output))

if __name__ == '__main__':
    main()
