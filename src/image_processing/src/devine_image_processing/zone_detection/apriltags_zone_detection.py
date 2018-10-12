import rospy
from ros_image_processor import ImageProcessor
from bson import json_util

import tf
from apriltags2_ros.msg import AprilTagDetectionArray

CAMERA_TOPIC = "/camera"
TOP_LEFT_TOPIC = "/top-left"
BOTTOM_RIGHT_TOPIC = "/bottom-right"

class AprilTagsZoneDetection(ImageProcessor):

    def __init__(self):
        self.tf = tf.TransformListener()

    def process(self, *_):
        '''Process a new image by detecting possible zones'''
        # tags = rospy.wait_for_message("/tag_detections", AprilTagDetectionArray, timeout=0.5)
        # for tag in tags.detections:
        #     position = tag.pose.pose.pose.position
        #     rospy.logwarn(position)

        top_left = [-1, -1, -1]
        bottom_right = [-1, -1, -1]

        try:
            time = self.tf.getLatestCommonTime(CAMERA_TOPIC, TOP_LEFT_TOPIC)
            position, quaternion = self.tf.lookupTransform(CAMERA_TOPIC, TOP_LEFT_TOPIC, time)
            top_left = position
        except tf.Exception as err:
            rospy.logerr(err)
            rospy.signal_shutdown(err)
        
        try:
            time = self.tf.getLatestCommonTime(CAMERA_TOPIC, BOTTOM_RIGHT_TOPIC)
            position, quaternion = self.tf.lookupTransform(CAMERA_TOPIC, BOTTOM_RIGHT_TOPIC, time)
            bottom_right = position
        except tf.Exception as err:
            rospy.logerr(err)
            rospy.signal_shutdown(err)

        result = {
            "top_left_corner": top_left,
            "bottom_right_corner": bottom_right 
        }

        return json_util.dumps(result)
