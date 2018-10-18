#! /usr/bin/env python2
''' Test to validate segmentation rates '''
from Queue import Queue, Empty
import argparse
import json
import os
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from devine_config import topicname

IMAGE_TOPIC = topicname('segmentation_image')
SEGMENTATION_IMAGE_TOPIC = topicname('objects')


def list_diff(x1, x2):
    """ Returns the difference between two lists"""
    i = len(x1)-1
    j = len(x2)-1
    x1.sort()
    x2.sort()
    while i >= 0 and j >= 0:
        if x1[i] > x2[j]:
            i -= 1
        elif x1[i] < x2[j]:
            j -= 1
        else:
            del x1[i]
            del x2[j]
            i -= 1
            j -= 1
    return [x1, x2]


def image_file_to_ros_msg(image_path):
    """ Convert an image file to a ros readable data message """
    img = cv2.imread(get_fullpath(image_path), cv2.IMREAD_COLOR)

    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "png"
    msg.data = np.array(cv2.imencode('.png', img)[1]).tostring()

    return msg


def load_test_data(test_filepath):
    """ Loads test data and images"""
    imgs = []
    data = []
    with open(test_filepath) as json_data:
        test_data = json.load(json_data)

    i = 0 # TODO: to be removed
    for test in test_data["tests"]:
        i += 1
        imgs.append(image_file_to_ros_msg(test["imageName"]))
        data.append(test["refData"]["objects"])
        if i == 1:
            break
    return imgs, data


def flatten_json(data):
    """ Takes input json object and returns a list"""
    return [object["category"] for object in data["objects"]]


def segmentation_call_back(data):
    """ Callback for segmentation topic """
    if seg_queue.full():
        seg_queue.get()
    try:
        seg_queue.put(flatten_json(json.loads(data.data)))
    except ValueError as exp:
        rospy.logerr(exp)


def run_test(image_msg, ref_data):
    """ Evaluates  segmentation rate for a single image """
    # send over node
    segmentation_pub = rospy.Publisher(IMAGE_TOPIC, CompressedImage, queue_size=1)
    rospy.Subscriber(SEGMENTATION_IMAGE_TOPIC, String, segmentation_call_back)

    # msg = CompressedImage()
    # msg.header.stamp = rospy.Time.now()
    # msg.format = "png"
    # msg.data = np.array(cv2.imencode('.png', image)[1]).tostring()
    rospy.sleep(1)
    segmentation_pub.publish(image_msg)
    rospy.sleep(1)
    # receive data
    try:
        results = seg_queue.get()
        [missed_detections, false_detections] = list_diff(ref_data, results)
        missed_detection_count = len(missed_detections)
        false_detection_count = len(false_detections)
    except Empty:
        missed_detection_count = -1
        false_detection_count = -1
    # extract and comp dat
    return [missed_detection_count, false_detection_count]


def get_fullpath(relative_file):
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), relative_file)


def main():
    '''Loads images and posts the corresponding segmentation rates'''
    # Start ROS node
    rospy.init_node('image_seg_test')
    global seg_queue
    seg_queue = Queue(1)
    [imgs, data] = load_test_data(get_fullpath("test.json"))

    pos = 0
    rate = rospy.Rate(1)
    while not rospy.is_shutdown() and pos < len(imgs):
        [missed_detection_count, false_detection_count] = run_test(
            imgs[pos], data[pos])
        pos += 1
        rospy.loginfo("Num missed detections: " + str(missed_detection_count))
        rospy.loginfo("Num false detections: " + str(false_detection_count))
        rate.sleep()
        print("Percentage of objects missed: " +
              str(float(missed_detection_count)/len(data[pos-1])))
        print("Number of objects falsely detected: ", false_detection_count)


if __name__ == '__main__':
    main()
