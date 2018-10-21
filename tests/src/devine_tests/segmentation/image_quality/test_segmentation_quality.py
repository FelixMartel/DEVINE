import json
import unittest
import rospy
from devine_tests import utils
import devine_tests.segmentation.segmentation_helper as helper
from std_msgs.msg import String

class TestSegmentationQuality(unittest.TestCase):
    """ Test to validate segmentation quality """

    def test_segmentation_quality(self):
        """ Tests segementation quality """
        # Start ROS node
        rospy.init_node("image_seg_test")
        test_images = helper.load_test_images(__file__, utils.get_fullpath(__file__, "test.json"))

        expected_objects = test_images[0][helper.EXPECTED_OBJECTS]

        images_objects_found = []
        for image in [test_images[1]]:
            self.assertFalse(rospy.is_shutdown())

            rospy.loginfo("Loading objects from images %d...", image[helper.FILENAME])
            images_objects_found.append(self.get_objects_found(image))

        pass

    def get_objects_found(self, test_image):
        """ Trigger the segmentation on a given image """
        # send over node
        helper.IMAGE_PUB.publish(test_image[helper.IMAGE_MSG])
        # receive data
        data = rospy.wait_for_message(helper.SEGMENTATION_IMAGE_TOPIC, String)
        rospy.logwarn("Message timestamp: %s", json.loads(data.data)['timestamp'])

        return helper.get_segmented_objets(data)
