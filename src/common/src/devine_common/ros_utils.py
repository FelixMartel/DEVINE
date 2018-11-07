""" ROS Utils """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import os
import sys

is_python_2 = sys.version_info[0] < 3

if is_python_2:
    import rospy
    import tf
    from geometry_msgs.msg import PoseStamped

    def pose_stamped(x, y, z, roll=0, pitch=0, yaw=0, ref_frame='base_link', stamp=None):
        """ Convert x, y, z, roll, pitch, yaw to PoseStamped """
        
        pose = PoseStamped()
        pose.header.stamp = stamp or rospy.Time.now() - rospy.rostime.Duration(0.1)
        pose.header.frame_id = ref_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        return pose


def get_fullpath(file, relative_file):
    """ Return the full path of a file in a directory relative to this one
    Inputs:
    - file: the __file__ variable of the python file to get the reference path from
    - relative_file: the file relative path to __file__
    """
    return os.path.join(os.path.dirname(os.path.abspath(file)), relative_file)
