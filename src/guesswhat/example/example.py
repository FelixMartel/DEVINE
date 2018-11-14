__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import rospy
from std_msgs.msg import String, Float64MultiArray
import h5py
import gzip
import json
from devine_config import topicname

SEGMENTATION_TOPIC = topicname('objects')
FEATURES_TOPIC = topicname('image_features')

rospy.init_node('guesswhat_example', anonymous=True)
f = gzip.open('guesswhat.image.jsonl.gz')
seg = json.loads(next(f).decode())
f.close()
h5file = h5py.File('image_features.h5', 'r')
feats = h5file['features'][0]
rospy.loginfo(seg)
rospy.loginfo(feats)

seg_pub = rospy.Publisher(SEGMENTATION_TOPIC, String, queue_size=1, latch=True)
seg_pub.publish(json.dumps(seg))

feats_pub = rospy.Publisher(FEATURES_TOPIC, Float64MultiArray, queue_size=1, latch=True)
feats_pub.publish(Float64MultiArray(data=feats))

rospy.spin()
