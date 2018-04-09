#!/usr/bin/env python3

import rospy
from std_msgs import String, Int32MultiArray

from guesswhat.models.guesser.guesser_network import GuesserNetwork
from guesswhat.models.qgen.qgen_lstm_network import QGenNetworkLSTM
from guesswhat.models.qgen.qgen_wrapper import QGenWrapper
from guesswhat.models.looper.basic_looper import BasicLooper
from modelwrappers import GuesserROSWrapper, OracleROSWrapper

from guesswhat.data_provider.guesswhat_dataset import Game
from guesswhat.data_provider.guesswhat_tokenizer import GWTokenizer
from generic.utils.config import load_config, get_config_from_xp

from queue import Queue
import json

SEGMENTATION_TOPIC = '/rcnn_segmentation'
FEATURES_TOPIC = '/VGG16_features'
OBJECT_TOPIC = '/object_found'

segmentations = Queue(1)
features = Queue(1)

class ImgFeaturesLoader():
    def __init__(self, data):
        self.data = data

    def get_image(self, **kwargs):
        return self.data

class ImgFeaturesBuilder():
    def __init__(self, data):
        self.data = data

    def build(self, **kwargs):
        return ImgFeaturesLoader(self.data)

def segmentation_callback(data):
    segmentations.put(json.loads(data.data))

def feature_callback(data):
    features.put(data.data)

if __name__ == '__main__':
    rospy.init_node('guesswhat')
    rospy.Subscriber(SEGMENTATION_TOPIC, String, segmentation_callback)
    rospy.Subscriber(FEATURES_TOPIC, String, feature_callback)
    object_found = rospy.Publisher(OBJECT_TOPIC, Int32MultiArray)

    eval_config, exp_identifier, save_path = load_config(CONFIG_PATH, EXP_DIR)
    tokenizer = GWTokenizer(DICT_PATH)
    with tf.Session() as sess:
        guesser_config = get_config_from_xp(GUESSER_DIR, guesser_id)
        guesser_network = GuesserNetwork(guesser_config['model'], num_words=tokenizer.no_words)
        guesser_var = [v for v in tf.global_variables() if 'guesser' in v.name]
        guesser_saver = tf.train.Saver(var_list=guesser_var)
        guesser_saver.restore(sess, GUESSER_NTW_PATH)
        guesser_wrapper = GuesserROSWrapper(guesser_network)

        qgen_config = get_config_from_xp(QGEN_DIR, qgen_id)
        qgen_network = QGenNetworkLSTM(qgen_config['model'], num_words=tokenizer.no_words, policy_gradient=False)
        qgen_var = [v for v in tf.global_variables() if 'qgen' in v.name]
        qgen_saver = tf.train.Saver(var_list=qgen_var)
        qgen_saver.restore(sess, QGEN_NTW_PATH)
        qgen_network.build_sampling_graph(qgen_config['model'], tokenizer=tokenizer, max_length=eval_config['loop']['max_depth'])
        qgen_wrapper = QGenWrapper(qgen_network, tokenizer,
                                   max_length=eval_config['loop']['max_depth'],
                                   k_best=eval_config['loop']['beam_k_best'])

        oracle_wrapper = OracleROSWrapper(tokenizer)

        while not rospy.is_shutdown():
            seg = segmentations.get(timeout=1)
            if seg is None:
                continue
            feat = features.get(timeout=1)
            if feat is None:
                continue

            img = {'id': 0, 'width': 0, 'height': 0, 'coco_url': ''}
            game = Game(id=0,
                        object_id=0,
                        objects=seg['objects'],
                        qas=[],
                        image=img,
                        status="false",
                        which_set=None,
                        image_builder=ImgFeaturesBuilder(feat),
                        crop_builder=None)

            looper = BasicLooper(eval_config,
                                 guesser_wrapper=guesser_wrapper,
                                 qgen_wrapper=qgen_wrapper,
                                 oracle_wrapper=oracle_wrapper,
                                 batch_size=1)

            looper.process(sess, [game], mode='greedy', store_games=True)
            storage = looper.storage[0]
            choice_index = storage['guess_object_index']
            choice_bbox = game.objects[choice_index].bbox
            object_found.publish(Int32MultiArray(data=[bbox.x_center, bbox.y_center]))
