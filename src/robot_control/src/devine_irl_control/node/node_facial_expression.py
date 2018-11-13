#!/usr/bin/env python2
"""
Simple ROS node that subscribes to objects_confidence and object_guess_success
and publishes to robot_facial_expression to show facial emotions
"""

from enum import Enum
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from jn0_face_msgs.msg import EmoIntensity
from jn0_face_msgs.msg import EmoPulse
from devine_dialog.msg import TtsQuery
from devine_config import topicname

class RobotFacialExpression(Enum):
    """ Valid facial expressions """

    SURPRISE = 'Surprise'
    SAD = 'Sad'
    ANGER = 'Anger'
    JOY = 'Joy'

class RobotTalkingExpression(Enum):
    """ Valid talking expressions """

    TALKING_1 = 'Talking_1'
    TALKING_2 = 'Talking_2'

OBJECT_CONFIDENCE_TOPIC = topicname('objects_confidence')
GAME_SUCCESS_TOPIC = topicname('object_guess_success')
ROBOT_FACIAL_EXPRESSION_TOPIC = topicname('robot_facial_expression')
FACIAL_EXPRESSION_COMPLETED = topicname('robot_facial_expression_completed')
TTS_QUERY_TOPIC = topicname('tts_query')

NODE_NAME = 'facial_expression'


class FacialExpression(object):
    """ Subscribes to object confidence and publishes facial expressions for a specific duration """

    FACIAL_EXPRESSION_DURATION = 10

    def __init__(self):
        self.showing_emotion = False
        self.confidence = None

        self.robot_expression_publisher = rospy.Publisher(ROBOT_FACIAL_EXPRESSION_TOPIC,
                                                          EmoIntensity, queue_size=1)

        self.expression_completed_pub = rospy.Publisher(FACIAL_EXPRESSION_COMPLETED,
                                                        Bool, queue_size=1)

        self.robot_talking_expression_publisher = rospy.Publisher(ROBOT_FACIAL_EXPRESSION_TOPIC,
                                                                  EmoPulse, queue_size=1)

        rospy.Subscriber(TTS_QUERY_TOPIC, TtsQuery, self.on_tts_query)

        rospy.Subscriber(OBJECT_CONFIDENCE_TOPIC, Float64MultiArray,
                         self.on_object_confidence)

        rospy.Subscriber(GAME_SUCCESS_TOPIC, Bool,
                         self.on_game_success)

    def on_tts_query(self, query):
        """ Callback on new tts query. Shows talking expression to robot """
        rospy.loginfo('%s received: %s', rospy.get_name(), query.text)
        print 'New Query!: '
        print query.text

    def on_object_confidence(self, objects_confidence):
        """ Callback on new object confidence. Updates max confidence """
        rospy.loginfo('%s received: %s', rospy.get_name(), objects_confidence.data)
        self.confidence = max(objects_confidence.data)

    def on_game_success(self, game_success):
        """ Callback on end game. Shows emotion depending on success and confidence """
        rospy.loginfo('%s received: %s', rospy.get_name(), game_success.data)
        expression = self.get_facial_expression(self.confidence, game_success.data)

        if expression is not None and not self.showing_emotion:
            self.show_facial_expression_for(expression, self.FACIAL_EXPRESSION_DURATION)

    def show_facial_expression_for(self, expression, duration):
        """ Publishes the expression for a specific duration"""
        value = 1  # default value
        self.showing_emotion = True

        # Special case for anger which is a bit too intense
        if expression == RobotFacialExpression.ANGER:
            value = 0.5

        face_expression = EmoIntensity(name=expression.value, value=value)
        self.robot_expression_publisher.publish(face_expression)

        rospy.sleep(duration)

        face_expression = EmoIntensity(name=expression.value, value=0)
        self.robot_expression_publisher.publish(face_expression)

        # Initialize for a new game
        self.showing_emotion = False
        self.confidence = None
        rospy.sleep(0.5)
        self.expression_completed_pub.publish(True)

    def get_facial_expression(self, confidence, success):
        """ Get the wanted expression depending on the received confidence.
        Defaults to ANGER
        """
        expression = None

        if self.confidence is not None:
            expression = RobotFacialExpression.ANGER

            # Devine Won
            if success:
                if 0 <= confidence < 0.6:
                    expression = RobotFacialExpression.SURPRISE
                else:
                    expression = RobotFacialExpression.JOY

            # Devine Lost
            else:
                if 0 <= confidence < 0.6:
                    expression = RobotFacialExpression.SAD
                else:
                    expression = RobotFacialExpression.ANGER

        return expression


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    rospy.loginfo('Running node \'%s\'', NODE_NAME)
    FacialExpression()
    rospy.spin()
