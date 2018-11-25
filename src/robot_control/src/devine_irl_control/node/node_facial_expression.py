#!/usr/bin/env python2
"""
Simple ROS node that subscribes to objects_confidence and object_guess_success
and publishes to robot_facial_expression to show facial emotions
"""
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

from enum import Enum
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from jn0_face_msgs.msg import EmoIntensity
from devine_config import topicname


class RobotExpression(Enum):
    """ Valid expressions """

    SURPRISE = 'Surprise' 
    SAD = 'Sad'
    ANGER = 'Anger'
    JOY = 'Joy'


OBJECT_CONFIDENCE_TOPIC = topicname('objects_confidence')
GAME_SUCCESS_TOPIC = topicname('object_guess_success')
ROBOT_EXPRESSION_TOPIC = topicname('robot_facial_expression')
FACIAL_EXPRESSION_COMPLETED = topicname('robot_facial_expression_completed')

NODE_NAME = 'facial_expression'


class FacialExpression(object):
    """ Subscribes to object confidence and publishes facial expression for a specific duration """

    EXPRESSION_DURATION = 10

    def __init__(self):
        self.showing_emotion = False
        self.confidence = None

        self.robot_expression_publisher = rospy.Publisher(ROBOT_EXPRESSION_TOPIC,
                                                          EmoIntensity, queue_size=1)
        self.expression_completed_pub = rospy.Publisher(FACIAL_EXPRESSION_COMPLETED,
                                                        Bool, queue_size=1)
        rospy.Subscriber(OBJECT_CONFIDENCE_TOPIC, Float64MultiArray,
                         self.on_new_object_confidence)

        rospy.Subscriber(GAME_SUCCESS_TOPIC, Bool,
                         self.on_game_success)

    def on_new_object_confidence(self, objects_confidence):
        """ Callback on new object confidence. Updates max confidence """
        self.confidence = max(objects_confidence.data)

    def on_game_success(self, game_success):
        """ Callback on end game. Shows emotion depending on success and confidence """
        expression = self.get_expression(self.confidence, game_success.data)

        if expression is not None and not self.showing_emotion:
            self.show_expression_for(expression, self.EXPRESSION_DURATION)

    def show_expression_for(self, expression, duration):
        """ Publishes the expression for a specific duration"""
        value = 1  # default value
        self.showing_emotion = True

        # Special case for anger which is a bit too intense
        if expression == RobotExpression.ANGER:
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

    def get_expression(self, confidence, success):
        """ Get the wanted expression depending on the received confidence.
        Defaults to ANGER
        """
        expression = None

        if self.confidence is not None:
            expression = RobotExpression.ANGER

            # Devine Won
            if success:
                if 0 <= confidence < 0.6:
                    expression = RobotExpression.SURPRISE
                else:
                    expression = RobotExpression.JOY

            # Devine Lost
            else:
                if 0 <= confidence < 0.6:
                    expression = RobotExpression.SAD
                else:
                    expression = RobotExpression.ANGER

        return expression


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    rospy.loginfo('Running node \'%s\'', NODE_NAME)
    FacialExpression()
    rospy.spin()
