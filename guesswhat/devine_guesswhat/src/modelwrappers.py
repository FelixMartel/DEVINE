from guesswhat.models.guesser.guesser_wrapper import GuesserWrapper
import rospy
from std_msgs.msg import String, Float32MultiArray

from queue import Queue

ANSWER_TOPIC = '/answer'
QUESTION_TOPIC = '/question'
CONFIDENCE_TOPIC = '/confidence'
SELECTION_TOPIC = '/object_found'

class GuesserROSWrapper(GuesserWrapper):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.confidence = rospy.Publisher(CONFIDENCE_TOPIC, Float32MultiArray)

    def find_object(self, *args, **kwargs):
        found, softmax, selected_object = super().find_object(*args, **kwargs)
        found = False # found is based on the oracle's choice, which we dont know

        self.confidence.publish(Float32MultiArray(data=softmax))

        return found, softmax, selected_object

class OracleROSWrapper(object):
    def __init__(self, tokenizer):
        self.tokenizer = tokenizer
        self.answers = Queue(1)
        self.questions = rospy.Publisher(QUESTION_TOPIC, String)
        rospy.Subscriber(ANSWER_TOPIC, String, self.answer_callback)

    def initialize(self, sess):
        pass

    def answer_question(self, sess, question, **kwargs):
        if self.tokenizer.stop_dialogue in question[0]:
            return [self.tokenizer.non_applicable_token]

        text_question = self.tokenizer.decode(question[0]).replace('<padding>', '').strip()

        self.questions.publish(text_question)

        while not rospy.is_shutdown():
            answer = self.answers.get(timeout=1)
            if answer is None: # check for shutdown
                continue

            if answer == 'yes':
                token = self.tokenizer.yes_token
                break
            elif answer == 'no':
                token = self.tokenizer.no_token
                break
            elif answer == 'na':
                token = self.tokenizer.not_applicable_token
                break
            else:
                rospy.logerr('Garbage on answer topic, expects yes|no|na')
        else:
            exit(1)

        return [token]

    def answer_callback(self, data):
        self.answers.put(data.data)
