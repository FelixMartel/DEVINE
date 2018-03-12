'''State Machine to be used for DEVINE Game System.'''
from enum import Enum
from transitions import Machine


class States(Enum):
    # pylint: disable=too-few-public-methods
    '''The possibles states of the machine'''
    __order__ = 'INITIALISATION SHOWING_INSTRUCTIONS MOVING_TO_SCENE TAKING_PICTURE TURNING_HEAD ' \
                'ASKING_QUESTION LISTENING ANALYSING GUESSING POINTING SAYING_GUESSED_OBJECT ' \
                'LISTENING_TO_FINAL_ANSWER SHOWING_EMOTION MOVE_BACK_TO_PLAYER'

    INITIALISATION = 'initialisation'
    SHOWING_INSTRUCTIONS = 'showingInstructions'
    MOVING_TO_SCENE = 'movingToScene'
    TAKING_PICTURE = 'takingPicture'
    TURNING_HEAD = 'turningHead'
    ASKING_QUESTION = 'askingQuestion'
    LISTENING = 'listening'
    ANALYSING = 'analysing'
    GUESSING = 'guessing'
    POINTING = 'pointing'
    SAYING_GUESSED_OBJECT = 'sayingGuessedObject'
    LISTENING_TO_FINAL_ANSWER = 'listeningToFinalAnswer'
    SHOWING_EMOTION = 'showingEmotion'
    MOVE_BACK_TO_PLAYER = 'movingBackToPlayer'


class GameSystem(object):
    '''The state machine itself'''
    states = []
    name = ''

    def __init__(self, name):
        self.name = name
        for state in States:
            self.states.append(state.value)

        self.machine = Machine(model=self, states=GameSystem.states,
                               send_event=True, initial='initialisation')

        # must use same states names
        self.machine.on_enter_showingInstructions('show_instructions')
        self.machine.on_enter_movingToScene('move_robot_to_scene')
        self.machine.on_enter_takingPicture('take_the_picture')
        self.machine.on_enter_turningHead('turn_head_towards_player')
        self.machine.on_enter_askingQuestion('ask_a_question')
        self.machine.on_enter_listening('listen_to_answer')
        self.machine.on_enter_analysing('analyse_answer')
        self.machine.on_enter_pointing('point_object')
        self.machine.on_enter_sayingGuessedObject('say_guessed_object')
        self.machine.on_enter_listeningToFinalAnswer('listen_to_final_answer')
        self.machine.on_enter_showingEmotion('show_emotion')
        self.machine.on_enter_movingBackToPlayer('move_back_to_player')

        self.machine.add_transition(trigger='ready', source='*',
                                    dest=States.SHOWING_INSTRUCTIONS.value)
        self.machine.add_transition(trigger='moveToScene', source=States.SHOWING_INSTRUCTIONS.value,
                                    dest=States.MOVING_TO_SCENE.value)
        self.machine.add_transition(trigger='takePicture', source=States.MOVING_TO_SCENE.value,
                                    dest=States.TAKING_PICTURE.value)
        self.machine.add_transition(trigger='turnHead', source=States.TAKING_PICTURE.value,
                                    dest=States.TURNING_HEAD.value)
        self.machine.add_transition(trigger='askQuestion', source=States.TURNING_HEAD.value,
                                    dest=States.ASKING_QUESTION.value)
        self.machine.add_transition(trigger='listenAnswer', source=States.ASKING_QUESTION.value,
                                    dest=States.LISTENING.value)
        self.machine.add_transition(trigger='analyse', source=States.LISTENING.value,
                                    dest=States.ANALYSING.value)
        self.machine.add_transition(trigger='guess', source=States.ANALYSING.value,
                                    dest=States.GUESSING.value)
        self.machine.add_transition(trigger='point', source=States.GUESSING.value,
                                    dest=States.POINTING.value)
        self.machine.add_transition(trigger='sayObject', source=States.POINTING.value,
                                    dest=States.SAYING_GUESSED_OBJECT.value)
        self.machine.add_transition(trigger='listenFinalAnswer',
                                    source=States.SAYING_GUESSED_OBJECT.value,
                                    dest=States.LISTENING_TO_FINAL_ANSWER.value)
        self.machine.add_transition(trigger='emotion',
                                    source=States.LISTENING_TO_FINAL_ANSWER.value,
                                    dest=States.SHOWING_EMOTION.value)
        self.machine.add_transition(trigger='moveBack', source=States.SHOWING_EMOTION.value,
                                    dest=States.MOVE_BACK_TO_PLAYER.value)

    def initialisation(self):
        # pylint: disable=no-self-use
        # pylint: disable=unused-argument
        """ Initial state of the Machine, waiting everything is ready to go! """
        print"Booting DEVINE state Machine..."

        # add stuff here to make sure everything is ready

        print"All systems are ready to go!"

    def show_instructions(self, event):
        # pylint: disable=no-self-use
        # pylint: disable=unused-argument
        """ Showing instructions and waiting for trigger to start the game """
        print"""\nWelcome to DEVINE GuessWhat?! Please follow the instructions above:
            1- Have fun
            2- (...)
        """

        answer = raw_input("To start a game, please type 'start'")
        while answer.strip() != 'start':
            print"'"+ answer + "'" + " is not a valid entry..."
            answer = raw_input("\nTo start a game, please type 'start'")

        print"Starting a new game!"


    def move_robot_to_scene(self, event):
        # pylint: disable=no-self-use
        # pylint: disable=unused-argument
        """ Prepares the robot to take a picture of the scene """
        print"\nMoving to scene..."

    def take_the_picture(self, event):
        # pylint: disable=no-self-use
        # pylint: disable=unused-argument
        """ Takes a picture of the scene """
        print"\nTaking picture..."

    def turn_head_towards_player(self, event):
        # pylint: disable=no-self-use
        # pylint: disable=unused-argument
        """ Turns head towards player """
        print"\nTurning head to player..."

    def ask_a_question(self, event):
        # pylint: disable=no-self-use
        # pylint: disable=unused-argument
        """ Asks a question """
        print"\nAsking a question to player..."

    def listen_to_answer(self, event):
        # pylint: disable=no-self-use
        # pylint: disable=unused-argument
        """ Waits for the answer """
        print"\nListening to player..."

    def analyse_answer(self, event):
        # pylint: disable=no-self-use
        """ Analysing the answers and take action depending if is ready or not to guess """
        print"\nAnalysing answer..."
        ready = event.kwargs.get('readyToAnswer')
        print"Analysing done!"

        if ready:
            print"Ready to guess!"
            self.machine.set_state('guessing')
        else:
            print"I want to ask another question!"
            self.machine.set_state('askingQuestion')
            self.ask_a_question(None)

    def point_object(self, event):
        # pylint: disable=no-self-use
        # pylint: disable=unused-argument
        """ Points the guessed object"""
        print"\nPointing Object..."

    def say_guessed_object(self, event):
        # pylint: disable=no-self-use
        # pylint: disable=unused-argument
        """ Say the guessed object to played """
        print"\nI guess object X"

    def listen_to_final_answer(self, event):
        # pylint: disable=no-self-use
        # pylint: disable=unused-argument
        """ Listens to final answer """
        print"\nListening to final answer"

    def show_emotion(self, event):
        # pylint: disable=no-self-use
        # pylint: disable=unused-argument
        """ Showing emotion depending on final answer """
        print"\nShowing happy face!"

    def move_back_to_player(self, event):
        # pylint: disable=no-self-use
        # pylint: disable=unused-argument
        """ Moving back to robot to player """
        print"\nMoving back to player..."
