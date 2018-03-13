#!/usr/bin/env python

from transitions import Machine
from enum import Enum

import rospy
from std_msgs.msg import String

class GameSystem(object):
    states = []
    name = ''

    def __init__(self, name, pub):
        self.name = name
        self.pub = pub
        for state in States:
            self.states.append(state.value)

        self.machine = Machine(model=self, states=GameSystem.states, send_event=True, initial='initialisation')

        # must use same states names
        self.machine.on_enter_showingInstructions('showInstructions')
        self.machine.on_enter_movingToScene('moveRobotToScene')
        self.machine.on_enter_takingPicture('takeThePicture')
        self.machine.on_enter_turningHead('turnHeadTowardsPlayer')
        self.machine.on_enter_askingQuestion('askAQuestion')
        self.machine.on_enter_listening('listenToAnswer')
        self.machine.on_enter_analysing('analyseAnswer')
        self.machine.on_enter_pointing('pointObject')
        self.machine.on_enter_sayingGuessedObject('sayGuessedObject')
        self.machine.on_enter_listeningToFinalAnswer('listenToFinalAnswer')
        self.machine.on_enter_showingEmotion('showEmotion')
        self.machine.on_enter_movingBackToPlayer('moveBackToPlayer')

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
        self.machine.add_transition(trigger='listenFinalAnswer', source=States.SAYING_GUESSED_OBJECT.value,
                                    dest=States.LISTENING_TO_FINAL_ANSWER.value)
        self.machine.add_transition(trigger='emotion', source=States.LISTENING_TO_FINAL_ANSWER.value,
                                    dest=States.SHOWING_EMOTION.value)
        self.machine.add_transition(trigger='moveBack', source=States.SHOWING_EMOTION.value,
                                    dest=States.MOVE_BACK_TO_PLAYER.value)

    def initialisation(self):
        """ Initial state of the Machine, waiting everything is ready to go! """
        print("Booting DEVINE state Machine...")

        # add stuff here to make sure everything is ready

        print("All systems are ready to go!")
        self.printDiagnostic("All systems are ready to go!")

    def showInstructions(self, event):
        """ Showing instructions and waiting for trigger to start the game """
        print("""\nWelcome to DEVINE GuessWhat?! Please follow the instructions above:
            1- Have fun
            2- (...)
        """)

        answer = raw_input("To start a game, please type 'start'")
        while answer.strip() != 'start':
            print("'"+ answer + "'" + " is not a valid entry...")
            self.printDiagnostic("'"+ answer + "'" + " is not a valid entry...")
            answer = raw_input("\nTo start a game, please type 'start'")

        print("Starting a new game!")
        self.printDiagnostic("Starting a new game!")


    def moveRobotToScene(self, event):
        """ Prepares the robot to take a picture of the scene """
        print("\nMoving to scene...")
        self.printDiagnostic("Moving to scene...")

    def takeThePicture(self, event):
        """ Takes a picture of the scene """
        print("\nTaking picture...")
        self.printDiagnostic("Taking picture...")

    def turnHeadTowardsPlayer(self, event):
        """ Turns head towards player """
        print("\nTurning head to player...")
        self.printDiagnostic("Turning head to player...")

    def askAQuestion(self, event):
        """ Asks a question """
        print("\nAsking a question to player...")
        self.printDiagnostic("Asking a question to player...")

    def listenToAnswer(self, event):
        """ Waits for the answer """
        print("\nListening to player...")
        self.printDiagnostic("Listening to player...")

    def analyseAnswer(self, event):
        """ Analysing the answers and take action depending if is ready or not to guess """
        print("\nAnalysing answer...")
        self.printDiagnostic("Analysing answer...")
        ready = event.kwargs.get('readyToAnswer')
        print("Analysing done!")
        self.printDiagnostic("Analysing done!")

        if ready:
            print("Ready to guess!")
            self.printDiagnostic("Ready to guess!")
            self.machine.set_state('guessing')
        else:
            print("I want to ask another question!")
            self.printDiagnostic("I want to ask another question!")
            self.machine.set_state('askingQuestion')
            self.askAQuestion(None)

    def pointObject(self, event):
        """ Points the guessed object"""
        print("\nPointing Object...")
        self.printDiagnostic("Pointing Object...")

    def sayGuessedObject(self, event):
        """ Say the guessed object to played """
        print("\nI guess object X")
        self.printDiagnostic("I guess object X")

    def listenToFinalAnswer(self, event):
        """ Listens to final answer """
        print("\nListening to final answer")
        self.printDiagnostic("Listening to final answer")

    def showEmotion(self, event):
        """ Showing emotion depending on final answer """
        print("\nShowing happy face!")
        self.printDiagnostic("Showing happy face!")

    def moveBackToPlayer(self, event):
        """ Moving back to robot to player """
        print("\nMoving back to player...")
        self.printDiagnostic("Moving back to player...")

    def printDiagnostic(self, text):
        if not rospy.is_shutdown():
            try:
                message = text + " -  %s" % rospy.get_time()
                rospy.loginfo(message)
                self.pub.publish(message)
            except rospy.ROSInterruptException:
                pass

# the possibles states of the GameSystem's machine
class States(Enum):
    __order__ = 'INITIALISATION SHOWING_INSTRUCTIONS MOVING_TO_SCENE TAKING_PICTURE TURNING_HEAD ASKING_QUESTION ' \
                'LISTENING ANALYSING GUESSING POINTING SAYING_GUESSED_OBJECT LISTENING_TO_FINAL_ANSWER SHOWING_EMOTION '\
                'MOVE_BACK_TO_PLAYER'

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
