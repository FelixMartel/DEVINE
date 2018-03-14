'''Example of how to trigger different states on the State Machine'''
import rospy
from std_msgs.msg import String
import src.game_system as GameSystem


if __name__ == "__main__":

    pub = rospy.Publisher('/game_system_state', String, queue_size=10)
    rospy.init_node('game_system', anonymous=False)
    # pylint: disable=no-member
    GAME_SYSTEM = GameSystem.GameSystem("DEVINE State Machine")

    GAME_SYSTEM.initialisation()

    GAME_SYSTEM.ready()

    GAME_SYSTEM.moveToScene()

    GAME_SYSTEM.takePicture()

    GAME_SYSTEM.turnHead()

    GAME_SYSTEM.askQuestion()

    GAME_SYSTEM.listenAnswer()

    GAME_SYSTEM.analyse(readyToAnswer=True)

    GAME_SYSTEM.point()

    GAME_SYSTEM.sayObject()

    GAME_SYSTEM.listenFinalAnswer()

    GAME_SYSTEM.emotion()

    GAME_SYSTEM.moveBack()

    GAME_SYSTEM.ready()

    rospy.signal_shutdown('End of game');
