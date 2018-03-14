import rospy
from std_msgs.msg import String

import GameSystem

pub = rospy.Publisher('/game_system_state', String, queue_size=10)
rospy.init_node('game_system', anonymous=False)

gameSystem = GameSystem.GameSystem("DEVINE State Machine", pub)

gameSystem.initialisation()

gameSystem.ready()

gameSystem.moveToScene()

gameSystem.takePicture()

gameSystem.turnHead()

gameSystem.askQuestion()

gameSystem.listenAnswer()

gameSystem.analyse(readyToAnswer=True)

gameSystem.point()

gameSystem.sayObject()

gameSystem.listenFinalAnswer()

gameSystem.emotion()

gameSystem.moveBack()

gameSystem.ready()

rospy.signal_shutdown('End of game');
