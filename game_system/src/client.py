import GameSystem

gameSystem = GameSystem.GameSystem("DEVINE State Machine")

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
