#!/usr/bin/env python2
import rospy
import paho.mqtt.client as mqtt
import json
from std_msgs.msg import String

#Snips settings
SNIPS_HOST = "localhost"
SNIPS_PORT = 1883
SNIPS_TOPICS = ['hermes/intent/TSchmidty:YesNoResponse']
mqttClient = mqtt.Client()

'''
ROS Topics
snips_ask -> question as string input
snips_answer -> answer as string output
'''
#ROS
ROSPublisher = rospy.Publisher('snips_answer', String, queue_size=10)

def snipsAskCallback(data):
    question = data.data
    rospy.loginfo(rospy.get_name() + " received: %s", question)
    args = {'init': {'type': 'action', 'text': question, 'canBeEnqueued': True}}
    mqttClient.publish('hermes/dialogueManager/startSession', json.dumps(args))
    
def onSnipsConnect(client, userdata, flags, rc):
    rospy.loginfo("Connected to snips at %s:%i", SNIPS_HOST, SNIPS_PORT)
    for topic in SNIPS_TOPICS:
        mqttClient.subscribe(topic)

def onSnipsMessage(client, userdata, msg):
    if msg.topic not in SNIPS_TOPICS:
        return
    data = json.loads(msg.payload)
    ROSPublisher.publish(data['input'])

def createRosListener():
    rospy.Subscriber('snips_ask', String, snipsAskCallback)

def onSnipsDisconnect():
    rospy.loginfo("Disconnected from snips")

def setupSnips():
    mqttClient.on_connect = onSnipsConnect
    mqttClient.on_message = onSnipsMessage
    mqttClient.on_disconnect = onSnipsDisconnect
    mqttClient.connect(SNIPS_HOST, SNIPS_PORT)

if __name__ == '__main__':
    rospy.init_node('snips')
    setupSnips()
    createRosListener()
    rospy.spin()