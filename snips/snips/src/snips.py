#!/usr/bin/env python2
"""
Snips ROS integration

ROS Topics
snips_ask -> question as string input
snips_answer -> answer as string output
"""
import json
import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import String

# Snips settings
SNIPS_HOST = "localhost"
SNIPS_PORT = 1883
SNIPS_TOPICS = ['hermes/intent/TSchmidty:YesNoResponse']
MQTT_CLIENT = mqtt.Client()

# ROS
ROS_PUBLISHER = rospy.Publisher('snips_answer', String, queue_size=10)


def snips_ask_callback(data):
    """
    Callback executed when a question is received from ROS
    """
    question = data.data
    rospy.loginfo("%s received: %s", rospy.get_name(), question)
    args = {'init': {'type': 'action', 'text': question, 'canBeEnqueued': True}}
    MQTT_CLIENT.publish(
        'hermes/dialogueManager/startSession', json.dumps(args))


def on_snips_connect():
    """
    Callback executed when snips is connected
    """
    rospy.loginfo("Connected to snips at %s:%i", SNIPS_HOST, SNIPS_PORT)
    for topic in SNIPS_TOPICS:
        MQTT_CLIENT.subscribe(topic)


def on_snips_message(client, userdata, msg):
    """
    Callback executed when snips receive an answer
    """
    if msg.topic not in SNIPS_TOPICS:
        return
    data = json.loads(msg.payload)
    ROS_PUBLISHER.publish(data['input'])


def create_ros_listener():
    """
    Create the ROS listeners
    """
    rospy.Subscriber('snips_ask', String, snips_ask_callback)


def on_snips_disconnect():
    """
    Callback executed when snips is disconnected
    """
    rospy.loginfo("Disconnected from snips")


def setup_snips():
    """
    Snips setup function
    """
    MQTT_CLIENT.on_connect = on_snips_connect
    MQTT_CLIENT.on_message = on_snips_message
    MQTT_CLIENT.on_disconnect = on_snips_disconnect
    MQTT_CLIENT.connect(SNIPS_HOST, SNIPS_PORT)


if __name__ == '__main__':
    rospy.init_node('snips')
    setup_snips()
    create_ros_listener()
    rospy.spin()
