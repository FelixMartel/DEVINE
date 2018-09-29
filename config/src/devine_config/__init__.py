import os
import json

CONFIG_FILE_NAME = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'static', 'Parameters.json')

with open(CONFIG_FILE_NAME) as f:
    config = json.load(f)

def topicname(metaname):
    '''fetch the full name for a topic'''
    return config["topics"][metaname]["name"]

def constant(name):
    '''fetch a constant by name'''
    return config["constants"][name]

def printtopics():
    '''write json topics to stdout'''
    print(json.dumps(config["topics"]))
