#!/usr/bin/env python

import ConfigParser
import sys
import os

#Path
CONFIG_FILE_NAME = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'static', 'Parameters.ini')

Config = ConfigParser.RawConfigParser()
Config.optionxform = str
Config.read(CONFIG_FILE_NAME)

def ConfigSectionMap(section):
    dict = {}
    options = Config.options(section)
    for option in options:
        try:
            dict[option] = Config.get(section, option)
            if dict[option] == -1:
                print("skip: %s" % option)
        except:
            print("exception on %s!" % option)
            dict[option] = None
    return dict
