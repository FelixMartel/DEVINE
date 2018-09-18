#!/usr/bin/env python

import ConfigParser
import sys
import os

#paths
ROOT_DIR = sys.path[0]
FILE_DIR = os.path.join(ROOT_DIR, "Parameters.ini")

Config = ConfigParser.RawConfigParser()
Config.optionxform=str
Config.read(FILE_DIR)

#ConfigSectionMap("Section")['name'] 

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
