#!/usr/bin/env python


import rospy
import time
import yaml
from simple_script_server import *
import threading



sss = simple_script_server()


if __name__ == '__main__':
    rospy.init_node("talking1")   
    sss.say("sound",["Ok Nakshatra! Nice to meet you"], True)
    sss.say("sound",["Lets be friends"], True)
    sss.say("sound",[" I need to go"], True)
    sss.say("sound",["Bye"], True)
    sss.say("sound",["See You later"], False)