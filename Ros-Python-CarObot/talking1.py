#!/usr/bin/env python


import rospy
import time
import yaml
from simple_script_server import *
import threading



sss = simple_script_server()


if __name__ == '__main__':
    rospy.init_node("talking1")   
    sss.say("sound",["Hi,"], True)
    sss.say("sound",["I am Tom,"], False)
    sss.say("sound",[" What is your name?"], False)