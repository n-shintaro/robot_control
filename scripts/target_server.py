#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from robot_control.srv import RandomPos
from robot_control.srv import RandomPosResponse
import random

def generate_random(req):
    pos_x=random.uniform(req.min, req.max)
    pos_y=random.uniform(req.min, req.max)
    print("set x="+str(pos_x)+',y='+str(pos_y))
    return RandomPosResponse(pos_x, pos_y)

def add_target_server():
    rospy.init_node('target_server')
    server = rospy.Service('/random_target', RandomPos, generate_random)
    print("Ready to add target position")
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('target_server')
    add_target_server()
