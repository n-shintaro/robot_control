#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from robot_control.srv import RandomPos
import random

def generate_random(req,res):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    res.pos_x=random.uniform(req.min, req.max)
    res.pos_y=random.uniform(req.min, req.max)
    return AddTwoIntsResponse(req.a + req.b)

def add_target_server():
    rospy.init_node('velocity_server')
    s = rospy.Service('/random_target', RandomPos, generate_random)
    print("Ready to add target position")
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('target_server', anonymous=True)
    add_velocity_server()
