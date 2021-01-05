#!/usr/bin/env python

"""
    This is the file in order to give the robot target.
        node_name: target_server
"""

import rospy
from std_msgs.msg import String
from robot_control.srv import RandomPos
from robot_control.srv import RandomPosResponse
import random

def generate_random(req):
    """
        generation the target position of the robot randomly.
        the target position is generated between the interval req.min and req.max
        Parameters:
        ----------
        req :RandomPos
            req.max: the maximum value of random target position.
            req.min: the minimum value of random target position.

        Returns:
        ----------
        RandomPosResponse(pos_x, pos_y)
            pos_x: the random position x
            pos_y: the random position y
    """

    pos_x=random.uniform(req.min, req.max)
    pos_y=random.uniform(req.min, req.max)
    print("set x="+str(pos_x)+',y='+str(pos_y))
    return RandomPosResponse(pos_x, pos_y)

def add_target_server():
    """
        When the the service (/random_target) is called, 
        the function "generate_random" is executed.

        Parameters:
        ----------
        None

        Returns:
        ----------
        None

    """

    # This declares a new service named random_target with the RandomPos service type.
    # All requests are passed to generate_random function.
    # generate_random is called with instances of RandomPos and returns instances of RandomPosResponse.

    server = rospy.Service('/random_target', RandomPos, generate_random) # random target server
    print("Ready to add target position")
    rospy.spin()

# main function
if __name__ == "__main__":
    """
        initialize the node.
        the function "add_target_server" ia called

        Parameters:
        ----------
        None

        Returns:
        ----------
        None

    """
    rospy.init_node('target_server')
    add_target_server()
