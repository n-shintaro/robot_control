# coding:utf-8
#! /usr/bin/env python

import rospy
import numpy as np
import math

"""
A class used to represent an Robot

...
Attributes
----------
(x,y): float 
    the position of robot

(x_t,y_t): float 
    target position of robot

(vel_x, vel_y) : float
    velocity of robot

goal_flag: bool
    True: the robot reached the target position

k:constant of proportionality


Methods
-------
update_state
    Update the robot position and velocity.
"""

class Robot:
    """
    Parameters
    ----------
    name : str
    ...
    The name of the animal
    sound : str
    Attributes
    The sound the animal makes
    ----------
    num_legs : int, optional
    says_str : str
    The number of legs the animal (default is 4)
    a formatted string to print out what the animal says
    """

    def __init__(self,x_init,y_init,x_t,y_t):
        self.x=x_init
        self.y=y_init
        self.x_t=x_t
        self.y_t=y_t
        self.vel_x=None
        self.vel_y=None
        self.goal_flag=False
        self.k=1.0

    def update_state(self,x,y):
         """
        Returns sum of x and y

        Parameters:
        ----------
        x : int
            operand1 of addition
        y : int
            operand2 of addition

        Returns:
        ----------
        int
            sum of x and y
        """
        
        print('x='+str(x))
        print('y='+str(y))
        self.x=x
        self.y=y
        self.vel_x=self.k*(self.x_t-self.x)
        self.vel_y=self.k*(self.y_t-self.y)