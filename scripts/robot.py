# coding:utf-8
#! /usr/bin/env python

"""
This file define the class "Robot"
"""

import rospy
import numpy as np
import math

class Robot:
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


    def __init__(self,x_init,y_init,x_t,y_t):
        """
        Parameters
        -----------
        (x,y): float
        the position of robot

        (x_t,y_t): float
        target position of robot

        (vel_x, vel_y) : float
        velocity of robot

        goal_flag: bool
            True: the robot is reaching the target position

        k: float
            constant of proportionality (default is 1.0)
        """
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
        Update the state of robot (position and velocity)
        new velocity x depends on the difference between the target x_t and the robot position x
        new velocity y depends on the difference between the target y_t and the robot position y

        Parameters:
        ----------
        x : float
            the component of position x of the robot
        y : int
            the component of position y of the robot

        Returns:
        ----------
        None
        """
        self.x=x
        self.y=y
        self.vel_x=self.k*(self.x_t-self.x)
        self.vel_y=self.k*(self.y_t-self.y)