# coding:utf-8
#! /usr/bin/env python

import rospy
import numpy as np
import math

class Robot:
    def __init__(self,x_init,y_init,x_t,y_t):
        self.x=x_init
        self.y=y_init
        self.x_t=x_t
        self.y_t=y_t
        self.goal_flag=False
        self.k=10
        self.vel_x=None
        self.vel_y=None

    def update_state(self,x,y):
        print('x='+str(x))
        print('y='+str(y))
        self.x=x
        self.y=y
        self.vel_x=self.k*(self.x_t-self.x)
        self.vel_y=self.k*(self.y_t-self.y)