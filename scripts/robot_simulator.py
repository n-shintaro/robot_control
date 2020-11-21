#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Pose,Twist
from nav_msgs.msg import Odometry
from robot_control.srv import RandomPos
from robot import *

# (self,init_x, init_y,x_t,x_t):

robot=Robot(0.0,0.0,2.0,2.0)

def robot_simulator():
    rospy.init_node('robot_cmd_vel')
    rospy.Subscriber("/odom", Odometry, state_update_callback)
    while not rospy.is_shutdown():
        if (robot.goal_flag):
            create_target()
        else:    
            robot_control()

def create_target():
    try:
        set_target=rospy.ServiceProxy('/random_target', RandomPos)
        random_pos=RandomPos()
        max=6.0
        min=-6.0
        response=set_target(min,max)
        rospy.loginfo('set[%f,%f] success')

        robot.x_t=response.x
        robot.y_t=response.y
        robot.goal_flag=False

    except rospy.ServiceException as e:
        rospy.logerr('fail to set target position: %s' %e)

def robot_control():
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    vel=Twist()
    vel.linear.x=robot.vel_x
    vel.linear.y=robot.vel_y
    vel_info = "publish cmd_vel t= %s" % rospy.get_time()+" velocity={}".format(vel.linear)
    rospy.loginfo(vel_info)
    vel_pub.publish(vel)

def state_update_callback(odom_data):
    robot_pose=odom_data.pose.pose
    distance = math.sqrt((robot.x_t - robot.x) ** 2 + (robot.y_t - robot.y) ** 2)
    if distance<0.1:
        robot.goal_flag=True
    robot.update_state(robot_pose.position.x,robot_pose.position.y)
    pos_info = "subscribe odom t= %s" % rospy.get_time()+" position=[%f, %f]" %(robot.x,robot.y)
    rospy.loginfo(pos_info)


if __name__ == '__main__':
    try:
        robot_simulator()
    except rospy.ROSInterruptException:
        pass
