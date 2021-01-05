#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Pose,Twist
from nav_msgs.msg import Odometry
from robot_control.srv import RandomPos
from robot import *

"""
    This is the file in order to send the velocity to the robot
    and update the robot position by using odom.
    node_name: robot_simulator
"""


# This declares a new instance of class Robot
# This is initialized (x_init,y_init,x_t,y_t)=(0.0, 0.0, 0.0, 0.0).

robot=Robot(0.0,0.0,0.0,0.0)


def create_target():
    """
        send the robot the new target position

        We create a handle for calling the service (/random_target)
        and call it.
        If we get the return value (response) which is the RandomPosResponse object,
        we make the robot target updated and goal is change to False.

        Parameters:
        ----------
        None

        Returns:
        ----------
        None

        Raise:
        ----------
        If the call fails, a rospy.ServiceException is thrown.
    """

    try:
        set_target=rospy.ServiceProxy('/random_target', RandomPos) # random_target client
        max=6.0
        min=-6.0
        response=set_target(min,max) # call service
        rospy.loginfo('set[%f,%f]' %(response.x,response.y)+ 'success')
        # update the robot target
        robot.x_t=response.x
        robot.y_t=response.y

    except rospy.ServiceException as e:
        rospy.logerr('fail to set target position: %s' %e)

def robot_control():

    """
        Send the velocity to the robot.

       vel_pub declares that this node is publishing to the /cmd_vel using the message type Twist.

        Parameters:
        ----------
        None

        Returns:
        ----------
        None

    """

    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # cmd_vel publisher
    rate = rospy.Rate(10) # 10hz
    vel=Twist()
    vel.linear.x=robot.vel_x
    vel.linear.y=robot.vel_y
    vel_info = "publish cmd_vel t= %s" % rospy.get_time()+" velocity={}".format(vel.linear)
    rospy.loginfo(vel_info)
    vel_pub.publish(vel) # publish the topic /cmd_vel
    rate.sleep()

def state_update_callback(odom_data):

    """
        Callback function of the subscriber "rospy.Subscriber("/odom", Odometry, state_update_callback) "
        When new messages are received, this callback is invoked with the message as the first argument.

        The position and velocity of the robot is updated by using the odometry wchich is the data subscribed.
        Parameters:
        ----------
        odom_data :Odometry

        Returns:
        ----------
        None

    """

    robot_pose=odom_data.pose.pose
    robot.update_state(robot_pose.position.x,robot_pose.position.y) # update the robot state
    # pos_info = "subscribe odom t= %s" % rospy.get_time()+" position=[%f, %f]" %(robot.x,robot.y)
    # rospy.loginfo(pos_info)

def robot_simulator():
    """
        Decide the robot next motion depending whether the robot is reaching the goal or not.

        When the the service (/odom) is called,
        the function "state_update_callback" is executed.

        First of all, we check whether the robot is reaching the goal or not.
        We consider the robot as reaching the goal when the distance between the robot and the target is below 0.1.
        When the robot is reaching the goal, the robot.goal_flag is chaged to True

        if the robot.goal_flag=True which means the robot reach the goal,
        call the function "create_target" to provide the new target to the robot.

        else
        call the function "robot" to publish cmd_vel to the robot.

        Parameters:
        ----------
        None

        Returns:
        ----------
        None

    """
    rospy.Subscriber("/odom", Odometry, state_update_callback) #service
    while not rospy.is_shutdown():
        print('the robot position =('+str(robot.x)+','+str(robot.y)+')')
        distance = math.sqrt((robot.x_t - robot.x) ** 2 + (robot.y_t - robot.y) ** 2)
        if distance<0.1:
            robot.goal_flag=True

        if (robot.goal_flag):
            print('the robot reaches the goal!!!!')
            print('send the new target to the robot')
            create_target()
            robot.goal_flag=False
        else:
            robot_control()


# main function
if __name__ == '__main__':
    """
        initialize the node.
        the function "robot_simulator" ia called.

        Parameters:
        ----------
        None

        Returns:
        ----------
        None

    """

    rospy.init_node('robot_simulator')
    try:
        rate = rospy.Rate(10)
        robot_simulator()
        rate.sleep()
    except rospy.ROSInterruptException:
        pass
