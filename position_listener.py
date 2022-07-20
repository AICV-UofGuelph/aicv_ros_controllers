#!/usr/bin/env python
import imp
import math
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

def normalize(x):
    x = x % 360
    if x < 0:
        x += 360
    return x

def callback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    qt = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w
    )
    (roll, pitch, theta) = euler_from_quaternion(qt)

    theta = normalize(theta)

    x_dot = data.twist.twist.linear.x
    y_dot = data.twist.twist.linear.y
    theta_dot = data.twist.twist.angular.z

    print("Odom:")
    print("X:" + str(round(x,2)) + " Y:" + str(round(y,2)) + " Theta:" + str(round(math.degrees(theta),2)))

def callback2(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    qt = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w
    )
    (roll, pitch, theta) = euler_from_quaternion(qt)

    # x_dot = data.twist.twist.linear.x
    # y_dot = data.twist.twist.linear.y
    # theta_dot = data.twist.twist.angular.z

    print("amcl:")
    print("X:" + str(round(x,2)) + " Y:" + str(round(y,2)) + " Theta:" + str(round(math.degrees(theta),2)))
    
def position_listener():

    rospy.init_node('position_listener', anonymous=True)

    rospy.Subscriber("/robot/robotnik_base_control/odom", Odometry, callback)
    rospy.Subscriber("/robot/amcl_pose", PoseWithCovarianceStamped, callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    position_listener()