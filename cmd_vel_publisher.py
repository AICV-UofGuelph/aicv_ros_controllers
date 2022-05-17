#!/usr/bin/env python
from operator import indexOf
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def talker():
    msg = Twist()
    pub = rospy.Publisher('/pub_cmd_vel', Twist, queue_size=10)
    rospy.init_node('vel_talker')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        cmd_in = input("enter <lin_vel,angular_vel>: ")
        #txt = str(cmd_in).split(',')
        lin_vel = float(cmd_in[0])
        ang_vel = float(cmd_in[1])
        rospy.loginfo(lin_vel)
        rospy.loginfo(ang_vel)
        msg.linear.x = lin_vel
        msg.linear.y = 0
        msg.angular.z = ang_vel
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass