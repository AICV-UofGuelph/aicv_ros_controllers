#!/usr/bin/env python
import imp
import math
import numpy as np
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import pandas as pd
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

DEBUG = True # set true to print error messages

def log(s):
    if DEBUG:
        print(s)

def movebase_client():
    log('moving to start point')
    client = actionlib.SimpleActionClient('/robot/move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "robot_map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = -1
    goal.target_pose.pose.position.y = 4.05
    goal.target_pose.pose.orientation.z = 1.571

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def get_states():
    log('waiting for odom message')
    data = rospy.wait_for_message("/robot/robotnik_base_control/odom", Odometry)
    log('got odom message')

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    qt = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w
    )
    (roll, pitch, theta) = euler_from_quaternion(qt)

    x_dot = data.twist.twist.linear.x
    y_dot = data.twist.twist.linear.y
    theta_dot = data.twist.twist.angular.z

    return np.array([x,y,theta,x_dot,y_dot,theta_dot])

def controller_pub(u, pub):
    vel_msg = Twist()
    vel_msg.linear.x = u[0]
    vel_msg.linear.y = u[1]
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    #vel_msg.angular.z = u[2]
    vel_msg.angular.z = 0 ######## HARDCODE TO 0 (for now)
    pub.publish(vel_msg)

#df = pd.read_csv("./paths/waypoints_path_0.csv")
df = pd.read_csv("./paths/waypoints_path_0_1ms.csv")
waypoints_x = df['x'].to_numpy()[0:]
waypoints_y = df['y'].to_numpy()[0:]
theta = df['theta'].to_numpy()[0:]
vel_x = df['x_dot'].to_numpy()[0:]
vel_y = df['y_dot'].to_numpy()[0:]
dtheta = df['theta_dot'].to_numpy()[0:]
num_points = np.shape(waypoints_x)[0]

log('num points: ' + str(num_points))

# define gains
kp_x = -0.4 # x gains
kd_x = -0.2

kp_y = -0.4 # y gains
kd_y = -0.2

kp_theta = 0.1 # theta gains
kd_theta = 0.2


if __name__ == '__main__':
    try:
        rospy.init_node('PID_Control', anonymous=True)
        log('node initialized')
        dt = 0.2 #desired controller loop time [s]


        #move robot to start point (hardcoded for now)
        result = movebase_client()

        if result:
            rospy.loginfo("Goal execution done!")

        #state_publisher = rospy.Publisher("/odometry/filtered_map", Odometry, queue_size=10)
        desired_state_publisher = rospy.Publisher('/pub_cmd_vel', Twist, queue_size=10)

        log('pub initialized')
        #get current robot states
        states = get_states()
        log('got current states:')
        log(states)
        log('state 0:')
        log(states[0])

        actual_x = []
        actual_y = []
        actual_theta = []

        desired_x = []
        desired_y = []
        desired_theta = []
        # iterate through all waypoints in text file
        for itr in range(num_points):
            log('itr: ' + str(itr))
            t1 = rospy.Time.now().to_sec()

            states = get_states()
            log('got current states:')
            log(states)

            actual_x.append(states[0])
            actual_y.append(states[1])
            actual_theta.append(states[2])

            x_d = waypoints_x[itr]
            y_d = waypoints_y[itr]
            theta_d = theta[itr]
            x_dot_d = vel_x[itr]
            y_dot_d = vel_y[itr]
            theta_dot_d = dtheta[itr]

            #theta_dot_d = states[2] #assume no error for now (set theta desired to current theta so not trying to correct currently)

            desired_x.append(x_d)
            desired_y.append(y_d)
            desired_theta.append(theta_d)

            log('got desired states:')
            log('x: ' + str(x_d))
            log('y: ' + str(y_d))
            log('theta: ' + str(theta_d))


            # calculate the error for all states for this time step
            error = np.array([x_d - states[0], y_d - states[1], theta_d - states[2],
                                x_dot_d - states[3], y_dot_d - states[4], theta_dot_d - states[5]])

            log('error:')
            log(error)

            control_x = error[0]*kp_x + error[3]*kd_x #control signal x
            control_y = error[1]*kp_y + error[4]*kd_y #control signal y
            control_theta = error[2]*kp_theta + error[5]*kd_theta #control signal theta

            log('control signals:')
            log(control_x)
            log(control_y)
            log(control_theta)

            controller_pub([control_x, control_y, control_theta], desired_state_publisher)
            #controller_pub([control_x, control_y], desired_state_publisher)

            t2 = rospy.Time.now().to_sec()
            elapsed_time = t2 - t1
            rate = rospy.Rate(1/(dt-elapsed_time)) 
            rate.sleep()
        
        controller_pub([0, 0, 0], desired_state_publisher) # stop when done
        log('done!')
        np.savetxt("actual_x.txt", actual_x, fmt='%.2f')
        np.savetxt("actual_y.txt", actual_y, fmt='%.2f')
        np.savetxt("actual_theta.txt", actual_theta, fmt='%.2f')
        np.savetxt("desired_x.txt", desired_x, fmt='%.2f')
        np.savetxt("desired_y.txt", desired_y, fmt='%.2f')
        np.savetxt("desired_theta.txt", desired_theta, fmt='%.2f')

    except rospy.ROSInterruptException:
        pass