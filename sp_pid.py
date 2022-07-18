#!/usr/bin/env python
import math
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import pandas as pd
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

DEBUG = True # set true to print error messages
FILE = pd.read_csv("./waypoints/world_3/waypoints.csv")

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
    goal.target_pose.pose.position.x = FILE['x'].to_numpy()[0]
    goal.target_pose.pose.position.y = FILE['y'].to_numpy()[0]
    angle = FILE['theta'].to_numpy()[0]
    (goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w) = (0.0000001,0.0000001,math.sin(angle/2), math.cos(angle/2))

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

    # x_dot = data.twist.twist.linear.x # commented out, instead calculate velocity based on (pos(t) - pos(t-1)) / dt
    # y_dot = data.twist.twist.linear.y
    # theta_dot = data.twist.twist.angular.z

    # return np.array([x,y,theta,x_dot,y_dot,theta_dot])
    return np.array([x,y,theta])

def controller_pub(u, pub):
    vel_msg = Twist()
    vel_msg.linear.x = u[0]
    vel_msg.linear.y = u[1]
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = u[2]
    # vel_msg.angular.z = 0 ######## HARDCODE TO 0 (for now)
    pub.publish(vel_msg)

def linear_sat(x, val=0.4):
    return min(val, max(-val, x))

def theta_sat(x, val=0.1):
    return min(val, max(-val, x))


waypoints_x = FILE['x'].to_numpy()[0:]
waypoints_y = FILE['y'].to_numpy()[0:]
waypoints_theta = FILE['theta'].to_numpy()[0:]
vel_x = FILE['x_dot'].to_numpy()[0:]
vel_y = FILE['y_dot'].to_numpy()[0:]
dtheta = FILE['theta_dot'].to_numpy()[0:]
num_points = np.shape(waypoints_x)[0]

log('num points: ' + str(num_points))

# define gains
ki_x = 0 # x gains
kp_x = 2

ki_y = 0.0 # y gains
kp_y = 2
# ki_y = 1.0 # y gains
# kp_y = 0.2

ki_theta = 0.0 # theta gains
kp_theta = 0.0
# ki_theta = 0.05 # theta gains
# kp_theta = 1.0


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
        desired_state_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        log('pub initialized')
        #get current robot states
        states = get_states()
        log('got current states:')
        log(states)

        x_list = [states[0]] # lists to hold x, y, theta (states)
        y_list = [states[1]]
        theta_list  = [states[2]]

        x_dot_list = [0] # lists to hold x_dot, y_dot, theta_dot (velocity of states)
        y_dot_list = [0]
        theta_dot_list  = [0]

        xd_list = [0] # lists to hold desired states (from file)
        yd_list = [0]
        thetad_list = [0]

        x_dotd_list = [0] # lists to hold desired velocities (from file)
        y_dotd_list = [0]
        theta_dotd_list = [0]

        # iterate through all waypoints in text file
        for itr in range(num_points):
            t1 = rospy.Time.now().to_sec()

            log('itr: ' + str(itr))

            states = get_states() #gets position of robot from ROS

            # actual x y and theta
            x = states[0]
            y = states[1]
            theta = states[2]

            # actual x, y and theta dot
            x_dot = (x - x_list[itr])/dt
            y_dot = (y - y_list[itr-1])/dt
            theta_dot = (theta - theta_list[itr-1])/dt

            # desired x y and theta
            x_d = waypoints_x[itr]
            y_d = waypoints_y[itr]
            theta_d = waypoints_theta[itr]

            # desired x, y and theta dot
            x_dot_d = vel_x[itr]
            y_dot_d = vel_y[itr]
            theta_dot_d = dtheta[itr]
            
            
            log('got current actual states:')
            log('x: ' + str(x))
            log('y: ' + str(y))
            log('theta: ' + str(theta))
            log('x_dot: ' + str(x_dot))
            log('y_dot: ' + str(y_dot))
            log('theta_dot: ' + str(theta_dot))

            log('got desired states:')
            log('x: ' + str(x_d))
            log('y: ' + str(y_d))
            log('theta: ' + str(theta_d))
            log('x_dot: ' + str(x_dot_d))
            log('y_dot: ' + str(y_dot_d))
            log('theta_dot: ' + str(theta_dot_d))


            # calculate the error for all states for this time step
            error = np.array([
                            x_d - x, # x position error, error[0]
                            y_d - y, # y position error, error[1]
                            theta_d - theta, # theta position error, error[2]
                            x_dot_d - x_dot, # x_dot error, error[3]
                            y_dot_d - y_dot, # y_dot error, error[4]
                            theta_dot_d - theta_dot # theta_dot error, error[5]
                            ])

            log('error:')
            log(error)

            # control_x = linear_sat(error[3]*ki_x + error[0]*kp_x) #control signal x
            # control_y = linear_sat(error[4]*ki_y + error[1]*kp_y) #control signal y
            # control_theta = theta_sat(error[5]*ki_theta + error[2]*kp_theta) #control signal theta

            control_x = float(error[3]*ki_x) + float(error[0]*kp_x) #control signal x
            control_y = float(error[4]*ki_y) + float(error[1]*kp_y) #control signal y
            control_theta = float(error[5]*ki_theta) +float(error[2]*kp_theta) #control signal theta

            log('control signals:')
            log(control_x)
            log(control_y)
            log(control_theta)

            controller_pub([control_x, control_y, control_theta], desired_state_publisher)
            #controller_pub([control_x, control_y], desired_state_publisher)

            # save actual and desired states to file
            x_list.append(x)
            y_list.append(y)
            theta_list.append(theta)

            x_dot_list.append(x_dot) # lists to hold x_dot, y_dot, theta_dot (velocity of states)
            y_dot_list.append(y_dot)
            theta_dot_list.append(theta_dot)

            xd_list.append(x_d) # lists to hold desired states (from file)
            yd_list.append(y_d)
            thetad_list.append(theta_d)

            x_dotd_list.append(x_dot_d) # lists to hold desired velocities (from file)
            y_dotd_list.append(y_dot_d)
            theta_dotd_list.append(theta_dot_d)

            t2 = rospy.Time.now().to_sec()
            elapsed_time = t2 - t1
            rate = rospy.Rate(1/(dt-elapsed_time)) 
            rate.sleep()
        
        controller_pub([0, 0, 0], desired_state_publisher) # stop when done
        log('done!')
        np.savetxt("actual_x.txt", x_list, fmt='%.2f')
        np.savetxt("actual_y.txt", y_list, fmt='%.2f')
        np.savetxt("actual_theta.txt", theta_list, fmt='%.2f')
        np.savetxt("desired_x.txt", xd_list, fmt='%.2f')
        np.savetxt("desired_y.txt", yd_list, fmt='%.2f')
        np.savetxt("desired_theta.txt", thetad_list, fmt='%.2f')

    except rospy.ROSInterruptException:
        pass