#!/usr/bin/env python
import sys, math
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import pandas as pd
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

DEBUG = True                    # set true to print error messages

# getting FILE and DT constants:
if len(sys.argv) < 2:
    print("Proper usage: python "+str(sys.argv[0])+" csv_file [time step]")
    exit()
FILE = pd.read_csv(sys.argv[1])

if len(sys.argv) > 2:
    DT = float(sys.argv[2])                # desired controller loop time [s]
else:
    DT = 0.2

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
    vel_msg.angular.z = u[2]
    # vel_msg.angular.z = 0 ######## HARDCODE TO 0 (for now)
    pub.publish(vel_msg)

def linear_sat(x, val=0.4):
    return min(val, max(-val, x))

def theta_sat(x, val=0.1):
    return min(val, max(-val, x))


waypoints_x = FILE['x'].to_numpy()[0:]
waypoints_y = FILE['y'].to_numpy()[0:]
theta = FILE['theta'].to_numpy()[0:]
vel_x = FILE['x_dot'].to_numpy()[0:]
vel_y = FILE['y_dot'].to_numpy()[0:]
dtheta = FILE['theta_dot'].to_numpy()[0:]
num_points = np.shape(waypoints_x)[0]

log('time step: ' + str(DT) + '\nnum points: ' + str(num_points))

# define gains
ki_x = 5.0 # x gains
kp_x = 0.2

ki_y = 5.0 # y gains
kp_y = 0.2

ki_theta = 0.05 # theta gains
kp_theta = 1.0


if __name__ == '__main__':
    try:
        rospy.init_node('PID_Control', anonymous=True)
        log('node initialized')

        #move robot to start point
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

            control_x = linear_sat(error[0]*ki_x + error[3]*kp_x) #control signal x
            control_y = linear_sat(error[1]*ki_y + error[4]*kp_y) #control signal y
            control_theta = theta_sat(error[2]*ki_theta + error[5]*kp_theta) #control signal theta

            log('control signals:')
            log(control_x)
            log(control_y)
            log(control_theta)

            controller_pub([control_x, control_y, control_theta], desired_state_publisher)

            t2 = rospy.Time.now().to_sec()
            elapsed_time = t2 - t1
            rate = rospy.Rate(1/(DT-elapsed_time)) 
            rate.sleep()
        
        controller_pub([0, 0, 0], desired_state_publisher) # stop when done
        log('done!')
        actual_path_arr = list(zip(actual_x, actual_y))
        desired_path_arr = list(zip(desired_x, desired_y))
        mse = np.square(np.subtract(actual_path_arr, desired_path_arr)).mean()
        rmse = math.sqrt(mse)
        log("RMSE: " + str(round(rmse, 4)))
        np.savetxt("actual_x.txt", actual_x, fmt='%.2f')
        np.savetxt("actual_y.txt", actual_y, fmt='%.2f')
        np.savetxt("actual_theta.txt", actual_theta, fmt='%.2f')
        np.savetxt("desired_x.txt", desired_x, fmt='%.2f')
        np.savetxt("desired_y.txt", desired_y, fmt='%.2f')
        np.savetxt("desired_theta.txt", desired_theta, fmt='%.2f')

    except rospy.ROSInterruptException:
        pass