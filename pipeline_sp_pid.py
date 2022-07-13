#!/usr/bin/env python
import os, sys, math
import getopt, shutil
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import pandas as pd
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import matplotlib.pyplot as plt

DEBUG = True                    # set true to print error messages

# checking for proper num of command line args:
if len(sys.argv) < 2 or len(sys.argv) > 6:
    print("Proper usage: python "+str(sys.argv[0])+" csv_file [-t timestep] [-f folder_name]")
    exit()

# getting FILE, DT, FOLDER_NAME:
FILE = pd.read_csv(sys.argv[1])

options, arguments = getopt.getopt(                 # code from https://realpython.com/python-command-line-arguments/#getopt
    sys.argv[2:],
    't:f:',
    ["timestep=", "folder_name="])
dt = 0.2
folder_name = None
for o, a in options:
    if o in ("-t", "--timestep"):
        dt = float(a)
    if o in ("-f", "--folder_name"):
        folder_name = a
DT = dt
FOLDER_NAME = folder_name

# FUNCTIONS:
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

def linear_sat(x, val=1):
    return min(val, max(-val, x))

def theta_sat(x, val=1.0):
    return min(val, max(-val, x))

def rotate(values, angle):
    R = np.array([[math.cos(angle), -math.sin(angle), 0],           # rotation matrix (from https://www.mathworks.com/matlabcentral/answers/93554-how-can-i-rotate-a-set-of-points-in-a-plane-by-a-certain-angle-about-an-arbitrary-point)
                  [math.sin(angle),  math.cos(angle), 0],
                  [0,                0,               1]])

    new_values = np.matmul(values,R)
    return new_values[0], new_values[1], new_values[2]

# MAIN:
waypoints_x = FILE['x'].to_numpy()
waypoints_y = FILE['y'].to_numpy()
theta_list = FILE['theta'].to_numpy()
vel_x = FILE['x_dot'].to_numpy()
vel_y = FILE['y_dot'].to_numpy()
dtheta = FILE['theta_dot'].to_numpy()
num_points = np.shape(waypoints_x)[0]

log('time step: ' + str(DT) + '\nnum points: ' + str(num_points))

# define gains
kd_x = 1.0 # x gains
kp_x = 1/DT

kd_y = kd_x # y gains
kp_y = kp_x

kd_theta = 1.0 # theta gains
kp_theta = 4.0


if __name__ == '__main__':
    try:
        rospy.init_node('PID_Control', anonymous=True)
        log('node initialized')

        # move robot to start point
        result = movebase_client()

        if result:
            rospy.loginfo("Goal execution done!")

        # state_publisher = rospy.Publisher("/odometry/filtered_map", Odometry, queue_size=10)
        desired_state_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        log('pub initialized')

        # get current robot states
        states = get_states()
        log('got current states:')
        log(states)

        actual_x = []
        actual_y = []
        actual_theta = []

        desired_x = []
        desired_y = []
        desired_theta = []

        x_d = waypoints_x[0]
        y_d = waypoints_y[0]
        theta_d = theta_list[0]
        
        # iterate through all waypoints in text file
        for itr in range(1, num_points):

            log('itr: ' + str(itr))
            t1 = rospy.Time.now().to_sec()

            states = get_states()
            log('got current states:')
            log(states)

            actual_x.append(states[0])
            actual_y.append(states[1])
            actual_theta.append(states[2])

            # current actual states:
            x = states[0]
            y = states[1]
            theta = states[2]
            x_dot = (x - actual_x[itr-1])/DT
            y_dot = (y - actual_y[itr-1])/DT
            theta_dot = (theta - actual_theta[itr-1])/DT

            # desired states:
            x_d = waypoints_x[itr]
            y_d = waypoints_y[itr]
            if itr == 1 or itr == num_points-1:
                theta_d = theta_list[itr]
            else:
                theta_d = (theta_list[itr-1] + theta_list[itr] + theta_list[itr+1])/3
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
            # error = np.array([x_d - states[0], y_d - states[1], theta_d - states[2]])

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

            control_x = linear_sat(error[0]*kp_x + error[3]*kd_x) #control signal x
            control_y = linear_sat(error[1]*kp_y + error[4]*kd_y) #control signal y
            control_theta = theta_sat(error[2]*kp_theta + error[5]*kd_theta) #control signal theta

            log('control signals:')
            log(control_x)
            log(control_y)
            log(control_theta)

            # rotate x, y, theta from global frame to robot frame:
            control_x, control_y, control_theta = rotate(np.array([control_x, control_y, control_theta]), states[2])

            controller_pub([control_x, control_y, control_theta], desired_state_publisher)

            t2 = rospy.Time.now().to_sec()
            elapsed_time = t2 - t1
            rate = rospy.Rate(1/(DT-elapsed_time)) 
            rate.sleep()
        
        controller_pub([0, 0, 0], desired_state_publisher) # stop when done
        log('done!')

        # creating run_data directory:
        dir_name = "run_data/"
        if not os.path.exists(dir_name):
            os.mkdir(dir_name)

        # creating specific run folder:
        if FOLDER_NAME == None:
            # counting num of directories in run_data:
            num_dirs = 0
            for base, dirs, files in os.walk(dir_name):
                for directories in dirs:
                    num_dirs += 1
            dir_name = dir_name+"run_"+str(num_dirs)+"/"
        else:
            dir_name = dir_name+FOLDER_NAME+"/"

        if os.path.exists(dir_name):
            shutil.rmtree(dir_name)
        os.mkdir(dir_name)

        actual_path_arr = list(zip(actual_x, actual_y))
        desired_path_arr = list(zip(desired_x, desired_y))
        mse = np.square(np.subtract(actual_path_arr, desired_path_arr)).mean()
        rmse = math.sqrt(mse)
        log("RMSE: " + str(round(rmse, 4)))

        other_run_data = ["Time step: "+str(DT), "Number of waypoints: "+str(num_points), "RMSE: " + str(round(rmse, 4))]
        # np.savetxt(dir_name+"rmse.txt", [rmse], fmt='%.4f')
        np.savetxt(dir_name+"other_data.txt", other_run_data, fmt='%s')
        np.savetxt(dir_name+"actual_x.txt", actual_x, fmt='%.2f')
        np.savetxt(dir_name+"actual_y.txt", actual_y, fmt='%.2f')
        np.savetxt(dir_name+"actual_theta.txt", actual_theta, fmt='%.2f')
        np.savetxt(dir_name+"desired_x.txt", desired_x, fmt='%.2f')
        np.savetxt(dir_name+"desired_y.txt", desired_y, fmt='%.2f')
        np.savetxt(dir_name+"desired_theta.txt", desired_theta, fmt='%.2f')

        # create/save plots:
        x_vals_coords = []
        for i in range(len(actual_x)):
            x_vals_coords.append(i)

        x_vals_theta = []
        for i in range(len(desired_theta)):
            x_vals_theta.append(i)

        plt.plot(actual_x, actual_y, label="Actual")
        plt.plot(desired_x, desired_y, label="Desired")
        plt.legend()
        plt.xlabel("x Value")
        plt.ylabel("y Value")
        plt.title("Actual vs. Desired Path")
        plt.savefig(dir_name+"path.png", bbox_inches='tight')

        plt.clf()
        plt.plot(x_vals_coords, actual_x, label="Actual")
        plt.plot(x_vals_coords, desired_x, label="Desired")
        plt.legend()
        plt.xlabel("Time")
        plt.ylabel("x Value")
        plt.title("Actual vs. Desired x Co-ordinates")
        plt.savefig(dir_name+"x_coords.png", bbox_inches='tight')

        plt.clf()
        plt.plot(x_vals_coords, actual_y, label="Actual")
        plt.plot(x_vals_coords, desired_y, label="Desired")
        plt.legend()
        plt.xlabel("Time")
        plt.ylabel("y Value")
        plt.title("Actual vs. Desired y Co-ordinates")
        plt.savefig(dir_name+"y_coords.png", bbox_inches='tight')

        plt.clf()
        plt.plot(x_vals_theta, actual_theta, label="Actual", zorder=2)
        plt.plot(x_vals_theta, desired_theta, label="Desired", zorder=1)
        plt.legend()
        plt.xlabel("Time")
        plt.ylabel("Theta")
        plt.title("Actual vs. Desired Theta Values")
        plt.savefig(dir_name+"theta.png", bbox_inches='tight')

    except rospy.ROSInterruptException:
        pass
