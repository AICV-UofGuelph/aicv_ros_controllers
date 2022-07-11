#plot the acutal robot positions

import sys #, os
import numpy as np
import matplotlib.pyplot as plt

# getting run name constant:
if len(sys.argv) < 2:
    print("Proper usage: python "+str(sys.argv[0])+" run_name")
    exit()
RUN_NAME = sys.argv[1]

# load data:
x = np.loadtxt("run_data/"+RUN_NAME+"/actual_x.txt")
y = np.loadtxt("run_data/"+RUN_NAME+"/actual_y.txt")

x_des = np.loadtxt("run_data/"+RUN_NAME+"/desired_x.txt")
y_des = np.loadtxt("run_data/"+RUN_NAME+"/desired_y.txt")

theta_des = np.loadtxt("run_data/"+RUN_NAME+"/desired_theta.txt")
theta_acc = np.loadtxt("run_data/"+RUN_NAME+"/actual_theta.txt")

x_vals_coords = []
for i in range(len(x)):
    x_vals_coords.append(i)

x_vals_theta = []
for i in range(len(theta_des)):
    x_vals_theta.append(i)

# # create plots folder:
# dir_name = "plots/"
# if not os.path.exists(dir_name):
#     os.mkdir(dir_name)

# create/show/save plots:
plt.plot(x, y, label="Actual")
plt.plot(x_des, y_des, label="Desired")
plt.legend()
plt.xlabel("x Value")
plt.ylabel("y Value")
plt.title("Actual vs. Desired Path")
# plt.savefig(dir_name+"path.png", bbox_inches='tight')
plt.show()

plt.clf()
plt.plot(x_vals_coords, x, label="Actual")
plt.plot(x_vals_coords, x_des, label="Desired")
plt.legend()
plt.xlabel("Time")
plt.ylabel("x Value")
plt.title("Actual vs. Desired x Co-ordinates")
# plt.savefig(dir_name+"x_coords.png", bbox_inches='tight')
plt.show()

plt.clf()
plt.plot(x_vals_coords, y, label="Actual")
plt.plot(x_vals_coords, y_des, label="Desired")
plt.legend()
plt.xlabel("Time")
plt.ylabel("y Value")
plt.title("Actual vs. Desired y Co-ordinates")
# plt.savefig(dir_name+"y_coords.png", bbox_inches='tight')
plt.show()

plt.clf()
plt.plot(x_vals_theta, theta_acc, label="Actual", zorder=2)
plt.plot(x_vals_theta, theta_des, label="Desired", zorder=1)
plt.legend()
plt.xlabel("Time")
plt.ylabel("Theta")
plt.title("Actual vs. Desired Theta Values")
# plt.savefig(dir_name+"theta.png", bbox_inches='tight')
plt.show()
