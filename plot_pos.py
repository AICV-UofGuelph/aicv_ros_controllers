#plot the acutal robot positions

import numpy as np
import matplotlib.pyplot as plt

x = np.loadtxt("actual_x.txt")
y = np.loadtxt("actual_y.txt")

x_des = np.loadtxt("desired_x.txt")
y_des = np.loadtxt("desired_y.txt")

theta_des = np.loadtxt("desired_theta.txt")
theta_acc = np.loadtxt("actual_theta.txt")

x_vals_coords = []
for i in range(len(x)):
    x_vals_coords.append(i)

x_vals_theta = []
for i in range(len(theta_des)):
    x_vals_theta.append(i)

plt.plot(x, y)
plt.plot(x_des, y_des)
plt.show()

plt.clf()
plt.plot(x_vals_coords, x)
plt.plot(x_vals_coords, x_des)
plt.show()

plt.clf()
plt.plot(x_vals_coords, y)
plt.plot(x_vals_coords, y_des)
plt.show()

plt.clf()
plt.plot(x_vals_theta, theta_des)
plt.plot(x_vals_theta, theta_acc)
plt.show()