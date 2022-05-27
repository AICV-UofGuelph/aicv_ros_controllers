#plot the acutal robot positions

import numpy as np
import matplotlib.pyplot as plt

x = np.loadtxt("actual_x.txt")
y = np.loadtxt("actual_y.txt")

x_des = np.loadtxt("desired_x.txt")
y_des = np.loadtxt("desired_y.txt")

#plot unflattened map and path
plt.plot(x, y)
plt.plot(x_des, y_des)
plt.show()