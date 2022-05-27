#plot the acutal robot positions

import numpy as np
import matplotlib.pyplot as plt

x = np.loadtxt("actual_x.txt")
y = np.loadtxt("actual_y.txt")


#plot unflattened map and path
plt.plot(x, y)
plt.show()