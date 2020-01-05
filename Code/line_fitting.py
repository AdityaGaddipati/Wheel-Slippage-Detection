import numpy as np
import matplotlib.pyplot as plt
import random

x = np.array([])
y = np.array([])
x1 = []
y1 = []

for i in range(10):
	x1.append(random.uniform(0,1))
	y1.append(x1[-1] + random.uniform(-0.1,0.1))

x = np.append(x,x1)
y = np.append(y,y1)
p = np.polyfit(x1,y1,1)

print p

plt.plot(x, y, 'o', label='Original data', markersize=10)
plt.plot(x, p[0]*x + p[1], 'r', label='Fitted line')
plt.legend()
plt.show()

