import matplotlib.pyplot as plt
import numpy as np

fin = open('0000000002.txt', 'r')
a = []
i = []
j = []
u = []
for line in fin.readlines():
    a.append([float(num) for num in line.split(' ')])
    i.append(a[0][0])
    j.append(a[0][1])
    u.append(a[0][2])
    a.clear()

#i = i[0:10000]
#j = j[0:10000]
#u = u[0:10000]

fig = plt.figure()
ax = plt.axes(projection='3d')
#X, Y = np.meshgrid(x, y)
Z = (min(u), max(u), len(u))

ax.scatter(i, j, u, zdir='z', s=0.02, c=None, depthshade=True)
plt.show()

