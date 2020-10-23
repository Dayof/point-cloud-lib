import matplotlib.pyplot as plt
import numpy as np

a, i, j, u = [], [], [], []
with open('0000000002.txt', 'r') as fin:
    for line in fin.readlines():
        a.append([float(num) for num in line.split(' ')])
        i.append(a[0][0])
        j.append(a[0][1])
        u.append(a[0][2])
        a.clear()

fig = plt.figure()
ax = plt.axes(projection='3d')
Z = (min(u), max(u), len(u))

ax.scatter(i, j, u, zdir='z', s=0.02, c=None, depthshade=True)
plt.show()

