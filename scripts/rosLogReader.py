#!/usr/bin/python

import csv
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

pos3d = np.zeros((9847,3))

with open("log1.csv") as csv_file:
    csv_reader = csv.reader(csv_file)
    line_count = 0
    for row in csv_reader:
        if line_count != 0:
            r = np.array(map(float, row[5:8]))
            pos3d[line_count, :] = r
        line_count += 1

with open("log2.csv") as csv_file:
    csv_reader = csv.reader(csv_file)
    line_count2 = 0
    for row in csv_reader:
        if line_count2 != 0:
            r = np.array(map(float, row[5:8]))
            pos3d[line_count, :] = [r[0], r[1], r[2]]
        line_count2 += 1
        line_count += 1

# print pos3d[2][0:end]
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(pos3d[:,0], pos3d[:,1], pos3d[:,2])
ax.set_xlabel("x(m)")
ax.set_ylabel("y(m)")
ax.set_zlabel("depth(m)")
ax.set_zlim(-2,2)
plt.show()
