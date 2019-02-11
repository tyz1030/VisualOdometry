#!/usr/bin/python

import csv
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

R = np.zeros((550, 3, 3))
T = np.zeros((3, 550))

with open("../logfile.csv") as csv_file:
    csv_reader = csv.reader(csv_file)
    line_count = 0
    R[0][:][:] = np.eye(3)
    for row in csv_reader:
        R[line_count+1, :, :] = np.array(map(float, row[0:9])).reshape((3, 3))
        T[:, line_count+1] = np.array(map(float, row[9:12]))
        # print(np.array(map(float, row[9:12])).reshape((3,1)))
        # print T[:,line_count+1]
        line_count += 1
# print(T)

R_global = R.copy()
T_global = T.copy()

for i in range(1, line_count):
    R_global[i] = np.matmul(R_global[i, :, :], R_global[i-1, :, :])
    T_global[:, i] = T_global[:, i-1] + \
        np.matmul(R_global[i-1, :, :], T_global[:, i])
    print T_global[:, i]


# for i in range(1, 100):
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(T_global[0, 1:154], T_global[1, 1:154], T_global[2, 1:154])
# ax.plot(T_global[0, 0:400], T_global[1, 0:400])

ax.set_xlabel("x(m)")
ax.set_ylabel("y(m)")
# ax.set_zlabel("depth(m)")
ax.set_xlim(-3,1)
# ax.set_ylim(-20,0)
ax.set_zlim(-3,3)
# plt.ion()
plt.show()
# plt.pause(0.3)
# plt.savefig('../data/trace' + str(i) + '.png', dpi = 300)
# plt.close()
    # plt.waitforbuttonpress()


# plt.plot(-T_global[0, :], T_global[1, :], T_global[2, :])
# print T
