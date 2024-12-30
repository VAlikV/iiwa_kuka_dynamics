import matplotlib.pyplot as plt
import numpy as np

data = np.genfromtxt("Position.txt", delimiter="\t", dtype=float).T

data = np.insert(data,0,[0,0,0],axis=1)

# data = np.reshape(data,(8,3))
print(data.T)
print("------------------------------------------")


fig = plt.figure()

ax = plt.axes(projection='3d')
plt.axis('equal')

ax.set_xlim3d([-1, 1])
ax.set_ylim3d([-1, 1])
ax.set_zlim3d([0, 1.3])
ax.set_xlabel("X")
ax.set_ylabel("Y")

ax.plot3D(data[0], data[1], data[2], 'red')

ax.scatter3D(data[0], data[1], data[2])

a = np.array([-1.87559, 2.15057, -0.362261, 1.95107, 0.290562, 1.97678, -0.0652557])
a = a*180/np.pi
print(a)
plt.show()