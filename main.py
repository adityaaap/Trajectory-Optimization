from matplotlib import pyplot as plt

# from RRTStar_FromUAV import RRTStar
from RRTStar_forTrajOptim import RRTStar

# from MinSnapTraj_FromUAV import MinimumSnap
from MinSnapTraj import MinimumSnap

import numpy as np

start = np.array([0, 0, 0])
goal = np.array([7.0*10, 7.0*10, 7.0*10]) # Dont keep goal as integer values

space_limits = np.array([[0., 0., 0.9], [100., 100., 100.]])

rrt = RRTStar(
    space_limits,
    start=start,
    goal=goal,
    max_distance=2,
    max_iterations=1000,
    obstacles=None,
)

####### Path Planning ########
# rrt = RRTStar(space_limits, start, goal, max_distance, max_iterations, obstacles)
rrt.run()
global_path = rrt.bestPath  # long-term path
# print(global_path)

####### Trajectory Optimization ######
print("start traj optim")
min_snap = MinimumSnap(global_path, obstacles=None, velocity=2, dt=0.1)

# global_trajectory = min_snap.get_trajectory()
global_trajectory = min_snap.getTrajectory()  # long-term trajectory

positions = min_snap.positions
print(positions)


######### PLOTTING #########

#### Path Plan Plotting ####
rrt.plot()


#### Trajectory Optimization Plot ####
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# tree = list(self.bestPath.values())  # Extract values from the dictionary
# tree = self.bestPath
tree = np.array(positions) 

# print(tree)
# Extracting x, y, z coordinates from the array
x = tree[:, 0]
y = tree[:, 1]
z = tree[:, 2]

# Plotting the points
# ax.scatter(x, y, z)
for i in range(len(tree) - 1):
    ax.plot([x[i], x[i + 1]], [y[i], y[i + 1]], [z[i], z[i + 1]], color='blue')

# Set labels
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

plt.show()
# print(global_trajectory)