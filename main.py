from matplotlib import pyplot as plt

# from RRTStar_FromUAV import RRTStar
from RRTStar_forTrajOptim import RRTStar

# from MinSnapTraj_FromUAV import MinimumSnap
from MinSnapTraj import MinimumSnap

import numpy as np

start = np.array([0, 0, 0])
goal = np.array([7.0*10, 7.0*10, 7.0*10]) # Dont keep goal as integer values

space_limits = np.array([[0., 0., 0.9], [100., 100., 100.]])
obs = [[15,55, 15,55, 15,55]]

rrt = RRTStar(
    space_limits,
    start=start,
    goal=goal,
    max_distance=5,
    max_iterations=1000,
    obstacles=[[15,55, 15,55, 15,55]],
)

####### Path Planning ########
# rrt = RRTStar(space_limits, start, goal, max_distance, max_iterations, obstacles)
rrt.run()
global_path = rrt.bestPath  # long-term path
# print(global_path)

####### Trajectory Optimization ######
print("start traj optim")
min_snap = MinimumSnap(global_path, obstacles=obs, velocity=2, dt=0.1)

# global_trajectory = min_snap.get_trajectory()
global_trajectory = min_snap.getTrajectory()  # long-term trajectory

positions = min_snap.positions
print(positions)


######### PLOTTING #########

#### Path Plan Plotting ####

rrt.plot()


#### Trajectory Optimization Plot ####
#### DO NOT RUN PLOT FUNCTION INSIDE MinSnapTraj.py FROM main.py - NOT YET CONFIGURED
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
ax.scatter(x, y, z, color='blue')
# for i in range(len(tree) - 1):
#     ax.plot([x[i], x[i + 1]], [y[i], y[i + 1]], [z[i], z[i + 1]], color='blue')

# Set labels
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

x_min, x_max = 15,55
y_min, y_max = 15,55
z_min, z_max = 15,55

# Plotting the cube
def plot_cube(ax, min_x, max_x, min_y, max_y, min_z, max_z):
    # Define the points of the cube
    vertices = [
        [min_x, min_y, min_z],
        [max_x, min_y, min_z],
        [max_x, max_y, min_z],
        [min_x, max_y, min_z],
        [min_x, min_y, max_z],
        [max_x, min_y, max_z],
        [max_x, max_y, max_z],
        [min_x, max_y, max_z]
    ]

    # Define the edges of the cube
    edges = [
        [vertices[0], vertices[1], vertices[2], vertices[3], vertices[0]],
        [vertices[4], vertices[5], vertices[6], vertices[7], vertices[4]],
        [vertices[0], vertices[4]],
        [vertices[1], vertices[5]],
        [vertices[2], vertices[6]],
        [vertices[3], vertices[7]]
    ]

    for edge in edges:
        x, y, z = zip(*edge)
        ax.plot(x, y, z, color='red')

# Plot the cube
plot_cube(ax, x_min, x_max, y_min, y_max, z_min, z_max)

plt.show()
# print(global_trajectory)