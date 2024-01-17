# Trajectory Optimization

# Introduction
In this repository I have implemented a RRT* algorithm followed my trajectory optimization for generating smooth trajectories between waypoints

# Contents
1. A RRT* alogorithm is used to generate waypoints which are then refrenced by Trajectory Optimization algorithm to generate smooth trajectories.
2. The PDF in the repository shows the theory and code structure used for Trajectory Optimization. (link: https://github.com/adityaaap/Trajectory-Optimization/blob/master/Traj%20optim%20and%20RRT%20Star.pdf)

# How to Run
Clone the repository and run
```
python3 main.py
```

# Results of Path Planning 
A scatter plot of the way points generated using RRT* algorithm

((https://github.com/adityaaap/Trajectory-Optimization/blob/master/images/pathPlanning1.png))

![image](https://github.com/adityaaap/RRT_Star/blob/master/images/pathPlanning%20results%202.png)

# Results of Trajectory Optimization
A plot of smoothened trajectory around the obstacle to the goal

![image](https://github.com/adityaaap/RRT_Star/blob/master/images/Traj%20Optim%20results%201.png)

![image](https://github.com/adityaaap/RRT_Star/blob/master/images/traj%20optim%20results%202.png)

# Notes
A lecture PDF from MIT VNAV course on Trajectory Optimization is a very useful resource for trajectory optimization
Link - https://vnav.mit.edu/material/09-TrajectoryOptimization1-notes.pdf

The PDF notes posted by me in the repo explain how the theory from above lecture has been used in this problem formulation and also the code structure associated with it

# Current Work
Working to improve the collision avoidance in trajectory optimization code.


References:
1. https://github.com/Mdhvince/UAV-Autonomous-control/tree/master
2. https://ocw.mit.edu/courses/6-832-underactuated-robotics-spring-2009/resources/lecture-9-trajectory-optimization/

