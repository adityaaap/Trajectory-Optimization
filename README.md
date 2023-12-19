# Path Planning and Trajectory Planning

A RRT* alogorithm is used to generate waypoints which are then refrenced by Trajectory Optimization algorithm to generate smooth trajectories.

The PDF in the repository shows the theory behind and code structure used for Trajectory Optimization.

# How to Run
Clone the repository and run
```
python3 main.py
```

# Results of Path Planning 
A scatter plot of the way points generated using RRT* algorithm

![image](https://github.com/adityaaap/RRT_Star/blob/master/images/pathPlanning1.png)

![image]((https://github.com/adityaaap/RRT_Star/blob/master/images/pathPlanning%20results%202.png))

# Results of Trajectory Optimization
A plot of smoothened trajectory around the obstacle to the goal

![image](https://github.com/adityaaap/RRT_Star/blob/master/images/Traj%20Optim%20results%201.png)

![image](https://github.com/adityaaap/RRT_Star/blob/master/images/traj%20optim%20results%202.png)

