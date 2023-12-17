import numpy as np
import copy
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

######################## KEY PARAMS ##########################
# 1. integer nodes
# 2. goal biasing for goal error
# 3. in ifPathFound - using dist metric and then creating a goal key
# 4. getPath - path = [] starts with self.goal Node hence, a key for self.goal is necessary

class RRTStar:
    def __init__(self, spaceLimits, start, goal, max_distance, max_iterations, obstacles = None):
        self.start = start
        self.goal = goal
        self.goalError = np.array([goal[0]-0.05, goal[1] - 0.05, goal[2] - 0.05]) # AN ERROR IS GIVEN, SO THAT WE CAN KEEP A PROBABILITY OF SAMPLING THIS ERROR GOAL POINT
        self.maxIterations = max_iterations
        self.maxDistance = max_distance
        self.lower = spaceLimits[0]
        self.upper = spaceLimits[1]
        self.allNodes = [self.start]
        self.tree = {} # Edges
        self.stepSize = max_distance
        self.neighbourRadius = 1.1 * self.maxDistance
        self.epsilon = 0.3
        self.currSample = None

        self.bestPath = None
        self.bestTree = None
        self.obstacles = obstacles

        self.dynamic_it_counter = 0
        self.dynamic_break_at = self.maxIterations / 10

    def updateTree(self, node, newNode):
        self.allNodes.append(newNode)
        key = str(newNode.tolist())
        parent = node
        if(not np.array_equal(newNode, node)):
            self.tree[key] = parent
        # print("all nodes", self.allNodes)
        # print("tree", self.tree)

    def generateNode(self):
        # with probability epsilon, sample the goal
        if np.random.uniform(0, 1) < self.epsilon:
            return self.goalError
        
        x = int(np.random.uniform(self.lower[0], self.upper[0]))
        y = int(np.random.uniform(self.lower[1], self.upper[1]))
        z = int(np.random.uniform(self.lower[2], self.upper[2]))

        randomNode = np.array([x,y,z])

        return randomNode

    def findNearest(self, newNode):
        # dist = np.linalg.norm(self.allNodes - newNode)
        # idx = np.argmin(dist)
        # nearestNode = self.allNodes[idx]
        # return nearestNode

        #################

        distances = []
        for node in self.allNodes:
            distances.append(np.linalg.norm(newNode - node))
        nearest_node = self.allNodes[np.argmin(distances)]
        return nearest_node
    
    def unitVectorToNode(self, newNode, nearestNode):
        # unitVector = newNode - nearestNode
        # unitVector = unitVector / (np.linalg.norm(unitVector))
        # newNode = nearestNode + unitVector*self.stepSize
        # newNode = np.round(newNode, 0)
        # return newNode

        ####################

        distance_nearest = np.linalg.norm(newNode - nearestNode)
        if distance_nearest > self.stepSize:
            new_node = nearestNode + (newNode - nearestNode) * self.stepSize / distance_nearest
            newNode = np.round(new_node, 0)
        return newNode

    def validNeighbours(self, newNode):
        neighbours = []
        for node in self.allNodes:
            if(np.linalg.norm(node - newNode) <= self.neighbourRadius):
                if(self.validConnection(node, newNode)):
                    neighbours.append(node)
        
        return neighbours

    def validConnection(self, node, newNode):
        if(self.obstacles is None):
            return True

        for obs in self.obstacles:
            xmin, xmax, ymin, ymax, zmin, zmax = obs
            node1, node2 = node, newNode

            direction = node2 - node1

            # Construct Line equation
            t = np.linspace(0,1,100)
            points = node1 + np.outer(t,direction)

            # checking if any points lie within obstacle
            if np.any((points[:, 0] >= xmin) & (points[:, 0] <= xmax) &
                      (points[:, 1] >= ymin) & (points[:, 1] <= ymax) &
                      (points[:, 2] >= zmin) & (points[:, 2] <= zmax)):
                return False

        return True

    def bestNeighbour(self, neighbours):
        # Find neighbour with lowest cost from the start
        cost = []
        for neigh in neighbours:
            cst = np.linalg.norm(neigh - self.start)
            cost.append(cst)

        idx = np.argmin(cost)
        return neighbours[idx]

    def rewire(self, neighbours, newNode):
        for neigh in neighbours:
            if(np.array_equal(neigh, self.tree[str(newNode.tolist())])):
                continue

            if(self.validConnection(neigh, newNode)):
                currentParent = self.tree[str(neigh.tolist())]

                currentCost = np.linalg.norm(neigh - self.start) ## THIS IS WEIRD, SHOULD BE COST OF PATH UPTIL THAT NODE

                newCost = np.linalg.norm(newNode - self.start) + np.linalg.norm(neigh - newNode)

                if(newCost < currentCost):
                    self.tree[str(neigh.tolist())] = newNode
                    return True

        return False

    def isPathFound(self, tree, newNode):
        # startTime = time.time()
        # distance = []
        # # print(self.allNodes)
        # for item in self.allNodes:
        #     distance.append(np.linalg.norm(item - self.goal))
        # # distance = np.linalg.norm(self.allNodes - self.goal)
        # print('len', len(distance))
        # print(min(distance))
        
        # for i in range(len(distance)):
        #     if(distance[i] < 4):
                
        #         node = self.allNodes[i]
        #         goal_node_key = str(self.goal.tolist())
        #         self.tree[goal_node_key] = node
        #         print('path found')
        #         return True
        
        # return False
        
        #####################################
        # print('path found')

        # goal_node_key = str(np.round(self.goalError, 2).tolist())
        # return goal_node_key in tree.keys()
        # print(self.tree)
        # goal_node_key = str(np.round(self.goal, 2).tolist())
        # endTime = time.time()
        # print('Time to run pathFound', endTime - startTime)
        # return goal_node_key in tree.keys()

        ###################################### WORKS ALL CASES BUT IS SLOW, BECAUSE IT IS BASED ON DISTANCE
        dist = np.linalg.norm(newNode - self.goal)
        # print(dist)
        if(dist < 3):
            goalKey = str(self.goal.tolist())
            tree[goalKey] = newNode
            print('path found')
            return True
        return False

        #################################### WORKS AND IS FAST, BECAUSE THIS IS LIKE SAMPLING GOAL ITSELF, WITHOUT ANY DISTANCE TO GOAL INVOLVED
        
        # if(np.array_equal(newNode, self.goalError)):
        #     goalKey = str(self.goal.tolist()) # NEED TO DO THIS BECAUSE IN FUNCTION getPath IT STARTS WITH self.goal AND I HAVE KEPT PROBABILITY OF SAMPLING ERROR GOAL AND NOT GOAL
        #     self.tree[goalKey] = self.goalError
        #     return True

    def getPath(self, tree):
        startTime = time.time()
        path = [self.goal]
        node = self.goal
        # path = [self.goalError]
        # node = self.goalError

        s_time = time.time()

        while(not np.array_equal(node, self.start)):
            node = tree[str(node.tolist())]
            path.append(node)
            # print("node", node)
            # print("path", path)
            # print('in while')
            # if time.time() - s_time > 5:
            #     # restart
            #     print("Restarting...")
            #     self.run()
            #     # raise Exception("A problem occurred while computing the path.")

        cost = RRTStar.path_cost(path)
        endTime = time.time()
        print('Time to run getPath', endTime - startTime)
        return np.array(path[::-1]).reshape(-1, 3), cost
    
    def store_best_tree(self):
        """
        Update the best tree with the current tree if the cost is lower
        """
        # deepcopy is very important here, otherwise it is just a reference. copy is enough for the
        # dictionary, but not for the numpy arrays (values of the dictionary) because they are mutable.
        self.bestTree = copy.deepcopy(self.tree)

    @staticmethod
    def path_cost(path):
        """
        Calculate the cost of the path
        """
        cost = 0
        for i in range(len(path) - 1):
            cost += np.linalg.norm(path[i + 1] - path[i])
        return cost


    def run(self):
        old_cost = np.inf
        for i in range(self.maxIterations):
            # print("iteration",i)
            # newNode = self.generateNode()
            node = self.generateNode()
            
            nearestNode = self.findNearest(node)
            # print(nearestNode)
            self.currSample = self.unitVectorToNode(node, nearestNode)
            # print("Curr Sample",self.currSample)
            neighbours = self.validNeighbours(self.currSample)
            # print(neighbours)

            if(len(neighbours) == 0): continue

            bestNeighbour = self.bestNeighbour(neighbours)
            # print(bestNeighbour)
            self.updateTree(bestNeighbour, self.currSample)
            # self.updateTree(nearestNode, self.currSample)
            hasRewired = self.rewire(neighbours, self.currSample)
            # print('here')
            if(self.isPathFound(self.tree, self.currSample)):
                print('here in run if path found')
                # print(self.tree)
                path,cost = self.getPath(self.tree)
                self.store_best_tree()
                print("iterations", i)
                break

                # if hasRewired and cost > old_cost:  # sanity check
                #     raise Exception("Cost increased after rewiring")

                # if cost < old_cost:
                #     print("Iteration: {} | Cost: {}".format(i, cost))
                #     self.store_best_tree()
                #     old_cost = cost
                #     self.dynamic_it_counter = 0
                # else:
                #     self.dynamic_it_counter += 1
                #     print(
                #         "\r Percentage to stop unless better path is found: {}%".format(
                #             np.round(self.dynamic_it_counter / self.dynamic_break_at * 100, 2)), end="\t")

                # if self.dynamic_it_counter >= self.dynamic_break_at:
                #     break

        if not self.isPathFound(self.bestTree, self.currSample):# IF ITERATIONS COMPLETE AND THIS RUNS< IT WILL GIVE KEY ERROR> BECAUSE GOAL KEY DOESNOT EXIST IN THE TREE
            raise Exception("No path found")

        self.bestPath, cost = self.getPath(self.bestTree)
        print("\nBest path found with cost: {}".format(cost))
        # print(self.bestPath)

    def plot(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # tree = list(self.bestPath.values())  # Extract values from the dictionary
        tree = self.bestPath
        tree = np.array(tree) 

        print(tree)
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


if __name__ == "__main__":

    start = np.array([0, 0, 0])
    goal = np.array([7.05*10, 7.05*10, 7.05*10]) # Dont keep goal as integer values

    space_limits = np.array([[0., 0., 0.9], [100., 100., 100.]])

    rrt = RRTStar(
        space_limits,
        start=start,
        goal=goal,
        max_distance=3,
        max_iterations=1000,
        obstacles=None,
    )
    rrt.run()
    rrt.plot()
