import numpy as np
import copy
import time

class RRTStar:
    def __init__(self, spaceLimits, start, goal, max_distance, max_iterations, obstacles = None):
        self.start = start
        self.goal = goal
        self.maxIterations = max_iterations
        self.maxDistance = max_distance
        self.lower = spaceLimits[0]
        self.upper = spaceLimits[1]
        self.allNodes = [self.start]
        self.tree = {} # Edges
        self.stepSize = max_distance
        self.neighbourRadius = 1.5 * self.maxDistance
        self.epsilon = 0.15
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
        # if np.random.uniform(0, 1) < self.epsilon:
        #     return self.goal
        
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
            new_node = np.round(new_node, 0)
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
            points = node1 + np.outer(t*direction)

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

    def isPathFound(self, tree):
        distance = []
        # print(self.allNodes)
        for item in self.allNodes:
            distance.append(np.linalg.norm(item - self.goal))
        # distance = np.linalg.norm(self.allNodes - self.goal)
        print('len', len(distance))
        print(min(distance))
        
        for i in range(len(distance)):
            if(distance[i] < 3):
                
                node = self.allNodes[i]
                goal_node_key = str(self.goal.tolist())
                self.tree[goal_node_key] = node
                print('path found')
                return True
        
        return False
        
        #####################################
        # print('path found')

        # goal_node_key = str(np.round(self.goal, 2).tolist())
        # return goal_node_key in tree.keys()

    def getPath(self, tree):
        path = [self.goal]
        node = self.goal

        s_time = time.time()

        while(not np.array_equal(node, self.start)):
            node = tree[str(node.tolist())]
            path.append(node)
            # print('in while')
            # if time.time() - s_time > 5:
            #     # restart
            #     print("Restarting...")
            #     self.run()
            #     # raise Exception("A problem occurred while computing the path.")

        cost = RRTStar.path_cost(path)
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
            
            newNode = self.generateNode()
            
            nearestNode = self.findNearest(newNode)
            # print(nearestNode)
            newNode = self.unitVectorToNode(newNode, nearestNode)
            # print(newNode)
            neighbours = self.validNeighbours(newNode)
            # print(neighbours)

            if(len(neighbours) == 0): continue

            bestNeighbour = self.bestNeighbour(neighbours)
            # print(bestNeighbour)
            self.updateTree(bestNeighbour, newNode)
            hasRewired = self.rewire(neighbours, newNode)
            # print('here')
            if(self.isPathFound(self.tree)):
                print('here in run if path found')
                print(self.tree)
                path,cost = self.getPath(self.tree)
                self.store_best_tree()
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

        if not self.isPathFound(self.bestTree):
            raise Exception("No path found")

        self.bestPath, cost = self.getPath(self.bestTree)
        print("\nBest path found with cost: {}".format(cost))

if __name__ == "__main__":

    start = np.array([0, 0, 0])
    goal = np.array([7.05*10, 7.05*10, 7.05*10]) # Dont keep goal as integer values

    space_limits = np.array([[0., 0., 0.9], [100., 100., 100.]])

    rrt = RRTStar(
        space_limits,
        start=start,
        goal=goal,
        max_distance=10,
        max_iterations=10000,
        obstacles=None,
    )
    rrt.run()