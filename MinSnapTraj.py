from matplotlib import pyplot as plt
import numpy as np

class MinimumSnap:
    def __init__(self, globalPath, obstacles, velocity, dt):
        self.globalPath = globalPath
        self.obstacles = obstacles
        self.velocity = velocity
        self.dt = dt
        self.n_coeffs = 8

        self.xmin = self.obstacles[0][0]
        self.xmax = self.obstacles[0][1]
        self.ymin = self.obstacles[0][2]
        self.ymax = self.obstacles[0][3]
        self.zmin = self.obstacles[0][4]
        self.zmax = self.obstacles[0][5]

        self.time = []
        self.A = None
        self.B = None
        # self.coeffs = None
        self.noSplines = None

        self.positions = []
        self.velocities = []
        self.acc = []
        self.spline_id = []

        self.coeffs = None

        self.rowCounter = 0
        self.completeTraj = None

    def reset(self):
        self.time = []
        self.A = None
        self.B = None
        # self.coeffs = None
        self.noSplines = None

        self.positions = []
        self.velocities = []
        self.acc = []
        self.spline_id = []

        self.coeffs = None

        self.rowCounter = 0
        self.completeTraj = None


    def getTrajectory(self):
        self.generateTrajectroy()
        return self.completeTraj
    
    def generateTrajectroy(self, method = "lstq"):
        self.computeSplineParameters(method)

        # generate trajectory points at step size of dt
        for i in range(self.noSplines):
            time = self.time[i]
            for tT in np.arange(0.0, time, self.dt):
                pos = self.generatepolynomial(self.n_coeffs, order=0, t=tT) @ self.coeffs[i * self.n_coeffs : (i+1) * self.n_coeffs]
                vel = MinimumSnap.generatepolynomial(self.n_coeffs, order=1, t=tT) @ self.coeffs[i * self.n_coeffs : (i+1) * self.n_coeffs]
                acc = MinimumSnap.generatepolynomial(self.n_coeffs, order=2, t=tT) @ self.coeffs[i * self.n_coeffs : (i+1) * self.n_coeffs]
                
                self.positions.append(pos)
                self.velocities.append(vel)
                self.acc.append(acc)
                #### NO YAW ANGLE CALCULATIONS DONE ####

        
        self.completeTraj = np.hstack((self.positions, self.velocities, self.acc))
        # return self.completeTraj
                


    def computeSplineParameters(self,method):
        self.createPolyMatrices()
        if(method == 'lstq'):
            self.coeffs,_,_,_ = np.linalg.lstsq(self.A, self.B, rcond=None)
        else:
            self.coeffs = np.linalg.solve(self.A, self.B)


    def createPolyMatrices(self):
        self.setup()
        self.positionConstraints()
        self.startGoalConstraints()
        self.continuityConstraints()

    
    def setup(self):
        self.numberSplines()
        self.computeTime()
        self.initMatrices()
        
        # print("here")
        
        
    def positionConstraints(self):
        # waypoint constraints at t=0 and t=T

        # at t=0
        for i in range(self.noSplines):
            poly = MinimumSnap.generatepolynomial(self.n_coeffs, order=0, t=0)
            self.A[self.rowCounter, i * self.n_coeffs : (i+1) * self.n_coeffs] = poly
            self.B[self.rowCounter,:]= self.globalPath[i]
            self.rowCounter += 1

        # at t=T
        for i in range(self.noSplines):
            tT = self.time[i]
            poly = MinimumSnap.generatepolynomial(self.n_coeffs, order=0, t=tT)
            self.A[self.rowCounter, i * self.n_coeffs : (i+1) * self.n_coeffs] = poly
            self.B[self.rowCounter,:]= self.globalPath[i+1]
            self.rowCounter += 1

    def startGoalConstraints(self):
        for k in [1,2,3]:
            poly = MinimumSnap.generatepolynomial(self.n_coeffs, order=k, t=0)
            self.A[self.rowCounter,0:self.n_coeffs] = poly
            self.rowCounter += 1

        for k in [1,2,3]:
            poly = MinimumSnap.generatepolynomial(self.n_coeffs, order=k, t=self.time[-1])
            self.A[self.rowCounter, (self.noSplines-1) * self.n_coeffs : self.noSplines * self.n_coeffs] = poly
            self.rowCounter += 1

    
    def continuityConstraints(self):
        for i in range(1, self.noSplines):
            timeT = self.time[i-1]
            for k in [1,2,3,4]:
                poly0 = -1 * MinimumSnap.generatepolynomial(self.n_coeffs, order=k,t=0)
                polyT = MinimumSnap.generatepolynomial(self.n_coeffs, order=k,t=timeT)
                poly = np.hstack((polyT,poly0))
                # print(poly.shape)
                # print(self.A.shape)
                self.A[self.rowCounter, (i - 1) * self.n_coeffs:self.n_coeffs * (i + 1)] = np.hstack((polyT,poly0))

                self.rowCounter += 1


    def initMatrices(self):
        self.A = np.zeros((self.n_coeffs * self.noSplines, self.n_coeffs * self.noSplines))
        self.B = np.zeros((self.n_coeffs * self.noSplines, len(self.globalPath[0]))) # or len(self.globalPath[0])

    def numberSplines(self):
        self.noSplines = self.globalPath.shape[0] - 1
        # print(self.noSplines)


    def computeTime(self):
        vel = self.velocity
        for i in range(self.noSplines):
            dist = np.linalg.norm(self.globalPath[i+1] - self.globalPath[i])
            time1 = dist/self.velocity
            self.time.append(time1)

    def plot(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # tree = list(self.bestPath.values())  # Extract values from the dictionary
        # tree = self.bestPath
        tree = np.array(self.positions) 

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

        ######### TEMP SINGLE OBSTCALE PLOTTING CODE ########
        x_min, x_max = self.xmin, self.xmax
        y_min, y_max = self.ymin, self.ymax
        z_min, z_max = self.zmin, self.zmax

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




    @staticmethod
    def generatepolynomial(noCoeffs, order, t):
        poly = np.zeros(noCoeffs)
        deri = np.zeros(noCoeffs)

        for i in range(noCoeffs):
            poly[i] = 1
            deri[i] = i
        
        for _ in range(order):
            for j in range(noCoeffs):
                poly[j] = poly[j] * deri[j]
                
                if(deri[j] > 0):
                    deri[j] = deri[j] - 1
        
        for i in range(noCoeffs):
            poly[i] = poly[i] * t**deri[i]

        return poly.T

if __name__ == "__main__":

    #### Can be used for unit testing of the code ####
    global_path = [[0.9,1.9,0],
                   [2,0.75,0],
                   [3.25,2,0],
                   [10,10,0]]
    
    global_path = np.array(global_path)
    # print(global_path.shape[0])
    obs = [[1,3,1,3,0,0]]
    xmin, xmax = 1,3
    ymin, ymax = 1,3
    zmin, zmax = 0,0

    min_snap = MinimumSnap(global_path, obstacles=obs, velocity=2, dt=0.1)

    # global_trajectory = min_snap.get_trajectory()
    global_trajectory = min_snap.getTrajectory()  # long-term trajectory

    positions = min_snap.positions
    print(positions)
    min_snap.plot()

