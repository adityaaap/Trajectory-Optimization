import numpy as np

class MinimumSnap:
    def __init__(self, globalPath, obstacles, velocity, dt):
        self.globalPath = globalPath
        self.obstacles = obstacles
        self.velocity = velocity
        self.dt = dt
        self.n_coeffs = 8

        self.time = []
        self.A = None
        self.B = None
        self.coeffs = None
        self.noSplines = None

        self.rowCounter = 0
        self.completeTraj = None

    def getTrajectory(self):
        self.generateTrajectory()
        return self.completeTraj
    
    def generateTrajectroy(self):
        self.computeSplineParameters()

    def computeSplineParameters(self):
        self.createPolyMatrices()
    

    def createPolyMatrices(self):
        self.setup()
        self.positionConstraints()
        self.startGoalConstraints()
        self.continuityConstraints()

    
    def setup(self):
        self.initMatrices()
        self.numberSplines()
        self.computeTime()
        
    def positionConstraints(self):
        # waypoint constraints at t=0 and t=T

        # at t=0
        for i in range(self.noSplines):
            poly = MinimumSnap.generatepolynomial(self.n_coeffs, order=0, t=0)
            self.A[self.rowCounter,(self.noSplines - 1)*self.n_coeffs:self.noSplines*self.n_coeffs] = poly
            self.B[self.rowCounter,:]= self.globalPath[i]
            self.rowCounter += 1

        # at t=T
        for i in range(self.noSplines):
            tT = self.time[i]
            poly = MinimumSnap.generatepolynomial(self.n_coeffs, order=0, t=tT)
            self.A[self.rowCounter,(self.noSplines - 1)*self.n_coeffs:self.noSplines*self.n_coeffs] = poly
            self.B[self.rowCounter,:]= self.globalPath[i+1]
            self.rowCounter += 1

    def startGoalConstraints(self):
        for k in [1,2,3]:
            poly = MinimumSnap.generatepolynomial(self.n_coeffs, k, 0)
            self.A[self.rowCounter,0:self.n_coeffs] = poly
            self.rowCounter += 1

        for k in [1,2,3]:
            poly = MinimumSnap.generatepolynomial(self.n_coeffs, k, t=self.time[-1])
            self.A[self.rowCounter, (self.noSplines-1) * self.n_coeffs: self.noSplines * self.n_coeffs] = poly
            self.rowCounter += 1

    
    def continuityConstraints(self):
        for i in range(self.noSplines):
            time = self.time[i-1]
            for k in [1,2,3,4]:
                poly0 = MinimumSnap.generatepolynomial(self.n_coeffs, order=k,t=0)
                polyT = MinimumSnap.generatepolynomial(self.n_coeffs, order=k,t=time)
                
                self.A[self.rowCounter, (i-1) * self.n_coeffs: (i+1) * self.n_coeffs] = np.hstack(polyT,poly0)
                self.rowCounter += 1


    def initMatrices(self):
        self.A = np.zeros((self.n_coeffs * self.noSplines, self.n_coeffs * self.noSplines))
        self.B = np.zeros((self.n_coeffs * self.noSplines, self.globalPath.shape[1])) # or len(self.globalPath[0])

    def numberSplines(self):
        self.noSplines = self.globalPath.shape[0] - 1


    def computeTime(self):
        vel = self.velocity
        for i in range(self.noSplines):
            dist = np.linalg.norm(self.globalPath[i+1] - self.globalPath[i])
            self.time.append(dist/vel)




    @staticmethod
    def generatepolynomial(noCoeffs, order, t):
        poly = np.zeros(noCoeffs)
        deri = np.zeros(noCoeffs)

        for i in range(noCoeffs):
            poly[i] = i
            deri[i] = 1
        
        for i in range(order):
            for j in range(noCoeffs):
                poly[i] = poly[i] * deri[i]
                if(deri[i] > 0):
                    deri[i] = deri[i] - 1
        
        for i in range(noCoeffs):
            poly[i] = poly[i] * t**deri[i]

        return poly.T

    ## More static methods may be required for checking collision
