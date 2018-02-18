import numpy
import matplotlib.pyplot as pl

class SimpleEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
#    def CollisionCheck(self,point):
#        #for b in self.robot.GetEnv().GetBodies():
#        #    if b.GetName() == self.robot.GetName():
#        #        continue
#        if (point[0] <= 1 and point[0] >= -1 and point[1] <= 1 and point[1] >=-1):
#            return False
#        else:
#            return True
            

    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        #
        # TODO: Generate and return a random configuration
        #
        
        while True:
            for dim in range(2):
                config[dim] = numpy.random.uniform(lower_limits[dim],upper_limits[dim])
            #if (CollisionCheck(self,config)):
            if not (config[0] <= 1.5 and config[0] >= 0.5 and config[1] <= 1 and config[1] >=-1):
            	break
            #else:
                #print config[0]
                #print config[1]
                #print("Config %.2f , %.2f is not viable", % (config[0],config[1]) )
        #print("Random config is %.2f , %.2f", % (config[0],config[1]) )
        return numpy.array(config)

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        distSquare = numpy.zeros(len(start_config))
        for dimension in range(len(start_config)):
            distSquare[dimension] = numpy.power(end_config[dimension] - start_config[dimension],2)
        distance = numpy.sum(distSquare)
        return distance

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        MAX_DIST = 1
        dist = self.ComputeDistance(start_config, end_config)
        #print("     extend toward: {0:1.2f}, {1:1.2f}".format(end_config[0], end_config[1]) )
        if (dist > MAX_DIST):
            end_config[0] = start_config[0] + (MAX_DIST/dist)*(end_config[0] - start_config[0])
            end_config[1] = start_config[1] + (MAX_DIST/dist)*(end_config[1] - start_config[1])
        #print("From configuration: {0:1.2f}, {1:1.2f}".format(start_config[0], start_config[1]) )
        #print("  to configuration: {0:1.2f}, {1:1.2f}".format(end_config[0], end_config[1]) )
        xdim = [0]*10
        ydim = [0]*10
        for point in range(10):
            xdim[point] = start_config[0] + (point/10) * (end_config[0] - start_config[0])
            ydim[point] = start_config[1] + (point/10) * (end_config[1] - start_config[1])
            #if (not CollisionCheck(self,point)):
            if (xdim[point] <= 1.5 and xdim[point] >= 0.5 and ydim[point] <= 1 and ydim[point] >=-1):
                return None
        return end_config

    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        return path


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        #print('Drawing edge')
        pl.draw()

