import numpy
import pylab as pl
import random
import IPython

class SimpleEnvironment(object):

    def __init__(self, simpleRobot):
        self.robot = simpleRobot.robot
        self.simpleBot = simpleRobot

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

    def GenerateRandomConfiguration(self):
        lower_limits, upper_limits = self.boundary_limits

	found = False
	while found == False:
            xPoint = random.uniform(lower_limits[0], upper_limits[0])
            yPoint = random.uniform(lower_limits[1], upper_limits[1])
            if not self.Collides([xPoint,yPoint]):
            	found = True

        print("x = "+str(xPoint) + " y = "+str(yPoint));

        config = [xPoint, yPoint]
        pl.plot(config, 'bx')

        return numpy.array(config)

    def ComputeDistance(self, start_config, end_config):
	x1 = start_config[0]
	x2 = end_config[0]
	y1 = start_config[1]
	y2 = end_config[1]
	return numpy.sqrt(((x2-x1)**2)+((y2-y1)**2))

    def Extend(self, start_config, end_config):
	travel_limit = 0.01 # maximum distance allowed to travel towards goal
	num_checks = 10 # number of collision checks along the way
	dist = self.ComputeDistance(start_config, end_config)

	travel_dist = min(travel_limit, dist)
	
	x0 = start_config[0]
	y0 = start_config[1]
	x1 = end_config[0]
	y1 = end_config[1]
	xg = x0 + ( (travel_dist/dist)*(x1-x0) ) # xg and yg limited by travel_limit
	yg = y0 + ( (travel_dist/dist)*(y1-y0) )

	# incrementally step along line from start to goal, checking for collisions
	for i in range(0, num_checks-1):
	    config[0] = x0 + ( (i/(num_checks-1)) * (xg-x0) )
	    config[1] = y0 + ( (i/(num_checks-1)) * (yg-y0) )
	    if self.Collides(config):
		return None
	
	return numpy.array([xg, yg])
	

    def ShortenPath(self, path, timeout=5.0):
        #
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the
        #  given timout (in seconds).
        #
        return path

    def Collides(self, point):
	#import IPython
        #IPython.embed()
	T = self.robot.GetTransform()
	Tnew = T
	Tnew[[0,1],3] = point
	env = self.robot.GetEnv()
	
	# move robot to potential point, return collision between robot and table
	self.robot.SetTransform(Tnew)
	return env.CheckCollision(self.robot, env.GetKinBody('conference_table'))



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
        pl.draw()
