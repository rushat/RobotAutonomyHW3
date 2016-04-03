import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.env = self.robot.GetEnv()
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(self.table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        self.table.SetTransform(table_pose)

    def GetSuccessors(self, node_id):

        successors = []
        coord = [0]*2
        new_coord = [0]*2
        
        coord = self.discrete_env.NodeIdToGridCoord(node_id)
        steps = [[0,1],[0,-1],[-1,0],[1,0]]
        #print node_id
        #print coord
        #print self.discrete_env.GridCoordToNodeId([-0.05,0.05])
        #print self.discrete_env.GridCoordToConfiguration(coord)
        for step in steps:
            new_coord = [coord[0] + step[0] ,coord[1] + step[1]]
            #print new_coord
            new_config = self.discrete_env.GridCoordToConfiguration(new_coord)
            #print new_config
            if not new_config > self.upper_limits or new_config < self.lower_limits:
                if not self.collision_check(self.discrete_env.GridCoordToConfiguration(new_coord)):
                    successors.append(self.discrete_env.GridCoordToNodeId(new_coord))

        #print successors
        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        
        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        dist = numpy.linalg.norm(start_config-end_config)

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        goal_config = self.discrete_env.NodeIdToConfiguration(goal_id)
        cost = numpy.linalg.norm(start_config - goal_config)
        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids

        return cost

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
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

    def collision_check(self, config):

        robot_pose = numpy.array([[ 1, 0,  0, config[0]], 
                                    [ 0, 1,  0, config[1]], 
                                    [ 0, 0,  1, 0], 
                                    [ 0, 0,  0, 1]])
        
        self.robot.SetTransform(robot_pose)

        return self.env.CheckCollision(self.robot,self.table)