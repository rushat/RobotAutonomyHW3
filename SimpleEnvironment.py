import numpy
import pylab as pl
import random
import time
from DiscreteEnvironment import DiscreteEnvironment

import pdb

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

    	# - ----------------------4 CONNECTED VERSION ------------------------
        # successors = []
        # coord = [0]*self.discrete_env.dimension
        # new_coord = [0]*self.discrete_env.dimension
        
        # coord = self.discrete_env.NodeIdToGridCoord(node_id)
        
        # for i in range(self.discrete_env.dimension):
            
        #     new_coord = list(coord)
        #     new_coord[i] = coord[i]-1
        #     new_config = self.discrete_env.GridCoordToConfiguration(new_coord)
            
        #     flag = True
        #     for j in range(len(new_config)):
        #         if (new_config[j] > self.upper_limits[j] or new_config[j] < self.lower_limits[j]):
        #             flag = False

        #     if flag == True and not self.collision_check(self.discrete_env.GridCoordToConfiguration(new_coord)):
        #         successors.append(self.discrete_env.GridCoordToNodeId(new_coord))        

        # for i in range(self.discrete_env.dimension):
        #     new_coord = list(coord)
        #     new_coord[i] = coord[i]+1

        #     new_config = self.discrete_env.GridCoordToConfiguration(new_coord)
            
        #     flag = True
        #     for j in range(len(new_config)):
        #         if (new_config[j] > self.upper_limits[j] or new_config[j] < self.lower_limits[j]):
        #             flag = False

        #     if flag == True and not self.collision_check(self.discrete_env.GridCoordToConfiguration(new_coord)):
        #         successors.append(self.discrete_env.GridCoordToNodeId(new_coord))        

        # # TODO: Here you will implement a function that looks
        # #  up the configuration associated with the particular node_id
        # #  and return a list of node_ids that represent the neighboring
        # #  nodes
        
        # return successors

        #----------------------------8 CONNECTED VERSION ----------------------------------- 

        successors = []
        coord = [0]*2
        new_coord = [0]*2

        coord = self.discrete_env.NodeIdToGridCoord(node_id)
        steps = [[0,1],[0,-1],[-1,0],[1,0],[1,1],[1,-1],[-1,1],[-1,-1]]
        #print node_id
        #print coord
        #print self.discrete_env.GridCoordToNodeId([-0.05,0.05])
        #print self.discrete_env.GridCoordToConfiguration(coord)
        for step in steps:
            new_coord = [coord[0] + step[0] ,coord[1] + step[1]]
            new_config = self.discrete_env.GridCoordToConfiguration(new_coord)
            #print new_config
            flag = True
            for j in range(len(new_config)):
                if (new_config[j] > self.upper_limits[j] or new_config[j] < self.lower_limits[j]):
                    flag = False

            if flag == True and not self.collision_check(self.discrete_env.GridCoordToConfiguration(new_coord)):
                successors.append(self.discrete_env.GridCoordToNodeId(new_coord))

        #print successors
        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        #print successors
        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)

        dist = numpy.linalg.norm(numpy.array(start_config) - numpy.array(end_config))

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        goal_config = self.discrete_env.NodeIdToConfiguration(goal_id)
        cost = numpy.linalg.norm(numpy.array(start_config) - numpy.array(goal_config))
        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids

        return cost

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')
        print "InitializePlot"
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

    def PlotRedEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'r.-', linewidth=2.5)
        pl.draw()

    def PlotCyanEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'c.-', linewidth=2.5)
        pl.draw()

    def collision_check(self, config):

        robot_pose = numpy.array([[ 1, 0,  0, config[0]],
                                    [ 0, 1,  0, config[1]],
                                    [ 0, 0,  1, 0],
                                    [ 0, 0,  0, 1]])

        self.robot.SetTransform(robot_pose)

        return self.env.CheckCollision(self.robot,self.table)



    def Extend(self, start_node, end_node):

        numPartitions = 1000

        start_config = self.discrete_env.NodeIdToConfiguration(start_node)
        end_config = self.discrete_env.NodeIdToConfiguration(end_node)

        x = start_config[0]
        y = start_config[1]

        deltaX = (end_config[0]-x)/numPartitions
        deltaY = (end_config[1]-y)/numPartitions


        tempTrans = self.robot.GetTransform()

        for i in range(1,numPartitions):
            x += deltaX
            y += deltaY
            #print i
            self.robot.SetTransform(numpy.array([[1, 0, 0, x],
                                              [0, 1, 0, y],
                                              [0, 0, 1, 0],
                                              [0, 0, 0, 1]]))
            if self.robot.GetEnv().CheckCollision(self.robot, self.table) == True:
                x -= deltaX
                y -= deltaY
                i = numPartitions+1
                if (x == start_config[0]) & (y == start_config[1]):
                    return None
                else:
                    return self.discrete_env.ConfigurationToNodeId([x, y])

        end_node = self.discrete_env.ConfigurationToNodeId(end_config)
        return end_node


    def GenerateRandomNode(self):
        config = [0] * 2

        #
        # TODO: Generate and return a random configuration
        #

        found = False

        while found == False:
                x = round(random.uniform(self.lower_limits[0], self.upper_limits[0]),3)
                y = round(random.uniform(self.lower_limits[1], self.upper_limits[1]),3)
                tempTrans = self.robot.GetTransform()
                self.robot.SetTransform(numpy.array([[1, 0, 0, x],
                                                  [0, 1, 0, y],
                                                  [0, 0, 1, 0],
                                                  [0, 0, 0, 1]]))
                if self.robot.GetEnv().CheckCollision(self.robot, self.table) == False:
                    found = True
                    config = [x,y]
                self.robot.SetTransform(tempTrans)

        return self.discrete_env.ConfigurationToNodeId(numpy.array(config))


    def ShortenPath(self, path, timeout=5.0):
        print "shortening path"

        start = time.time()
        while (time.time()-start)<5:
            increment_length = 0.1
            goal_config = path[-1]
            l = len(path)
            new_path = []
            new_path.append(path[0])
            for i in range(l):
                end_config = path[i+1]
                start_config = path[i]
                dist = self.ComputeDistance(self.discrete_env.ConfigurationToNodeId(start_config),self.discrete_env.ConfigurationToNodeId(end_config))
                unit_vector = (end_config - start_config)/dist
                increment_dist = unit_vector*increment_length
                interpolate_num = int(dist/increment_length)
                new_path.append(path[i])
                for j in range(interpolate_num):
                    current_position = start_config + increment_dist*(j+1)
                    check = self.Extend(self.discrete_env.ConfigurationToNodeId(current_position),self.discrete_env.ConfigurationToNodeId(goal_config))
                    if check != None:
                        new_path.append(current_position)
                        new_path.append(goal_config)
                        break
                else:
                    continue
                break
            dist = []
            for i in range(len(new_path)-1):
                dist.append(self.ComputeDistance(self.discrete_env.ConfigurationToNodeId(new_path[i]),self.discrete_env.ConfigurationToNodeId(new_path[i+1])))
                self.PlotCyanEdge(new_path[i],new_path[i+1])
            print("shortened distance: " + str(sum(dist)))
            print("time to shorten: " + str(time.time() - start))
            print("new number of vertices: " + str(len(path)))
            return new_path

        dist = []
        for i in range(len(path)-1):
            dist.append(self.ComputeDistance(self.discrete_env.ConfigurationToNodeId(path[i]),self.discrete_env.ConfigurationToNodeId(path[i+1])))
            self.PlotCyanEdge(path[i],path[i+1])
        print("shortened distance: " + str(sum(dist)))
        print("time to shorten: " + str(time.time() - start))
        print("new number of vertices: " + str(len(path)))
        return path




        return path
