import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import pdb
import time

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

    def collision_check(self, config):

        robot_pose = numpy.array([[ 1, 0,  0, config[0]], 
                                    [ 0, 1,  0, config[1]], 
                                    [ 0, 0,  1, 0], 
                                    [ 0, 0,  0, 1]])
        
        self.robot.SetTransform(robot_pose)

        return self.env.CheckCollision(self.robot,self.table)


    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        increment_length = 0.01
        step_size = 2
        current_position = numpy.array([0,0])
        dist = self.ComputeDistanceRRT(start_config,end_config)
        unit_vector = (end_config - start_config)/dist
        increment_dist = unit_vector*increment_length
        interpolate_num = int(dist/increment_length)
        for i in range(interpolate_num):
            current_position = start_config + increment_dist*(i+1)
            robot_pose = numpy.array([[ 1, 0,  0, current_position[0]], 
                                  [ 0, 1,  0, current_position[1]], 
                                  [ 0, 0,  1, 0], 
                                  [ 0, 0,  0, 1]])
            self.robot.SetTransform(robot_pose)
            if self.env.CheckCollision(self.robot):
                #One can either move until obstacle (CONNECT), or just discard it (EXTEND)
                #return current_position - increment_dist
                return None
        
        #if numpy.linalg.norm(current_position - start_config)> step_size:
        #    return start_config + step_size*unit_vector
        #else:
        return end_config

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits = self.upper_limits
        upper_limits = self.lower_limits
        original_pose = self.robot.GetTransform()
        gen_random_config_flag = True
        
        while gen_random_config_flag == True:
            config = numpy.random.uniform(lower_limits, upper_limits,2)
            #print "Config 0: " + str(config[0]) + " Config 1: " + str(config[1])
            robot_pose = numpy.array([[ 1, 0,  0, config[0]], 
                                    [ 0, 1,  0, config[1]], 
                                    [ 0, 0,  1, 0], 
                                    [ 0, 0,  0, 1]])

            self.robot.SetTransform(robot_pose)
            if self.env.CheckCollision(self.robot) == False:
                gen_random_config_flag = False
        self.robot.SetTransform(original_pose)
        return config

    def ComputeDistanceRRT(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        dist = numpy.linalg.norm(start_config-end_config)
        return dist

    def ComputeHeuristicCostRRT(self, start_config, goal_config):
        
        cost = 0
        cost = numpy.linalg.norm(numpy.array(start_config) - numpy.array(goal_config))
        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids

        return cost
    def ShortenPath(self, path, timeout=15.0):

    # TODO: Implement a function which performs path shortening
    #  on the given path.  Terminate the shortening after the 
    #  given timout (in seconds).
    #
    #pdb.set_trace()
        start = time.time()
        while (time.time()-start)<15:
            increment_length = 0.1
            goal_config = path[-1]
            l = len(path)
            new_path = []
            new_path.append(path[0])
            for i in range(l):
                end_config = path[i+1]
                start_config = path[i] 
                dist = self.ComputeDistanceRRT(start_config,end_config)
                unit_vector = (end_config - start_config)/dist
                increment_dist = unit_vector*increment_length
                interpolate_num = int(dist/increment_length)
                new_path.append(path[i])
                for j in range(interpolate_num):
                    current_position = start_config + increment_dist*(j+1)
                    check = self.Extend(current_position,goal_config)
                    if check != None:
                        new_path.append(current_position)
                        new_path.append(goal_config)
                        break
                else:
                    continue
                break
            return new_path
        return path