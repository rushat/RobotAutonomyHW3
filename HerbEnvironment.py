import numpy
from DiscreteEnvironment import DiscreteEnvironment
import random
class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.env = self.robot.GetEnv()
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(self.table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        self.table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def GetSuccessors(self, node_id):

        successors = []
        coord = [0]*self.discrete_env.dimension
        new_coord = [0]*self.discrete_env.dimension
        
        coord = self.discrete_env.NodeIdToGridCoord(node_id)
        
        for i in range(self.discrete_env.dimension):
            
            new_coord = list(coord)
            new_coord[i] = coord[i]-1
            new_config = self.discrete_env.GridCoordToConfiguration(new_coord)
            
            flag = True
            for j in range(len(new_config)):
                if (new_config[j] > self.upper_limits[j] or new_config[j] < self.lower_limits[j]):
                    flag = False

            if flag == True and not self.collision_check(self.discrete_env.GridCoordToConfiguration(new_coord)):
                successors.append(self.discrete_env.GridCoordToNodeId(new_coord))        

        for i in range(self.discrete_env.dimension):
            new_coord = list(coord)
            new_coord[i] = coord[i]+1

            new_config = self.discrete_env.GridCoordToConfiguration(new_coord)
            
            flag = True
            for j in range(len(new_config)):
                if (new_config[j] > self.upper_limits[j] or new_config[j] < self.lower_limits[j]):
                    flag = False

            if flag == True and not self.collision_check(self.discrete_env.GridCoordToConfiguration(new_coord)):
                successors.append(self.discrete_env.GridCoordToNodeId(new_coord))        

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        
        return successors
    def ComputeDistance(self, start_id, end_id):

        dist = 0
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        dist = numpy.linalg.norm(numpy.array(start_config)-numpy.array(end_config))

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
       
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        goal_config = self.discrete_env.NodeIdToConfiguration(goal_id)
        
        weights = numpy.array([4.,4.,4.,4.,1.,1.,1.])

        cost = numpy.linalg.norm(numpy.multiply((numpy.array(start_config) - numpy.array(goal_config)),weights))
        

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        return cost

    def collision_check(self, config):

        self.robot.SetDOFValues(config,self.robot.GetActiveDOFIndices(),checklimits=1)

        selfC = self.robot.CheckSelfCollision()
        env = self.robot.GetEnv()
        envC = env.CheckCollision(self.robot,self.table)
        if ((selfC) or (envC)):
            return True
        else:
            return False 
    def Extend(self, start_node, end_node):
        #
        # TODO: Implement a function which attempts to extend from
        #   a start configuration to a goal configuration
        #

        numPartitions = 10
        start_config = self.discrete_env.NodeIdToConfiguration(start_node)
        end_config = self.discrete_env.NodeIdToConfiguration(end_node)

        length = len(start_config)
        checkVals = [0] * length
        checkVals = numpy.copy(start_config)
        deltas = [0] * length
        indices = self.robot.GetActiveDOFIndices()

        for i in range(0,length):
            deltas[i] = (end_config[i]-start_config[i])/numPartitions

        tempTrans = self.robot.GetActiveDOFValues()

        for k in range(1,numPartitions):

            for i in range(0,length):
                checkVals[i]+=deltas[i]
                self.robot.SetDOFValues([checkVals[i]],[indices[i]])

            if self.robot.CheckSelfCollision() == True or self.robot.GetEnv().CheckCollision(self.robot, self.table) == True:
                for i in range(0,length):
                    checkVals[i]-=deltas[i]

                k = numPartitions+1

                if (checkVals==start_config).all():
                    return None
                else:
                    return  self.discrete_env.ConfigurationToNodeId(checkVals)
        return self.discrete_env.ConfigurationToNodeId(end_config)


    def GenerateRandomNode(self):
        #global table
        config = [0] * len(self.robot.GetActiveDOFIndices())

        #
        # TODO: Generate and return a random configuration
        #

        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        randDOF = [0] * len(self.robot.GetActiveDOFIndices())
        found = False
        tempDOF = self.robot.GetActiveDOFValues()
        indices = self.robot.GetActiveDOFIndices()
        while found == False:
            for i in range(0,len(config)):
                randDOF[i]=round(random.uniform(lower_limits[i], upper_limits[i]),3)
                self.robot.SetDOFValues([randDOF[i]],[indices[i]])
            if self.robot.CheckSelfCollision() == False & self.robot.GetEnv().CheckCollision(self.robot, self.table) == False:
                found = True
                config = randDOF
            for i in range(0,len(config)):
                self.robot.SetDOFValues([tempDOF[i]],[indices[i]])
        return self.discrete_env.ConfigurationToNodeId(numpy.array(config))