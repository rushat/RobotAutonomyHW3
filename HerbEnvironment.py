import numpy
from DiscreteEnvironment import DiscreteEnvironment
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

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())

        #
        # TODO: Generate and return a random configuration
        #

        lowerLimits,upperLimits = self.robot.GetDOFLimits()
        activeDOFIndices = self.robot.GetActiveDOFIndices()

        collision = True
        counter = 1
        maxIter = 100
        while collision: #Repeat until no collision with env
            config = numpy.array(numpy.random.rand(len(config))) #Generate random config between 0-1
            for i in activeDOFIndices:  #Scale to be within limits
                config[i] = (upperLimits[i] - lowerLimits[i])*config[i] + lowerLimits[i]

            collision = self.checkCollision(config)
            counter += 1
            if counter > maxIter:
                raise Exception('GenerateRandomConfiguration','MaxIter')
        #print "Random Config = "+str(config)
        return numpy.array(config)

    def ComputeDistanceRRT(self, start_config, end_config):

        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        dist = 0
        for i in range(0,len(start_config)):
           dist += numpy.power(end_config[i]-start_config[i],2)
        dist = numpy.power(dist,.5)
        return dist

    def Extend(self, start_config, end_config, dE = .01):
        #
        # TODO: Implement a function which attempts to extend from
        #   a start configuration to a goal configuration
        #
        # -------------------Dan's Code-------------------
        #initialize variables
        config = list(start_config)
        s = list(start_config)
        e = list(end_config)
        #print str(start_config)+"-->"+str(end_config)
        for t in numpy.arange(dE,1,dE):
             #Generate new config
             for i in range(0,len(s)):
                 config[i] = (e[i]-s[i])*t + s[i]
                 #print "t = "+str(t)+" config = "+str(config)
             #Check for collision
             if self.checkCollision(config):
                 return None
        return end_config

        #-------------------My Code------------------------
        # increment_length = 0.01
        # step_size = 0.3
        
        # current_position = numpy.array([0,0,0,0,0,0,0])

        # dist = self.ComputeDistanceRRT(start_config,end_config)
        # unit_vector = (end_config - start_config)/dist
        # increment_dist = unit_vector*increment_length
        # interpolate_num = int(dist/increment_length)

        # for i in range(interpolate_num):
            
        #     config = start_config + increment_dist*(i+1)
        #     #pdb.set_trace()
        #     self.robot.SetDOFValues(config,self.robot.GetActiveDOFIndices(),checklimits=1)

        #     selfC = self.robot.CheckSelfCollision()
        #     env = self.robot.GetEnv()
        #     envC = env.CheckCollision(self.robot,self.table)

        #     if ((selfC) or (envC)):
        #         #One can either move until obstacle (CONNECT), or just discard it (EXTEND)
        #         #return current_position - increment_dist
        #         self.robot.SetDOFValues(start_config,self.robot.GetActiveDOFIndices(),checklimits=1)
        #         return None
        # #if numpy.linalg.norm(current_position - start_config)> step_size:
        # #    return start_config + step_size*unit_vector
        # #else:
        # return end_config

    def toFullConfig(self,config):
        activeDOFIndices = self.robot.GetActiveDOFIndices()
        fullConfig = self.robot.GetDOFValues()
        ind = 0
        for i in activeDOFIndices:
            fullConfig[i] = config[ind]
            ind += 1
        return numpy.array(fullConfig)

    def checkCollision(self,config):
        fullConfig = self.toFullConfig(config)
        self.robot.SetDOFValues(fullConfig) #Move robot
        return self.robot.GetEnv().CheckCollision(self.robot) or self.robot.CheckSelfCollision()

    def ShortenPath(self, path, timeout=5.0, dE = .1):
        #
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the
        #  given timout (in seconds).
        #
        SuccessFlag = 0
        odist = self.totalPathDistance(path)
        print "Orig Total Path Distance = " + str(odist)
        maxTime = time.time() + timeout
        while time.time() < maxTime:
        #for k in range(0,10):
            i = 0
            while i <= len(path)-2:
                #print "i="+str(i)
            #for i in range(0,len(path)-2):
                s = numpy.array(path[i]) #Start point
                n = numpy.array(path[i+1]) #Start point + 1
                if list(s)==list(n):
                    #print "merging points"
                    del path[i+1]
                    SuccessFlag = 1
                else:
                    j = len(path)
                    while j > i:
                        j -= 1
                    #for j in range(len(path)-1,i+1,-1):
                        x = numpy.array(s) #interpolated between s and n
                        e = numpy.array(path[j]) #End point
                        #print "path length="+str(len(path))+" s="+str(s)+" n="+str(n)+"x="+str(x)+"e="+str(e)
                        #print "path="+str(path)
                        #Interpolate between s and n
                        for t in numpy.arange(0,1,dE):
                            x = numpy.array((n-s)*t+s)
                            #print "t="+str(t)+" x="+str(x)
                            if self.Extend(x,e) != None: #Try to connect
                                if t == 0:
                                    #print "Shortening"
                                    path[i+1:j] = [] #Success - delete points between
                                else:
                                    if j-i+2 > 2:
                                        #print "Adding "+str(x)
                                        path.insert(i+1,x) #Add new point
                                        path[i+2:j] = [] #Success - delete points between
                                        SuccessFlag = 1
                                break
                        if SuccessFlag == 1:
                            break
                #SuccessFlag = 0
                if SuccessFlag == 1:
                    SuccessFlag = 0
                i += 1
            SuccessFlag = 0
        ndist = self.totalPathDistance(path)
        print "New Total Path Distance = " + str(ndist)
        print "Amount Shortened = " + str(odist-ndist)
        return path

    def totalPathDistance(self,path):
        dist = 0
        for i in range(1,len(path)-1):
            dist +=self.ComputeDistanceRRT(path[i-1],path[i])
        return dist

    def ComputeHeuristicCostRRT(self, start_config, goal_config):
        
        cost = 0
        # start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        # goal_config = self.discrete_env.NodeIdToConfiguration(goal_id)
        
        weights = numpy.array([4.,4.,4.,4.,1.,1.,1.])

        cost = numpy.linalg.norm(numpy.multiply((numpy.array(start_config) - numpy.array(goal_config)),weights))
        

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        return cost