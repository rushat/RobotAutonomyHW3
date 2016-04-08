import numpy
from RRTTree import RRTTree
from SimpleEnvironment import SimpleEnvironment
from time import time
import pdb

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def ChooseTarget(self,goal_config):
        goal_config = numpy.copy(goal_config)
        p = numpy.random.uniform(0.0,1.0)
        goal_p = 0.2
        config = self.planning_env.GenerateRandomConfiguration()

        #print "P: " + str(p)
        #print "G: " + str(goal_p)

        # Randomly try goal
        if p<goal_p:
            #print "HERE"
            return goal_config
        else:
            return config




    def Plan(self, start_config, goal_config, epsilon = 0.001):
        start_time = time()
        start_config = numpy.copy(start_config)
        goal_config = numpy.copy(goal_config)

        self.tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        #plan.append(start_config)
        #plan.append(goal_config)
        self.tree.AddEdge(self.tree.GetRootId, self.tree.GetRootId)
        q_nearest = start_config

  
        while(self.planning_env.ComputeDistanceRRT(q_nearest,goal_config) > epsilon):
            # Get Random Point to add to tree
            q_target = self.ChooseTarget(goal_config)

            #Find nearest neighbor to new point
            vid, q_nearest = self.tree.GetNearestVertex(q_target)

            #pdb.set_trace()
            
            # Join goal to target if possible
            q_extended = self.planning_env.Extend(q_nearest,q_target)
            #q_connecting = self.planning_env.Extend(goal_config,q_target)

            if q_extended is not None:
                #print "QX: " + str(q_extended[0]) + " QY: " + str(q_extended[1]) 
                if numpy.array_equal(q_nearest,q_extended) == False:
                    self.tree.AddVertex(q_extended)
                    self.tree.AddEdge(vid,len(self.tree.vertices)-1)
                    if self.visualize: 
                        self.planning_env.PlotEdge(q_extended,q_nearest)

                    #self.planning_env.PlotEdge(q_nearest,q_extended)
            
            if numpy.array_equal(q_extended, goal_config):
                goal_index = len(self.tree.vertices)-1
                break
            
        current_index = goal_index
        while current_index != 0:
            plan.append(self.tree.vertices[current_index])
            current_index = self.tree.edges[current_index]
        plan.append(self.tree.vertices[current_index])
        plan = plan[::-1]
        #pdb.set_trace()
        #print "Tree Size: " + str(len(self.tree.vertices))
        plan = self.planning_env.ShortenPath(plan)
        l = time() - start_time
        print "time:" + str(l)
        path_length = 0
        if self.visualize: 
            for i in range(len(plan) - 1):
                self.planning_env.PlotRedEdge(plan[i],plan[i+1])
                path_length = path_length + self.planning_env.ComputeDistanceRRT(plan[i],plan[i+1])
        if self.visualize: 
            for i in range(len(plan) - 1):
                path_length = path_length + self.planning_env.ComputeDistanceRRT(plan[i],plan[i+1])
        
        print "path length:" + str(path_length)
        return plan
