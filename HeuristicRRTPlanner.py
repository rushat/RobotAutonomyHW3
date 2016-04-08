import numpy
from RRTTree import RRTTree
from SimpleEnvironment import SimpleEnvironment
import time

import pdb

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    def ChooseTarget(self,goal_config):
        goal_config = numpy.copy(goal_config)
        p = numpy.random.uniform(0.0,1.0)
        goal_p = 0.4 						# CHANGE THIS my defualt = 0.4
        config = self.planning_env.GenerateRandomConfiguration()

        #print "P: " + str(p)
        #print "G: " + str(goal_p)

        # Randomly try goal
        if p<goal_p:
            #print "HERE"
            return goal_config
        else:
            return config

    def Plan(self, start_config, goal_config, epsilon = 0.01):
        start_time = time.time()
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
        
        self.tree.AddEdge(self.tree.GetRootId, self.tree.GetRootId)
        
        q_nearest = start_config
        r = 2
        while(self.planning_env.ComputeDistanceRRT(q_nearest,goal_config) > epsilon):
            # Get Random Point to add to tree
            m_quality = 0
            
            while(r>m_quality) :
                q_target = self.ChooseTarget(goal_config)     
                
                #Find nearest neighbor to new point
                vid, q_nearest = self.tree.GetNearestVertex(q_target)
                if numpy.array_equal(q_target,goal_config):
                    break                
                g = self.planning_env.ComputeDistanceRRT(q_nearest,start_config)
                h = self.planning_env.ComputeHeuristicCostRRT(q_nearest,goal_config)


                near_cost = g+h
                opt_cost = self.planning_env.ComputeDistanceRRT(goal_config,start_config)
                if r==2:
                    max_cost = near_cost

                m_quality = 1 -(near_cost-opt_cost)/(max_cost - opt_cost)
                r = numpy.random.uniform(0.0,1.0)
                
                floor_val = 0.3					#CHANGE THis my default = 0.3
                if m_quality>floor_val:
                    m_quality=floor_val
            #print "m_quality: " + str(m_quality)
            #print "max_cost: " + str(max_cost)
            #pdb.set_trace()
            # Join goal to target if possible
            q_extended = self.planning_env.Extend(q_nearest,q_target)
            #q_connecting = self.planning_env.Extend(goal_config,q_target)


            if q_extended is not None:
                #print "QX: " + str(q_extended[0]) + " QY: " + str(q_extended[1]) 
                if numpy.array_equal(q_nearest,q_extended) == False:
                    self.tree.AddVertex(q_extended)
                    self.tree.AddEdge(vid,len(self.tree.vertices)-1)
                    g = self.planning_env.ComputeDistanceRRT(q_extended,start_config)
                    h = self.planning_env.ComputeHeuristicCostRRT(q_extended,goal_config)
                    new_cost = g+h
                    if new_cost > max_cost:
                        max_cost = new_cost
                    #pdb.set_trace()
                    if self.visualize: 
                        self.planning_env.PlotEdge(q_extended,q_nearest)
            
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
        
        print time.time() - start_time
        #plan.append(start_config)
        #plan.append(goal_config)
        path_length = 0
        if self.visualize: 
            for i in range(len(plan) - 1):
                self.planning_env.PlotRedEdge(plan[i],plan[i+1])
                #path_length = path_length + self.planning_env.ComputeDistanceRRT(plan[i],plan[i+1]) 
        if self.visualize: 
            for i in range(len(plan) - 1):
                path_length = path_length + self.planning_env.ComputeDistanceRRT(plan[i],plan[i+1])
        
        print "path length:" + str(path_length)
        return plan
