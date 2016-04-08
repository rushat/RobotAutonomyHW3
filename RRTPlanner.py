import numpy
from RRTTree import RRTTree
import time

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        plan = []
        dist = []
        debug =False

        startNode = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goalNode = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        tree = RRTTree(self.planning_env, startNode)

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        plan.append(start_config)

        start = time.time()
        i = 0
        far = True
        while far:
            if i%5 == 0:
                randNode = goalNode
                if debug: print "rand is goal"
            else:
                if debug: print "rand is rand"
                randNode = self.planning_env.GenerateRandomNode()
            if debug: print ("i: " + str(i))
            if debug: print ("attempt node: " + str(randNode))
            i+=1
            nearID, nearNode= tree.GetNearestVertex(randNode)
            if debug: print ("nearNode " + str(nearNode))
            if debug: print ("randNode " + str(randNode))
            newNode = self.planning_env.Extend(nearNode, randNode)
            if newNode != None:
                if debug: print ("Found new node:" + str(newNode))
                addedID = tree.AddVertex(newNode)
                tree.AddEdge(nearID, addedID)
                dist.append(self.planning_env.ComputeDistance(nearNode, newNode))
                if self.visualize: self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(nearNode), self.planning_env.discrete_env.NodeIdToConfiguration(newNode))
                if debug: print ("new node: " + str(newNode))
                if debug: print ("new node dist to goal: " + str(self.planning_env.ComputeDistance(newNode, goalNode)))

                if self.planning_env.ComputeDistance(newNode, goalNode) < epsilon:
                    if debug: print ("nearNode: " + str(nearNode))
                    if debug: print ("newNode: " + str(newNode))
                    far = False

        revPlan = []


        goalID = max(tree.edges.keys())
        foundID = tree.edges[goalID]
        while foundID != 0:
            revPlan.append(self.planning_env.discrete_env.NodeIdToConfiguration(tree.vertices[foundID]))
            foundID = tree.edges[foundID]

        revPlan.reverse()
        plan.extend(revPlan)
        plan.append(goal_config)

        if self.visualize:
            for i in range(len(plan)-1):
                self.planning_env.PlotRedEdge(plan[i],plan[i+1])

        print ("total dist: " + str(sum(dist)))
        print ("time: " + str(time.time() - start))
        print ("number of vertices: " + str(len(plan)))

        return plan
