import numpy
import time
import random
from RRTTree import RRTTree

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):

        plan = []
        dist = []
        debug =False
        maxCost = 0

        startNode = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goalNode = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        tree = RRTTree(self.planning_env, startNode)

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        plan.append(start_config)

        start = time.time()
        i = 1
        far = True
        while far:
            if i%5 == 0:
                randNode = goalNode
                if debug: print "rand is goal"
                nearID, nearNode= tree.GetNearestVertex(randNode)
            else:
                if debug: print "rand is rand"
                while True:
                    randNode = self.planning_env.GenerateRandomNode()
                    nearID, nearNode= tree.GetNearestVertex(randNode)

                    nearCost = self.planning_env.ComputeDistance(startNode,nearNode)
                    optCost = self.planning_env.ComputeHeuristicCost(nearNode, goalNode)

                    quality = 1-((nearCost-optCost)/(maxCost - optCost))
                    r = round(random.uniform(0, 2), 3) # 3 decimals between 0 and 2
                    if debug: print ("nearCost: " + str(nearCost) + ", optCost: " + str(optCost))
                    if debug: print ("quality: " + str(quality) + ", r: " + str(r))

                    if r > quality:
                        break;

            if debug: print ("i: " + str(i))
            if debug: print ("attempt node: " + str(randNode))
            i+=1

            if debug: print ("nearNode " + str(nearNode))
            if debug: print ("randNode " + str(randNode))


            newNode = self.planning_env.Extend(nearNode, randNode)

            if newNode != None:
                if debug: print ("Found new node:" + str(newNode))
                if debug: print ("max cost:" + str(maxCost))
                newCost = self.planning_env.ComputeDistance(startNode,newNode)
                maxCost = max(maxCost, newCost)
                if debug: print ("new cost:" + str(newCost) + ", max cost:" + str(maxCost))
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
