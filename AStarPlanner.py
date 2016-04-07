from heapq import heappush, heappop
import time
import pdb
class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):
    	start_time = time.time()
        plan = []
        
    # TODO: Here you will implement the AStar planner
    #  The return path should be a numpy array
    #  of dimension k x n where k is the number of waypoints
    #  and n is the dimension of the robots configuration space
        if self.visualize and hasattr(self.planning_env, "InitializePlot"):
            self.planning_env.InitializePlot(goal_config)


        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        openlist = []
        visited ={}
        dist = 0 
        hrstc = self.planning_env.ComputeHeuristicCost(start_id,goal_id)
        cost = dist + hrstc
        heappush(openlist, (cost, dist, hrstc, start_id, None))
        found = False
        backpointer = {}
        n=0
        while not found:
            curr = heappop(openlist)
            old_dist = curr[1]
            curr_id = curr[3]
            if curr_id == goal_id:
                backpointer[curr_id] = curr[4]
                found = True
                path_length = old_dist
                break
            n=n+1
            if curr_id in visited:
                continue
                
            visited[curr_id] = 1
            backpointer[curr_id] = curr[4]
            #pdb.set_trace()
            successors = self.planning_env.GetSuccessors(curr_id)
            for successor in successors:
                if successor not in visited:
                    dist = old_dist +  self.planning_env.ComputeDistance(curr_id, successor)
                    hrstc = self.planning_env.ComputeHeuristicCost(successor,goal_id)
                    cost = dist + hrstc
                    temp_tuple = (cost,dist,hrstc,successor,curr_id)
                    heappush(openlist, temp_tuple)
                    if self.visualize: 
                        s = self.planning_env.discrete_env.NodeIdToConfiguration(successor)
                        c = self.planning_env.discrete_env.NodeIdToConfiguration(curr_id)
                        self.planning_env.PlotEdge(c,s)

        # Shortest Path
        path = []
        path.append(self.planning_env.discrete_env.NodeIdToConfiguration(goal_id))
        element = backpointer[goal_id]
        while element is not None:
            path.append(self.planning_env.discrete_env.NodeIdToConfiguration(element))
            element = backpointer[element]

        plan = path[::-1]
        print "number of nodes"
        print n
        print "time (in seconds):" 
        print time.time()- start_time
        print "path path_length"
        print path_length
        #print plan
        return plan