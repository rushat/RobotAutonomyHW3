from heapq import heappush, heappop
import pdb
class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):

    #     plan = []
        
    #     # TODO: Here you will implement the AStar planner
    #     #  The return path should be a numpy array
    #     #  of dimension k x n where k is the number of waypoints
    #     #  and n is the dimension of the robots configuration space
    #     if self.visualize and hasattr(self.planning_env, "InitializePlot"):
				# self.planning_env.InitializePlot(goal_config)

        

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        openlist = []
        visited ={}
        dist = 0 #self.planning_env.ComputeDistance(start_id,start_id)
        hrstc = self.planning_env.ComputeHeuristicCost(start_id,goal_id)
        cost = dist + hrstc
        heappush(openlist, (cost, dist, hrstc, start_id, None))
        found = False
        backpointer = {}
        while not found:
            curr = heappop(openlist)
            old_dist = curr[1]
            curr_id = curr[3]
            
            if curr_id == goal_id:
                backpointer[curr_id] = curr[4]
                found = True
                break

            if curr_id in visited:
                continue
            #backpointer[curr_id] =  
            visited[curr_id] = 1
            backpointer[curr_id] = curr[4]

            successors = self.planning_env.GetSuccessors(curr_id)
            for successor in successors:
                if successor not in visited:
                    dist = old_dist +  self.planning_env.ComputeDistance(curr_id, successor)
                    hrstc = self.planning_env.ComputeHeuristicCost(successor,goal_id)
                    cost = dist + hrstc
                    temp_tuple = (cost,hrstc,dist,successor,curr_id)
                    heappush(openlist, temp_tuple)

                  
            





    #     h = [] 				 # open list
    #     start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
    #     goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
    #     found = False
        
    #     dist = self.planning_env.ComputeDistance(start_id,start_id)
    #     hrstc = self.planning_env.ComputeHeuristicCost(start_id,goal_id)
    #     cost = dist + hrstc
    #     heapq.heappush(h, (cost, hrstc,dist, start_id,None))


    #     #explored =[start_id]
    #     backtrack = {}
    #     backtrack[start_id] = None      # backpointer updates only if its inopen and not in visited
    #     visited = {}
    #     visited[start_id] = cost  		# closed or visited list...updates when popped
    #     #check[start_id] = cost
    #     while not found:
    #     	curr_b = heapq.heappop(h)
    #     	old_dist = curr_b[2]
    #     	curr_id = curr_b[3]
    #     	#backtrack[curr_id] = curr_b[4]
    #     	# if curr_id == goal_id:
    #     	# 	found = True
    #     	# 	break
        	
    #     	# if curr_id not in visited.keys():
    #     	# 	backtrack[curr_id] = curr_b[4]

    #     	#print curr_id
        	
    #     	successors = self.planning_env.GetSuccessors(curr_id)
    #     	#pdb.set_trace()
    #     	for successor in successors:
    #     		if successor not in visited:
				# 	visited[successor] = 1
				# 	#print successor
				# 	dist = old_dist +  self.planning_env.ComputeDistance(curr_id, successor)
				# 	hrstc = self.planning_env.ComputeHeuristicCost(successor,goal_id)
				# 	cost = dist + hrstc
				# 	temp_tuple = (cost,hrstc,dist,successor,curr_id)
				# 	#pdb.set_trace()
				# 	heapq.heappush(h, temp_tuple)
				# 	#print self.visualize
				# 	backtrack[successor] = curr_id
				# 	if self.visualize: 
				# 		#print "entered"
				# 		s = self.planning_env.discrete_env.NodeIdToConfiguration(successor)
				# 		c = self.planning_env.discrete_env.NodeIdToConfiguration(curr_id)
				# 		self.planning_env.PlotEdge(c,s)
				# 	#self.planning_env.PlotEdge(q_nearest,q_extended)
        # Shortest Path
        path = []
        path.append(self.planning_env.discrete_env.NodeIdToConfiguration(goal_id))
        element = backpointer[goal_id]
        while element is not None:
            path.append(self.planning_env.discrete_env.NodeIdToConfiguration(element))
            element = backpointer[element]

        plan = path[::-1]
        print plan
        return plan