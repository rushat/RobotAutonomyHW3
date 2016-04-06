
import heapq
class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):

        plan = []
        
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        h = []
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        found = False
        
        dist = self.planning_env.ComputeDistance(start_id,start_id)
        hrstc = self.planning_env.ComputeHeuristicCost(start_id,goal_id)
        cost = dist + hrstc
        heapq.heappush(h, (cost, start_id))


        #explored =[start_id]
        backtrack = {}
        backtrack[start_id] = None
        
        while not found:
        	curr_b = heapq.heappop(h)
        	old_dist = curr_b[0]
        	curr_id = curr_b[1]
        	if curr_id == goal_id:
        		found = True
        		break
        	successors = self.planning_env.GetSuccessors(curr_id)
        	i=0

        	for successor in successors:
        		if successor != backtrack[curr_id]:
					dist = old_dist +  self.planning_env.ComputeDistance(curr_id, successor)
					hrstc = self.planning_env.ComputeHeuristicCost(successor,goal_id)
					cost = dist + hrstc
					temp_tuple = (cost, successor)
					heapq.heappush(h, temp_tuple)
					backtrack[successor] = curr_id
					
        # Shortest Path
        path = []
        path.append(self.planning_env.discrete_env.NodeIdToConfiguration(goal_id))
        element = backtrack[goal_id]
        while element is not None:
            path.append(self.planning_env.discrete_env.NodeIdToConfiguration(element))
            element = backtrack[element]
        plan = path[::-1]
        print plan
        return plan