from Queue import Queue

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    def Plan(self, start_config, goal_config):
        
        plan = []
        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        q = Queue()
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        found = False
        q.put(start_id)
        explored =[start_id]
        backtrack = {}
        backtrack[start_id] = None
        while (q.qsize()>0) and not found:
            current = q.get()                
            successors = self.planning_env.GetSuccessors(current)
            for successor in successors:
                if not successor in explored:
                    q.put(successor)
                    explored.append(successor)
                    backtrack[successor] = current
                    if successor == goal_id:
                        found = True
                        break
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
