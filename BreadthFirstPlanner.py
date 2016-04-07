from Queue import Queue
import time
class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    def Plan(self, start_config, goal_config):
        start_time = time.time()
        if self.visualize and hasattr(self.planning_env, "InitializePlot"):
            self.planning_env.InitializePlot(goal_config)

        plan = []
        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is thgit e dimension of the robots configuration space
        q = Queue()
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        found = False
        q.put(start_id)
        explored =[start_id]
        backtrack = {}
        backtrack[start_id] = None
        n = 0
        while (q.qsize()>0) and not found:
            current = q.get()                
            successors = self.planning_env.GetSuccessors(current)
            for successor in successors:
                if not successor in explored:
                    q.put(successor)
                    n=n+1
                    explored.append(successor)
                    backtrack[successor] = current
                    if self.visualize: 
                        s = self.planning_env.discrete_env.NodeIdToConfiguration(successor)
                        c = self.planning_env.discrete_env.NodeIdToConfiguration(current)
                        self.planning_env.PlotEdge(c,s)
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
        if self.visualize: 
            for i in range(len(path) - 1):
                self.planning_env.PlotRedEdge(path[i],path[i+1])
        print "number of nodes"
        print n
        print "time (in seconds):" 
        print time.time()- start_time
        path_length = 0
        for i in range(len(path) - 1):
            path_length = path_length + self.planning_env.ComputeDistance(self.planning_env.discrete_env.ConfigurationToNodeId(path[i]), self.planning_env.discrete_env.ConfigurationToNodeId(path[i+1]))
            
        print "path path_length"
        print path_length
        return plan