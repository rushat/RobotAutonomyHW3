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
#        q = Queue()
 #       start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
  #      goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
   #     found = False
    #    q.put(start_id)

     #   while not (found):
            

      #      while (q.qsize()>0):

       #         successors = self.planning_env.GetSuccessors(q.get())
        #        print successors





     #   plan.append(start_config)
     #   plan.append(goal_config)
   
        return plan
