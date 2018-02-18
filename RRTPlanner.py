import numpy
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        plan.append(start_config)
        while True:
            next_config = self.planning_env.GenerateRandomConfiguration()
            #next_congfig = self.planning_env.Extend(plan[step-1], next_config)
            launch_vertex_id, launch_vertex = tree.GetNearestVertex(next_config)
            next_config = self.planning_env.Extend(launch_vertex, next_config)
            if (next_config == None):
                continue
            else:
                next_id = tree.AddVertex(next_config)
                tree.AddEdge(launch_vertex_id, next_id)
                self.planning_env.PlotEdge(launch_vertex,next_config)
            if (self.planning_env.ComputeDistance(goal_config, next_config) < 0.5):
                final_config = self.planning_env.Extend(next_config, goal_config)
                if (final_config == None):
                    continue
                else:
                    #plan.append(next_config)
                    sid = tree.AddVertex(final_config)
                    tree.AddEdge(next_id,sid)
                    path = []
                    while (sid <> 0):
                        path.append(sid)
                        sid = tree.edges.get(sid)
                        #print(sid)
                    break
        path.reverse()
        print(path)
        for step in path:
            plan.append(tree.vertices[step])
            print(plan[-1])
        return plan
