import numpy as np
import queue
from math import sqrt

class Astar:
    def __init__(self):
        # graph node: x, y, cost, parent
        self.cur_vis = []   # current visited node list
        self.next_vis = []  # next visit node list
        self.count = 0

    def find_path(self, graph, start_pos, goal_pos):
        
        print('start to Astar search') 

        frontier = queue.PriorityQueue()  # priority queue for algorithm to explore current points.
        frontier.put((0, start_pos))  # put the priority and position

        self.cur_vis.append([start_pos.x, start_pos.y])
        self.next_vis.append([start_pos.x, start_pos.y])
        
        cost_so_far = np.zeros((graph.width, graph.height)) # cost matrix from start point to this point
        final_node = None

        while not frontier.empty():
            # print('you should complete this part to find the path')
            # please complete this part for the homework question1 
            # each node has four parts: x position, y position, the cost so far, the parent node. Utilize the parent node, the path can be generated

            # parts: (1) get current node with priority (using frontier.get()[1])  
            # (2) check whether current node is the goal node (using graph.node_equal) 
            # (3) explore the neighbors of current node (using graph.neighbors)
            # (4) if the neighbor node is not in the next_vis (and cur_vis): put that in the frontier with priority and append that in the next_vis.  (priority= cost_so_far + heuristic)
            # (5) if the neighbor node is in the next_vis: check whether cost so far is less than that in the next_vis to determine whether put it in the frontier
            # (6) return the node when it is in the goal position.

            cur_node = frontier.get()[1]
            self.cur_vis.append([cur_node.x,cur_node.y])
            if graph.node_equal(cur_node, goal_pos):
                final_node=cur_node
                break
            for node in graph.neighbors(cur_node):
                if self.cur_vis.__contains__([node.x,node.y]):
                    continue
                elif not self.next_vis.__contains__([node.x,node.y]):
                    frontier.put(((node.cost + Astar.heuristic(self,node,goal_pos)),node))
                    self.next_vis.append([node.x,node.y])
                else:
                    cur = frontier.get()
                    if (node.cost + Astar.heuristic(self, node,goal_pos))<cur[0]:
                        frontier.put(((node.cost + Astar.heuristic(self,node,goal_pos)),node))
                    else:
                        frontier.put(cur)
        print('search done')

        return final_node, self.cur_vis

    def heuristic(self, node1, node2, coefficient=1):
        # please complete the heuristic function for the homework question1  (related to the distance to the goal)
        return coefficient*((node1.x-node2.x)**2 + (node1.y-node2.y)**2)
        

    def generate_path(self, final_node):
        # utilize the node to generate the path. 

        path = [ [final_node.x, final_node.y] ]
        
        while final_node.parent is not None:
            path.append( [final_node.parent.x, final_node.parent.y] )
            final_node = final_node.parent
        
        return path