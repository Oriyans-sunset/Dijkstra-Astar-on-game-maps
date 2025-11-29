import heapq
import math

class State:
    """
    Class to represent a state on grid-based pathfinding problems. The class contains two static variables:
    map_width and map_height containing the width and height of the map. Although these variables are properties
    of the map and not of the state, they are used to compute the hash value of the state, which is used
    in the CLOSED list. 

    Each state has the values of x, y, g, h, and cost. The cost is used as the criterion for sorting the nodes
    in the OPEN list for both Dijkstra's algorithm and A*. For Dijkstra the cost should be the g-value, while
    for A* the cost should be the f-value of the node. 
    """
    map_width = 0
    map_height = 0
    
    def __init__(self, x, y):
        """
        Constructor - requires the values of x and y of the state. All the other variables are
        initialized with the value of 0.
        """
        self._x = x
        self._y = y
        self._g = 0
        self._cost = 0
        self._parent = None
        
    def __repr__(self):
        """
        This method is invoked when we call a print instruction with a state. It will print [x, y],
        where x and y are the coordinates of the state on the map. 
        """
        state_str = "[" + str(self._x) + ", " + str(self._y) + "]"
        return state_str
    
    def __lt__(self, other):
        """
        Less-than operator; used to sort the nodes in the OPEN list
        """
        return self._cost < other._cost
    
    def state_hash(self):
        """
        Given a state (x, y), this method returns the value of x * map_width + y. This is a perfect 
        hash function for the problem (i.e., no two states will have the same hash value). This function
        is used to implement the CLOSED list of the algorithms. 
        """
        return self._y * State.map_width + self._x
    
    def __eq__(self, other):
        """
        Method that is invoked if we use the operator == for states. It returns True if self and other
        represent the same state; it returns False otherwise. 
        """
        return self._x == other._x and self._y == other._y

    def get_x(self):
        """
        Returns the x coordinate of the state
        """
        return self._x
    
    def set_parent(self, parent):
        """
        Sets the parent of a node in the search tree
        """
        self._parent = parent

    def get_parent(self):
        """
        Returns the parent of a node in the search tree
        """
        return self._parent
    
    def get_y(self):
        """
        Returns the y coordinate of the state
        """
        return self._y
    
    def get_g(self):
        """
        Returns the g-value of the state
        """
        return self._g
        
    def set_g(self, g):
        """
        Sets the g-value of the state
        """
        self._g = g

    def get_cost(self):
        """
        Returns the cost of a state; the cost is determined by the search algorithm
        """
        return self._cost
    
    def set_cost(self, cost):
        """
        Sets the cost of the state; the cost is determined by the search algorithm 
        """
        self._cost = cost

class Dijkstra:
    def __init__(self, grid_map):
        self.grid_map = grid_map  
        self.closed = {}         

    def search(self, start, goal):
        """ 
        Implement Dijkstra's Algorithm 
        """
        open_list = []  # priority queue
        closed_list = {} # explored nodes
        self.closed = closed_list

        heapq.heappush(open_list, start)  # initilise the open list wit start node

        while open_list:
            current = heapq.heappop(open_list)  
            if current.state_hash() in closed_list: # skip this node if it has already been expanded(is in closed list)
                continue 

            closed_list[current.state_hash()] = current  # mark as explored, add to closed list

            if current == goal:
                return self.reconstruct_path(current), current.get_g(), len(closed_list)

            for neighbor in self.grid_map.successors(current):
                if neighbor.state_hash() in closed_list:
                    continue  

                new_cost = current.get_g() + self.grid_map.cost(neighbor.get_x() - current.get_x(), neighbor.get_y() - current.get_y()) # curretn cost + cost of moving to neighbor

                if new_cost <= neighbor.get_g(): # better cost found
                    neighbor.set_g(new_cost)
                    neighbor.set_cost(new_cost) 
                    neighbor.set_parent(current)
                    heapq.heappush(open_list, neighbor) 

        return None, -1, len(closed_list)  
    
    def reconstruct_path(self, node):
        """ 
        Reconstructs the shortest path from goal to start 
        """
        path = []
        while node:
            path.append(node)
            node = node.get_parent()
        return path[::-1]

    def get_closed_data(self):
        """ 
        Returns the closed list
        """
        return self.closed


class AStar:
    def __init__(self, grid_map):
        self.grid_map = grid_map
        self.closed = {} 
    
    def search(self, start, goal):
        """
        Implement A* search
        """
        open_list = []     
        closed_list = {}    
        self.closed = closed_list
        
        start.set_g(0)
        h_start = self.heuristic(start, goal)
        start.set_cost(start.get_g() + h_start)  # compute f value = g + h

        heapq.heappush(open_list, start)
        
        while open_list:
            current = heapq.heappop(open_list)
            
            if current.state_hash() in closed_list:
                continue
            
            closed_list[current.state_hash()] = current
            
            if current == goal:
                return self.reconstruct_path(current), current.get_g(), len(closed_list)
            
            for neighbor in self.grid_map.successors(current):
                if neighbor.state_hash() in closed_list:
                    continue
                
                move_cost = self.grid_map.cost(neighbor.get_x() - current.get_x(), neighbor.get_y() - current.get_y())
                new_g = current.get_g() + move_cost
                
                if new_g <= neighbor.get_g():
                    neighbor.set_g(new_g)
                    h = self.heuristic(neighbor, goal)
                    neighbor.set_cost(new_g + h)  
                    neighbor.set_parent(current)
                    heapq.heappush(open_list, neighbor)
                    
        return None, -1, len(closed_list)
    
    def heuristic(self, state, goal):
        """
        Compute the octile distance between the given state and goal state
        """
        dx = abs(state.get_x() - goal.get_x())
        dy = abs(state.get_y() - goal.get_y())
        return 1.5 * min(dx, dy) + abs(dx - dy)
    
    def reconstruct_path(self, node):
        """
        Reconstructs the path from start to goal
        """
        path = []
        while node:
            path.append(node)
            node = node.get_parent()
        return path[::-1]
    
    def get_closed_data(self):
        """
        Returns the CLOSED list
        """
        return self.closed