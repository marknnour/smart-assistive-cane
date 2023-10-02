#! /usr/bin/python

import math
import rospy

from warnings import warn
import heapq
from assistive_cane.planner import Planner

"""
Code Implementation Overview

Day 1: None

Day 2: None

Day 3: TODO 1, TODO 2, TODO 3, TODO 4, TODO 5

Day 4: None

Day 5: Demo!
"""

class AStar(Planner):
    """
    A class implementing AStar planner

    Attributes
    ----------
    world_size_x: int
            length of world in x direction
    world_size_y: int
        length of world in y direction
    occupancy_grid_size_x: int
        number of rows in grid (default 100)
    occupancy_grid_size_y: int
        numver of columns in grid (default 100)

    Methods
    -------
    plan(start_node, end_node)
        Returns path from start_node to end_node if it exists, None otherwise
    return_path(current_node)
        Returns path to current_node following parent nodes
    """
    def __init__(self, world_size_x, world_size_y, occupancy_grid_size_x = 100, occupancy_grid_size_y = 100):
        """
        Parameters
        ----------
        world_size_x: int
            length of world in x direction
        world_size_y: int
            length of world in y direction
        occupancy_grid_size_x: int
            number of rows in grid (default 100)
        occupancy_grid_size_y: int
            number of columns in grid (default 100)
        """
        Planner.__init__(self, world_size_x, world_size_y, occupancy_grid_size_x, occupancy_grid_size_y)
        self.number = 0

    def plan(self, start_node, end_node):
        """
        Returns path from start_node to end_node if it exists, None otherwise

        Parameters
        ----------
        start_node: Node (see planner.py for details)
            start node in occupancy grid
        end_node: Node (see planner.py for details)
            goal node in occupancy grid

        Returns
        -------
        list
            list of nodes from start_node to end_node IF path exists

        OR

        None
            if path from start_node to end_node DOES NOT exist
        """

        """
        [Day 3] TODO 1: Implement the following:
            - Initialize both open and closed lists
            - Create a heap from open list
                NOTE: Node class has predefined greater than and less than functions which take care of heap ordering
            - Add the start node to the heap
        """
        ####### Insert Code Here #######
        
        # Initialize both open and closed lists
        open_list = []
        closed_list = []

        # Heapify the open_list
        heapq.heapify(open_list)

        # Add start node to heap
        heapq.heappush(open_list, start_node)
        # Remove the following line when done with above
	    
	
        ################################

        # Adding a stop condition
        outer_iterations = 0
        max_iterations = 1000000
        # Loop until you find the end
        while len(open_list) > 0:

            outer_iterations += 1

            if outer_iterations%1000 == 0:
                print("pathfinding ...")

            if outer_iterations > max_iterations:
              # if we hit this point return the path such as it is
              # it will not contain the destination
              warn("Could not find path: Exceeded maximum number of iterations")
              return self.return_path(current_node)       
            
            """
            [Day 3] TODO 2: Implement the following:
                - Remove node from heap (this is the current node) and add to closed list
                - Check if current node is end node.  If so return path to current node
            """
            ####### Insert Code Here #######

            # Get the current node
            current_node = heapq.heappop(open_list)
            
            # Found the goal
            closed_list.append(current_node)
            self.number += 1
            # Remove the following line when done with the above
            if current_node == end_node:
                return self.return_path(current_node)

            ################################

            # Loop through children
            for child in current_node.get_neighbors():

                # Make sure within range
                if child.x<0 or child.x >= self.occupancy_grid.shape[0] or child.y<0 or child.y >= self.occupancy_grid.shape[1]:
                    continue

                # Make sure walkable terrain
                if self.occupancy_grid[child.x, child.y] != 0:
                    # print(collision)
                    continue

                """
                [Day 3] TODO 3: Implement the following:
                    - Check if child is in closed list. If so continue to next iteration of for loop
                """
                ####### Insert Code Here #######
                
                # Child is on the closed list
                if child in closed_list:
                    continue     
                
                # Remove the following line when done with the above

                ################################

                h_x = abs(child.x - end_node.x)
                h_y = abs(child.y - end_node.y)
                h_direction = abs(child.direction - end_node.direction)

                if h_direction == 3:
                    h_direction = 1

                """
                [Day 3] TODO 4: Implement the following:
                    - Create f, g, and h values for child
                        NOTE: Combine h_x, h_y and h_direction as the heuristic. You can only move in 4 directions. 
                """
                ####### Insert Code Here #######

                # Create the f, g, and h values
                # TODO check
                #child.g = current_node.g + h_direction
                child.g = current_node.g + 1
                child.h = h_x + h_y
                child.f = child.g + child.h

                # Remove the following line when done with above
                ################################

                child.parent = current_node

                """
                [Day 3] TODO 5: Implement the following:
                    - Check if child is in open list and if it's g value is greater than any other open node's g value.  
                        If so continue to next iteration of loop
                    - Add child to open list
                """
                ####### Insert Code Here #######

                # Child is already in the open list
                print(child)
                if child in open_list:
                    if child.g > max([i.g for i in open_list]):
                        continue

                # Add the child to the open list
                heapq.heappush(open_list, child)
                # Remove the following line when done with above
                
            
                ################################
        warn("Could not find path: Path does not exist")
        return None

    def return_path(self, current_node):

        path = []
        current = current_node
        while True:
            path.append(current)
            if current.parent == None:
                break
            current = current.parent
        print("I am working")
        print("*" * 80)
        print("Number", self.number)
        return path[::-1]  # Return reversed path


