#! /usr/bin/python

import math
import numpy as np

from geometry_msgs.msg import Quaternion, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import cv2
from cv_bridge import CvBridge, CvBridgeError

from warnings import warn
import heapq

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec

from assistive_cane.world_visualizer import WorldVisualizer
from assistive_cane.constants import BOX_SIZE_X, BOX_SIZE_Y, BOX_SIZE_X_PADDED, BOX_SIZE_Y_PADDED, CANE_HEIGHT

class Node:
    """
    A node class for Pathfinding

    Attributes
    ----------
    x: int
        x coordinate of node
    y: int
        y coordinate of node
    direction: int
        direction/orientation of node
    g: int (default 0)
        
    h: int (default 0)
        
    f: int (default 0)
        
    parent: Node (default None)
        parent Node
    """

    def __init__(self, x, y, direction):
        """
        Parameters
        ----------
        x: int
            x coordinate of node
        y: int
            y coordinate of node
        direction: int
            direction/orientation of node
        """
        self.x = int(x)
        self.y = int(y) 
        self.direction = int(direction)

        self.g = 0
        self.h = 0
        self.f = 0

        self.parent = None

    def get_neighbors(self):
        """
        Returns neighbor Node in direction of self.direction

        Parameters
        ----------
        None

        Returns
        -------
        Node
            Node in direction of self.direction

        OR

        None
            if self.direction is not equal to 0-3
        """

        if self.direction == 0: # Up
            return [Node(self.x, self.y+1, 0), Node(self.x,self.y,3), Node(self.x,self.y,1)]
        elif self.direction == 1: #Right
            return [Node(self.x+1, self.y, 1), Node(self.x,self.y,0), Node(self.x,self.y,2)]
        elif self.direction == 2: #Down
            return [Node(self.x, self.y-1, 2), Node(self.x,self.y,1), Node(self.x,self.y,3)]
        elif self.direction == 3:
            return [Node(self.x-1, self.y, 3), Node(self.x,self.y,2), Node(self.x,self.y,0)]
        else:
            return None

    def __eq__(self, other):
        if other == None:
            return False
        return self.x == other.x and self.y == other.y and self.direction == other.direction
    
    def __repr__(self):
        return f"({self.x},{self.y},{self.direction}) - g: {self.g} h: {self.h} f: {self.f}"

    # defining less than for purposes of heap queue
    def __lt__(self, other):
        if self.f == other.f:
            return self.h < other.h
        return self.f < other.f
    
    # defining greater than for purposes of heap queue
    def __gt__(self, other):
        if self.f == other.f:
            return self.h > other.h
        return self.f > other.f

class Planner:
    def __init__(self, world_size_x, world_size_y, occupancy_grid_size_x = 100, occupancy_grid_size_y = 100):

        self.world_size_x = world_size_x
        self.world_size_y = world_size_y

        self.occupancy_grid = np.zeros((occupancy_grid_size_x,occupancy_grid_size_y))
        
        #  only used in visualization
        self.actual_object_grid = np.zeros((occupancy_grid_size_x,occupancy_grid_size_y))

        self.resolution_x = self.occupancy_grid.shape[0]/self.world_size_x
        self.resolution_y = self.occupancy_grid.shape[1]/self.world_size_y

    def visualize(self, start = None, goal = None, path = None):

        fig = plt.figure(figsize=(40,24), dpi=80)
        ax = fig.add_subplot(111)

        number_in_collision = 0
        for i in range(self.occupancy_grid.shape[0]):
            for j in range(self.occupancy_grid.shape[1]):
                if self.occupancy_grid[i,j]==1:
                    ax.add_patch(patches.Rectangle(
                        (i/self.occupancy_grid.shape[0], j/self.occupancy_grid.shape[1]),   # (x,y)
                        1.0/self.occupancy_grid.shape[0],          # width
                        1.0/self.occupancy_grid.shape[1],          # height
                        alpha=6,
                        color = "#676767"
                        ))
                    number_in_collision += 1

                # Visualize actual box pose without padding    
                if self.actual_object_grid[i,j]==1:
                    ax.add_patch(patches.Rectangle(
                        (i/self.actual_object_grid.shape[0], j/self.actual_object_grid.shape[1]),   # (x,y)
                        1.0/self.actual_object_grid.shape[0],          # width
                        1.0/self.actual_object_grid.shape[1],          # height
                        alpha=6,
                        color = "#000000"
                        ))

        print("Number in collision: ",number_in_collision)

        def get_marker_type(direction):
            if direction == 0:
                return "^"
            elif direction == 1:
                return ">"
            elif direction == 2:
                return "v"
            elif direction == 3:
                return "<"
        
        if path != None:
            for node in path:
                plt.scatter(node.x/self.occupancy_grid.shape[0], node.y/self.occupancy_grid.shape[1], color = "green",  s = 300, edgecolors="black", marker = get_marker_type(node.direction))

        if start != None:
            plt.scatter(start.x/self.occupancy_grid.shape[0], start.y/self.occupancy_grid.shape[1], color = "red",  s = 300, edgecolors="black", marker = get_marker_type(start.direction))

        if goal != None:
            plt.scatter(goal.x/self.occupancy_grid.shape[0], goal.y/self.occupancy_grid.shape[1], color = "blue",  s = 300, edgecolors="black", marker = get_marker_type(goal.direction))

        plt.xlim(0,1)
        plt.ylim(0,1)

        plt.show()

    def set_box_poses(self, box_poses):

        def is_on_right_side(x, y, xy0, xy1):
            x0, y0 = xy0
            x1, y1 = xy1
            a = float(y1 - y0)
            b = float(x0 - x1)
            c = - a*x0 - b*y0
            return a*x + b*y + c >= 0

        def test_point(x, y, vertices):
            num_vert = len(vertices)
            is_right = [is_on_right_side(x, y, vertices[i], vertices[(i + 1) % num_vert]) for i in range(num_vert)]
            all_left = not any(is_right)
            all_right = all(is_right)
            return all_left or all_right

        for i in range(self.occupancy_grid.shape[0]):
            for j in range(self.occupancy_grid.shape[1]):

                collision = False
                for box_pose in box_poses:
                    centre_x = box_pose.position.x*self.resolution_x 
                    centre_y = box_pose.position.y*self.resolution_y
                    
                    orientation_list = [box_pose.orientation.x, box_pose.orientation.y, box_pose.orientation.z, box_pose.orientation.w]
                    (_, _, rotation) = euler_from_quaternion(orientation_list)
                    top_left_x = centre_x + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X_PADDED/2.0)**2 + (self.resolution_y*BOX_SIZE_Y_PADDED/2.0)**2)*math.cos(rotation + math.pi - math.atan2(BOX_SIZE_Y_PADDED, BOX_SIZE_X_PADDED))
                    top_left_y = centre_y + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X_PADDED/2.0)**2 + (self.resolution_y*BOX_SIZE_Y_PADDED/2.0)**2)*math.sin(rotation + math.pi - math.atan2(BOX_SIZE_Y_PADDED, BOX_SIZE_X_PADDED))

                    top_right_x = centre_x + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X_PADDED/2.0)**2 + (self.resolution_y*BOX_SIZE_Y_PADDED/2.0)**2)*math.cos(rotation + math.atan2(BOX_SIZE_Y_PADDED, BOX_SIZE_X_PADDED))
                    top_right_y = centre_y + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X_PADDED/2.0)**2 + (self.resolution_y*BOX_SIZE_Y_PADDED/2.0)**2)*math.sin(rotation + math.atan2(BOX_SIZE_Y_PADDED, BOX_SIZE_X_PADDED))

                    bottom_left_x = centre_x + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X_PADDED/2.0)**2 + (self.resolution_y*BOX_SIZE_Y_PADDED/2.0)**2)*math.cos(rotation + math.pi + math.atan2(BOX_SIZE_Y_PADDED, BOX_SIZE_X_PADDED))
                    bottom_left_y = centre_y + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X_PADDED/2.0)**2 + (self.resolution_y*BOX_SIZE_Y_PADDED/2.0)**2)*math.sin(rotation + math.pi + math.atan2(BOX_SIZE_Y_PADDED, BOX_SIZE_X_PADDED))

                    bottom_right_x = centre_x + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X_PADDED/2.0)**2 + (self.resolution_y*BOX_SIZE_Y_PADDED/2.0)**2)*math.cos(rotation + 2*math.pi - math.atan2(BOX_SIZE_Y_PADDED, BOX_SIZE_X_PADDED))
                    bottom_right_y = centre_y + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X_PADDED/2.0)**2 + (self.resolution_y*BOX_SIZE_Y_PADDED/2.0)**2)*math.sin(rotation + 2*math.pi - math.atan2(BOX_SIZE_Y_PADDED, BOX_SIZE_X_PADDED))

                    vertices = [(top_left_x, top_left_y), (top_right_x, top_right_y), (bottom_right_x, bottom_right_y), (bottom_left_x, bottom_left_y)]

                    if test_point(i, j, vertices):
                        collision = True
                        break

                if collision:
                    self.occupancy_grid[i,j] = 1
                else:
                    self.occupancy_grid[i,j] = 0

        for i in range(self.actual_object_grid.shape[0]):
            for j in range(self.actual_object_grid.shape[1]):

                collision = False
                for box_pose in box_poses:
                    centre_x = box_pose.position.x*self.resolution_x 
                    centre_y = box_pose.position.y*self.resolution_y
                    
                    orientation_list = [box_pose.orientation.x, box_pose.orientation.y, box_pose.orientation.z, box_pose.orientation.w]
                    (_, _, rotation) = euler_from_quaternion(orientation_list)
                    top_left_x = centre_x + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X/2.0)**2 + (self.resolution_y*BOX_SIZE_Y/2.0)**2)*math.cos(rotation + math.pi - math.atan2(BOX_SIZE_Y, BOX_SIZE_X))
                    top_left_y = centre_y + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X/2.0)**2 + (self.resolution_y*BOX_SIZE_Y/2.0)**2)*math.sin(rotation + math.pi - math.atan2(BOX_SIZE_Y, BOX_SIZE_X))

                    top_right_x = centre_x + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X/2.0)**2 + (self.resolution_y*BOX_SIZE_Y/2.0)**2)*math.cos(rotation + math.atan2(BOX_SIZE_Y, BOX_SIZE_X))
                    top_right_y = centre_y + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X/2.0)**2 + (self.resolution_y*BOX_SIZE_Y/2.0)**2)*math.sin(rotation + math.atan2(BOX_SIZE_Y, BOX_SIZE_X))

                    bottom_left_x = centre_x + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X/2.0)**2 + (self.resolution_y*BOX_SIZE_Y/2.0)**2)*math.cos(rotation + math.pi + math.atan2(BOX_SIZE_Y, BOX_SIZE_X))
                    bottom_left_y = centre_y + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X/2.0)**2 + (self.resolution_y*BOX_SIZE_Y/2.0)**2)*math.sin(rotation + math.pi + math.atan2(BOX_SIZE_Y, BOX_SIZE_X))

                    bottom_right_x = centre_x + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X/2.0)**2 + (self.resolution_y*BOX_SIZE_Y/2.0)**2)*math.cos(rotation + 2*math.pi - math.atan2(BOX_SIZE_Y, BOX_SIZE_X))
                    bottom_right_y = centre_y + 1.3*math.sqrt((self.resolution_x*BOX_SIZE_X/2.0)**2 + (self.resolution_y*BOX_SIZE_Y/2.0)**2)*math.sin(rotation + 2*math.pi - math.atan2(BOX_SIZE_Y, BOX_SIZE_X))

                    vertices = [(top_left_x, top_left_y), (top_right_x, top_right_y), (bottom_right_x, bottom_right_y), (bottom_left_x, bottom_left_y)]

                    if test_point(i, j, vertices):
                        collision = True
                        break

                if collision:
                    self.actual_object_grid[i,j] = 1
                else:
                    self.actual_object_grid[i,j] = 0

    def convert_path_to_world(self, path):

        world_path = []

        for node in path:

            q = quaternion_from_euler(0, 0, math.pi/2.0 -node.direction*math.pi/2.0)

            pose = Pose()
            pose.position.x = node.x/self.resolution_x
            pose.position.y = node.y/self.resolution_y
            pose.position.z = CANE_HEIGHT
            pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

            world_path.append(pose)

        return world_path

    def solve(self, start, goal, box_poses):

        # Create start and end node
        start_node = Node(start[0]*self.resolution_x,start[1]*self.resolution_y,start[2])
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(goal[0]*self.resolution_x,goal[1]*self.resolution_y,goal[2])
        end_node.g = end_node.h = end_node.f = 0

        self.visualize(start_node,end_node)
        
        self.set_box_poses(box_poses)

        self.visualize(start_node,end_node)

        path = self.plan(start_node, end_node)
        print("Path cost: ",path[len(path)-1].g)
        self.visualize(start_node,end_node,path)

        path = self.get_waypoints(path)    
        self.visualize(start_node,end_node,path)    
        return self.convert_path_to_world(path)

    def get_waypoints(self,path):

        if path == None or len(path) == 0:
            return None

        waypoints = [path[0]]
        
        for i in range(1,len(path)-1):
            if (path[i-1].x != path[i].x, path[i-1].y != path[i].y, path[i-1].direction != path[i].direction) != (path[i].x != path[i+1].x, path[i].y != path[i+1].y, path[i].direction != path[i+1].direction):
                waypoints.append(path[i])
        waypoints.append(path[len(path)-1])
        return waypoints  # Return waypoints

    def plan(self, start, goal):
        raise NotImplementedError("Please Implement this method")