#!/usr/bin/env python3

from heapq import heapify, heappush, heappop
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
import numpy as np

from modules.tires import *
from modules.vehicle_models import *
from modules.simulator import *
from modules.engine_models import *
from modules.steering_models import *
from modules.environment import *
from modules.sprites import *


env = Environment(0.1, theme='dark')

# Define the graph
graph_points = np.array([[0.,    0.],
                        [114.28714842,   41.98759603],
                        [62.47783741,  -10.16037304],
                        [24.5058101,  179.34217451],
                        [-9.51912637,   86.66461683],
                        [-107.49794568,   23.53735607],
                        [91.98201947,  153.39087436],
                        [130.74459844,   95.76204485],
                        [100.11681872,   -9.67705624],
                        [-97.97803427, -116.85725875],
                        [12.76266999,  200.86528354],
                        [35.42322761,  -86.24626489],
                        [-15.17293957,  -39.18455774],
                        [50.71204359,  -43.96724648],
                        [-165.73686143,  -40.45509785],
                        [-59.11389879,  -75.68251789],
                        [-65.86307249,  -51.20886232],
                        [-118.65056562,   58.30787596],
                        [0.25090536,   49.73685338],
                        [95.02087717,   63.66352072]])

adj_list = np.array([[2, 4, 12, 18],
                    [7, 8, 19],
                    [0, 8, 11, 13, 18],
                    [4, 6, 10],
                    [0, 3, 10, 14, 17, 18],
                    [14, 16, 17],
                    [3, 7, 10, 19],
                    [1, 6, 19],
                    [1, 2, 13, 19],
                    [14, 15, 16],
                    [3, 4, 6],
                    [2, 12, 13],
                    [0, 11, 15, 16],
                    [2, 8, 11],
                    [4, 5, 9, 16, 17],
                    [9, 12, 16],
                    [5, 9, 12, 14, 15],
                    [4, 5, 14],
                    [0, 2, 4],
                    [1, 6, 7, 8]])


def transition_function(p):
    '''
    Use this function to get neighbors of a point p and the distances from the neighbors to that point p

    Parameters
    ----------
    p: np.array, shape=2
        [x,y] for the point

    Returns
    -------
    list of points, each one is a neighbor of the given point "p"
        [[neighbor1.x,neighbor1.y],[neighbor2.x,neighbor2.y],...]

    '''
    idx = np.where(graph_points == p)[0]
    if idx.shape[0] == 0:
        print("Error, Point given not in the graph")
        return []
    i = idx[0]
    pi = graph_points[i]
    neighbors = graph_points[adj_list[i]]
    dists = []
    for p in enumerate(neighbors):
        dists.append(np.linalg.norm(p-pi).sum())
    return neighbors, np.array(dists)


# Iterative BFS
def BFS(initial_state, goal_state):
    queue = []
    visited = [0]*len(graph_points)
    prev = [[0, 0]]*len(graph_points)

    queue.append(start_point)  # Enqueue(initital_state)
    visited[start_point] = True

    # while the queue is NOT empty
    while len(queue):
        node = queue.pop()  # Dequeue()

        for adj_node in adj_list[node]:
            if not visited[adj_node]:
                visited[adj_node] = True
                queue.append(adj_node)  # Enqueue(adj_node)
                prev[adj_node] = node

    # reconstructing the path from initital_state to goal_state
    path = [graph_points[end_point]]  # the path starts
    goal = end_point  # initail value
    while goal != start_point:  # condition

        # as the list will be inverted
        path.insert(0, graph_points[prev[goal]])

        goal = prev[goal]  # increment

    return path


def Dijkstra(initial_state, goal_state):
    pass


def A_star(initial_state, goal_state):
    pass


def DFS(initial_state, goal_state):
    pass


def tree_search(initial_state, goal_state):
    '''
    Implement this function, used to find the set of points to follow to go from the start to the goal

    Parameters
    ----------
    initial_state: np.array, shape=2
        [x,y] of the start point, must be a point from the graph_points array declared above

    goal_state: np.array, shape=2
        [x,y] of the goal point, must be a point from the graph_points array declared above

    Returns
    -------
    If found goal, list of points
        The list of points, each point is [x,y], so the function should return [[point1.x, point1.y],[point2.x,point2.y],...]
    '''
    # TODO, implement the function to search using BFS and/or A*
    return BFS(initial_state, goal_state)


points = tree_search(graph_points[0], graph_points[14])
env.add_path(points=points, spline_degree=2, resolution=10, smoothness=0)

# Draw Graph Nodes
for p in graph_points:
    env.add_cone(p[0], p[1], radius=3)
###########################################################################################


def update_car_state(data):
    env.update_state(data.data)


rospy.init_node("environment")
rospy.Subscriber("vehicle_model/state", Float64MultiArray, update_car_state)
paths_publisher = rospy.Publisher(
    'paths/current_paths', Float64MultiArray, queue_size=10)
cones_publisher = rospy.Publisher(
    'cones/current_cones', Float64MultiArray, queue_size=10)

# Creating a placeholder message to send current paths
current_paths_msg = Float64MultiArray()
path_layout = MultiArrayLayout()
path_dimension = MultiArrayDimension()
path_dimension.label = "paths_descriptions"
path_dimension.size = np.array(env.path_params).reshape(-1).shape[0]
path_dimension.stride = np.array(env.path_params).reshape(-1).shape[0]
path_layout.data_offset = 0
path_layout.dim = [path_dimension]

# Creating a placeholder message to send current cones
current_cones_msg = Float64MultiArray()
cone_layout = MultiArrayLayout()
cone_dimension = MultiArrayDimension()
cone_dimension.label = "paths_descriptions"
cone_dimension.size = np.array(env.cone_params).reshape(-1).shape[0]
cone_dimension.stride = np.array(env.cone_params).reshape(-1).shape[0]
cone_layout.data_offset = 0
cone_layout.dim = [cone_dimension]


def publish_paths():
    current_paths_msg.data = np.array(env.path_params).reshape(-1).tolist()
    current_paths_msg.layout = path_layout

    paths_publisher.publish(current_paths_msg)


def publish_cones():
    current_cones_msg.data = np.array(env.cone_params).reshape(-1).tolist()
    current_cones_msg.layout = cone_layout

    cones_publisher.publish(current_cones_msg)


env.play(funcs_call=[publish_paths, publish_cones])
