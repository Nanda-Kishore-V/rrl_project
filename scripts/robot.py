import numpy as np
import math
import heapq

from constants import (
    X,
    Y,
    X_GRIDS,
    Y_GRIDS,
)

class Robot():

    def __init__(self, id, pose, goal, color):
        self.id = id
        self.pose = pose #x, y, yaw
        self.d_pose = np.zeros(3) # x_dot, y_dot, theta_dot
        self.command = [0, 0] # v, w
        self.goal = goal # x, y, yaw
        self.color = color

        grid_size_x = X/X_GRIDS; grid_size_y = Y/Y_GRIDS
        self.state_for_planner = (math.floor(self.pose[0]/grid_size_x), math.floor(self.pose[1]/grid_size_y))
        self.goal_for_planner = (math.floor(self.goal[0]/grid_size_x), math.floor(self.goal[1]/grid_size_y))
        self.directions = [(-1,0),(0,-1),(0,1),(1,0),(-1,-1),(-1,1),(1,-1),(1,1)]
        self.trajectory = None

        self.collision_points = []; self.collision_priority = []
        self.collision_with = [];   self.collision_impending = []

        self.initialize_planner()

    def model(self):
        self.d_pose = np.dot(np.array([[math.cos(self.pose[2]), 0],
                                       [math.sin(self.pose[2]), 0],
                                       [0, 1]]),
                                       self.command)
        return self.d_pose

    def update_step(self, dt):
        self.pose += self.model()*dt
        self.pose[2] = math.atan2(math.sin(self.pose[2]), math.cos(self.pose[2]))
        return self.pose

    def cmd_vel(self, v, w):
        self.command = [v, w]

    def angle_to_goal(self):
        return math.atan2(self.goal[1] - self.pose[1], self.goal[0] - self.pose[0])

    def __str__(self):
        return "X: {}, Y: {}, Theta: {}".format(self.pose[0], self.pose[1], self.pose[2]*180/math.pi)

    def initialize_planner(self):
        self.path = [];              self.open_list = []
        self.cost = {};              self.came_from = {}
        heapq.heappush(self.open_list, (self.heuristic(self.state_for_planner), self.state_for_planner))
        self.came_from[self.state_for_planner] = None
        self.cost[self.state_for_planner] = 0

    def heuristic(self, location):
        return abs((location[0]-self.goal_for_planner[0])**2+(location[1]-self.goal_for_planner[1])**2)**0.5
