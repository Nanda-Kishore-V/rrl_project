import numpy as np
import itertools
import heapq

from constants import(
    X, Y,
    X_GRIDS, Y_GRIDS,
)

class World:

    def __init__(self, robots, time):
        self.robots = robots
        self.time = time
        self.x_range = [0, X]
        self.y_range = [0, Y]
        self.obstacles = []
        self.collision = []
        # self.pltr = Plotter()

    def is_obstacle(self, location):
        return location in self.obstacles

    def planning_iteration(self, robot):
        current = heapq.heappop(robot.open_list)[1]
        if(current[0] == robot.goal_for_planner[0] and current[1] == robot.goal_for_planner[1]):
            while current in robot.came_from:
                robot.path.append([current[0]*X/X_GRIDS, current[1]*Y/Y_GRIDS])
                current = robot.came_from[current]
            robot.path.reverse()
            return 1
        for i,j in robot.directions:
            neighbor = (current[0]+i,current[1]+j)
            if neighbor[0] < 0 or neighbor[0] >= X_GRIDS or neighbor[1] < 0 or neighbor[1] >= Y_GRIDS:
                continue
            if self.is_obstacle(neighbor):
                continue
            if neighbor not in robot.cost or robot.cost.get(neighbor,0) > robot.cost[current]+1:
                robot.came_from[neighbor] = current
                robot.cost[neighbor] = robot.cost[current]+1
                priority = robot.cost[neighbor] + robot.heuristic(neighbor)
                heapq.heappush(robot.open_list,(priority,neighbor))
        return 0

    def generate_paths(self):
        completed = set()
        while completed != set(range(len(self.robots))):
            for robot_id, robot in enumerate(self.robots):
                if robot_id not in completed:
                    if self.planning_iteration(robot):
                        completed.add(robot_id)


    def detect_collision(self):
        for r1, r2 in itertools.combinations(self.robots, 2):
            time = min(r1.trajectory[0], r2.trajectory[0])
            r1_x, r1_y = r1.trajectory[1], r1.trajectory[2]
            r2_x, r2_y = r2.trajectory[1], r2.trajectory[2]
            for i in range(1000):
                t = i*time/1000
                x1, y1 = r1_x(t), r1_y(t)
                x2, y2 = r2_x(t), r2_y(t)
                if ((x1 - x2)**2 + (y1 - y2)**2)**0.5 < 0.1:
                    self.collision.append([(r1.id, r2.id), (x1, y1)])
                    r1.collision_points.append((x1, y1))
                    r2.collision_points.append((x2, y2))
                    if r1.id > r2.id:
                        r1.collision_priority.append('inc')
                        r2.collision_priority.append('dec')
                    else:
                        r1.collision_priority.append('dec')
                        r2.collision_priority.append('inc')
                    break
        print self.collision
