"""
Written by Tianqi Liu, 2020 Feb.

It finds the optimal path for a car using Hybrid A* and bicycle model.
"""

import copy
import time
import heapq
import math
import matplotlib.pyplot as plt
import numpy as np


#possible steering controls
possible_str = {
    'l': -10,
    'l+': -50,
    'r+': +50,
    'r': +10,
    's': 0
}

#possible speed controls
possible_sp = {
    'f': 1,
    'b': -1
}


# total cost f(n) = actual cost g(n) + heuristic cost h(n)
class hybrid_a_star:
    def __init__(self, min_x, max_x, min_y, max_y, \
            obstacle=[], resolution=1, vehicle_length=2):
        ##TODO

        ###
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.obstacle = obstacle
        self.obstacle_dict = {}
        self.resolution = resolution
        self.vehicle_length = vehicle_length

        for obstacle_point in self.obstacle:
            self.obstacle_dict[obstacle_point] = 0

        ###

    """
    For each node n, we need to store:
    (discret_x, discret_y, heading angle theta),
    (continuous x, continuous y, heading angle theta)
    cost f, g,
    path [(continuous x, continuous y, continuous theta),...]

    start: discret (x, y, theta)
    end: discret (x, y, theta)
    sol_path = [(x1,y1,theta1),(x2,y2,theta2), ...]
    """
    def find_path(self, start, end):
        time_start = time.time()
        sol_path = []

        ##TODO

        frontier = []
        explored = {}
        current_path = {start : None}
        state = (self.heuristic(start, end), 0, start, start)
        heapq.heappush(frontier, (state[0], state))
        explored[start] = 0

        while len(frontier) != 0:
            current_state = heapq.heappop(frontier)[1]
            current_state_total_cost = current_state[0]
            current_state_previous_cost = current_state[1]
            current_state_point_discretized = current_state[2]
            current_state_point_continuous = current_state[3]
            next_points = self.get_next_points(current_state_point_discretized,
                                               current_state_point_continuous, explored)

            current_state_point_discretized_position = (current_state_point_discretized[0],
                                                        current_state_point_discretized[1])
            end_position = (end[0], end[1])

            if (current_state_point_discretized_position == end_position):
                sol_path = self.extract_path_list(current_path, current_state_point_continuous)
                break

            for next_point in next_points:
                next_state_point_discretized = next_point[0]
                next_state_point_continuous = next_point[1]
                next_state_steering_cost = self.steering_cost(current_state_point_discretized,
                                                              next_state_point_discretized)
                next_state_step_cost = self.heuristic(current_state_point_continuous,
                                                      next_state_point_continuous)
                next_state_total_cost = (current_state_previous_cost +
                                         next_state_step_cost +
                                         next_state_steering_cost +
                                         self.heuristic(next_state_point_discretized, end))
                next_state_previous_cost = (current_state_previous_cost +
                                            next_state_step_cost +
                                            next_state_steering_cost)
                current_path[next_state_point_continuous] = current_state_point_continuous
                next_state = (next_state_total_cost,
                              next_state_previous_cost,
                              next_state_point_discretized,
                              next_state_point_continuous)
                heapq.heappush(frontier, (next_state_total_cost, next_state))
                explored[next_state_point_discretized] = 0

        ###

        time_end = time.time()
        print("Time taken:", time_end - time_start)

        return sol_path

    def extract_path_list(self, current_path_dict, end_point):
        sol_path = []
        current_point = end_point

        while current_point != None:
            sol_path.append(current_point)
            current_point = current_path_dict[current_point]

        sol_path.reverse()

        return sol_path

    def vehicle_model(self, current_point_continuous, control_command):
        x = current_point_continuous[0]
        y = current_point_continuous[1]
        theta = math.radians(current_point_continuous[2])
        delta = math.radians(control_command[0])
        v = control_command[1]

        x_next = x + v * math.cos(theta)
        y_next = y + v * math.sin(theta)
        theta_next = int(math.degrees((theta + (float(v) / float(self.vehicle_length)) *
                                       math.tan(delta))))

        return (x_next, y_next, theta_next)

    def heuristic(self, start, end):
        h = ((start[0] - end[0]) ** 2 +
             (start[1] - end[1]) ** 2) ** 0.5

        return h

    def steering_cost(self, current_state_point_continuous, next_state_point_continuous):
        theta_difference = abs((float(next_state_point_continuous[2]) -
                                float(current_state_point_continuous[2])))
        theta_difference_delta_max = int(math.degrees(0.5 * math.tan(math.radians(50))))
        next_state_steering_cost = float(theta_difference) / float(theta_difference_delta_max)

        return next_state_steering_cost

    def get_next_points_continuous(self, current_point_continuous):
        next_points_continuous = []

        for str_key, str_value in possible_str.iteritems():
            for sp_key, sp_value in possible_sp.iteritems():
                next_point_continuous = self.vehicle_model(current_point_continuous,
                           (str_value, sp_value))

                if next_point_continuous not in next_points_continuous:
                    next_points_continuous.append(next_point_continuous)

        return next_points_continuous

    def get_next_points(self, current_point_discretized, current_point_continuous, explored):
        next_points = []
        next_points_continuous = self.get_next_points_continuous(current_point_continuous)

        for next_point_continuous in next_points_continuous:
            next_point_discretized_x = int(round(next_point_continuous[0]))
            next_point_discretized_y = int(round(next_point_continuous[1]))
            next_point_discretized_theta = next_point_continuous[2]
            next_point_discretized = (next_point_discretized_x,
                                      next_point_discretized_y,
                                      next_point_discretized_theta)
            next_point_discretized_position = (next_point_discretized_x,
                                               next_point_discretized_y)

            if (next_point_discretized_x >= self.min_x and
                next_point_discretized_x <= self.max_x and
                next_point_discretized_y >= self.min_y and
                next_point_discretized_y <= self.max_y and
                self.obstacle_dict.get(next_point_discretized_position) == None and
                explored.get(next_point_discretized) == None):
                next_points.append((next_point_discretized, next_point_continuous))

        return next_points

def main():
    print(__file__ + " start!!")

    # start and goal position
    #(x, y, theta) in meters, meters, degrees
    sx, sy, stheta= -5, -5, 0
    gx, gy, gtheta = 5, 5, 0

    #create obstacles
    obstacle = []

    for i in range(2):
        obstacle.append((0,i))
        obstacle.append((0,-i))

    ox, oy = [], []
    for (x,y) in obstacle:
        ox.append(x)
        oy.append(y)

    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "xr")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")

    hy_a_star = hybrid_a_star(-6, 6, -6, 6, obstacle=obstacle, \
        resolution=1, vehicle_length=2)
    path = hy_a_star.find_path((sx,sy,stheta), (gx,gy,gtheta))
    print (path)

    rx, ry = [], []
    for node in path:
        rx.append(node[0])
        ry.append(node[1])

    plt.plot(rx, ry, "-r")
    plt.show()


if __name__ == '__main__':
    main()
