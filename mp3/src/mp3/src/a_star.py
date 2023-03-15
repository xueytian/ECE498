import copy
import time
import heapq
import math
import matplotlib.pyplot as plt
import numpy as np


# car state = (x,y)
# state tuple (f,g,(x,y), [(x1,y1),(x2,y2)...])
# total cost f(n) = actual cost g(n) + heuristic cost h(n)
# obstacles = [(x,y), ...]
# min_x, max_x, min_y, max_y are the boundaries of the environment
class a_star:
    def __init__(self, min_x, max_x, min_y, max_y, \
            obstacle=[], resolution=1, robot_size=1):
        ##TODO

        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.obstacle = obstacle
        self.resolution = resolution
        self.robot_size = robot_size

        ####

    # state: (total cost f, previous cost g, current position (x,y), \
    # previous motion id)
    # start = (sx, sy)
    # end = (gx, gy)
    # sol_path = [(x1,y1),(x2,y2), ...]
    def find_path(self, start, end):
        time_start = time.time()
        sol_path = []

        ##TODO

        frontier = []
        explored = {}
        current_path = {start : None}
        state = (self.heuristic(start, end), 0, start, -1)
        heapq.heappush(frontier, (state[0], state))
        explored[start] = 0

        while len(frontier) != 0:
            current_state = heapq.heappop(frontier)[1]
            current_state_total_cost = current_state[0]
            current_state_previous_cost = current_state[1]
            current_state_position = current_state[2]
            current_state_previous_motion_id = current_state[3]
            next_points = self.get_next_points(current_state_position, explored)

            if (current_state_position == end):
                sol_path = self.extract_path_list(current_path, end)
                break

            for next_point in next_points:
                next_state_total_cost = (current_state_previous_cost +
                                         self.resolution +
                                         self.heuristic(next_point, end))
                next_state_previous_cost = current_state_previous_cost + self.resolution
                next_state_position = next_point
                next_state_previous_motion_id = current_state_previous_motion_id
                current_path[next_state_position] = current_state_position
                next_state = (next_state_total_cost,
                              next_state_previous_cost,
                              next_state_position,
                              next_state_previous_motion_id)
                heapq.heappush(frontier, (next_state_total_cost, next_state))
                explored[next_point] = 0

        ####

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

    def heuristic(self, start, end):
        h = ((start[0] - end[0]) ** 2 +
             (start[1] - end[1]) ** 2) ** 0.5

        return h

    def get_next_points(self, point, explored):
        point_x = point[0]
        point_y = point[1]
        next_points_candidate = [
            (point_x + self.resolution, point_y + self.resolution),
            (point_x - self.resolution, point_y - self.resolution),
            (point_x + self.resolution, point_y - self.resolution),
            (point_x - self.resolution, point_y + self.resolution),
            (point_x                  , point_y + self.resolution),
            (point_x                  , point_y - self.resolution),
            (point_x + self.resolution, point_y                  ),
            (point_x - self.resolution, point_y                  ),
        ]
        next_points = []

        for next_point in next_points_candidate:
            next_point_x = next_point[0]
            next_point_y = next_point[1]

            if (next_point_x >= self.min_x and
                next_point_x <= self.max_x and
                next_point_y >= self.min_y and
                next_point_y <= self.max_y and
                next_point not in self.obstacle and
                explored.get(next_point) == None):
                next_points.append(next_point)

        return next_points

def main():
    print(__file__ + " start!!")

    grid_size = 1  # [m]
    robot_size = 1.0  # [m]

    sx, sy = -10, -10
    gx, gy = 10, 10
    obstacle = []
    for i in range(30):
        obstacle.append((i-15, -15))
        obstacle.append((i-14, 15))
        obstacle.append((-15, i-14))
        obstacle.append((15, i-15))

    for i in range(3):
        obstacle.append((0,i))
        obstacle.append((0,-i))

    plt.plot(sx, sy, "xr")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")

    simple_a_star = a_star(-15, 15, -15, 15, obstacle=obstacle, \
        resolution=grid_size, robot_size=robot_size)
    path = simple_a_star.find_path((sx,sy), (gx,gy))
    print (path)

    rx, ry = [], []
    for node in path:
        rx.append(node[0])
        ry.append(node[1])

    plt.plot(rx, ry, "-r")
    plt.show()


if __name__ == '__main__':
    main()
