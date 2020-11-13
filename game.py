from shapely.geometry.polygon import Polygon
from detect_energy_cores_from_image import get_core_positions
from detect_aruco_markers_from_image import get_robot_positions
from shapely.geometry import Point
from ball import Balls


import numpy as np
from matplotlib import pyplot as plt


class Game:
    def __init__(self, robots, goal_own, goal_opponent):
        self.neg_core_positions = []
        self.pos_core_positions = []
        #self.balls = Balls()

        self.team_robots = robots

        self.goal_own = goal_own
        self.goal_opponent = goal_opponent

        self.tick = 0



    def update(self, capture):
        robot_frame_data = get_robot_positions(capture)
        for robot in self.team_robots:
            robot.update(robot_frame_data)

        core_positions = get_core_positions(capture)
        #print(core_positions)
        self.neg_core_positions = array_coords_to_points(core_positions[0])
        self.pos_core_positions = array_coords_to_points(core_positions[1])

        #self.pos_core_positions = []
        self.tick += 1

        #if(self.tick > 10):
        #    self.balls.update(capture) # tries to maintain same index for same ball
        #    self.balls.get_neg_balls()
            #self.balls.get_pos_balls()
        #for b in self.balls.get_neg_balls():
        #    print("ball idx: ", b.get_index(), " ball pos: ", b.get_pos())



    def get_cores_not_in_goal(self, ecore_positions):
        filtered = []
        for point in ecore_positions:
            point_in_goal = self.goal_own.contains(point) or self.goal_opponent.contains(point)
            if not point_in_goal:
                filtered.append(point)
            else:
                #print("REMOVED", point)
                pass

        return filtered
    

def array_coords_to_points(array_coords):
    points = []
    for coord in array_coords:
        points.append(Point(coord[0], coord[1]))
    return points
