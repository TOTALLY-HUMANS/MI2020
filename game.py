from shapely.geometry.polygon import Polygon
from detect_energy_cores_from_image import get_core_positions
from detect_aruco_markers_from_image import get_robot_positions
from shapely.geometry import Point
from ball import Balls


import numpy as np
from matplotlib import pyplot as plt

#plt.axis([-50,50,0,10000])


class Game:
    def __init__(self, robots, goal_own, goal_opponent):
        self.neg_core_positions = []
        self.pos_core_positions = []
        self.balls = Balls()

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
        #self.pos_core_positions = array_coords_to_points(core_positions[1])
        self.pos_core_positions = []
        self.tick += 1

        if(self.tick > 10):
            self.balls.update(capture) # tries to maintain same index for same ball
            self.balls.get_neg_balls()
            #self.balls.get_pos_balls()
        for b in self.balls.get_neg_balls():
            print("ball idx: ", b.get_index(), " ball pos: ", b.get_pos())



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

#computes unit vector between start&end
#uses this unit vector and its normal to
#compute points behind and both sides of the end
#start and end are xy-point, dists is list in form [dist_behind, dist_side1, dist_side2]
#return list of xy-points
def points_side_behind(start, end, dists=[10,10,10]):
    dist = start.distance(end)
    vec = [ (end.x - start.x)/dist, (end.y - start.y)/dist ]
    
    behind = Point( end.x + vec[0]*dists[0], end.y + vec[1]*dists[0] )
    side1 = Point( end.x - vec[1]*dists[1], end.y + vec[0]*dists[1] )
    side2 = Point( end.x + vec[1]*dists[2], end.y - vec[0]*dists[2] )
    
    return [behind, side1, side2]