
import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from detect_energy_cores_from_image import get_core_positions



class Ball:
    def __init__(self, pos, index):
        self.pos = pos
        self.index = index
        self.removed = False

    def get_index(self):
        return self.index

    def is_removed(self):
        return self.removed
    
    def remove(self):
        self.removed = True

    def get_pos(self):
        return self.pos
    
    def update_pos(self, pos):
        self.pos = pos

class Balls:
    def __init__(self):
        self.pos_balls = []
        self.neg_balls = []
        self.inited = False

    def update(self, capture):
        core_positions = get_core_positions(capture)
        neg_core_positions = array_coords_to_points(core_positions[0])
        pos_core_positions = array_coords_to_points(core_positions[1])

        #print(self.inited)
        #print("neg_balls", len(self.neg_balls))
        #print("pos_balls", len(self.pos_balls))
        #print(len(neg_core_positions))
        if not self.inited:

            for idx,p in enumerate(neg_core_positions):
                self.neg_balls.append(Ball(p, idx))
            for idx,p in enumerate(pos_core_positions):
                self.pos_balls.append(Ball(p, idx))
            self.inited = True
        else:
            for p in neg_core_positions:
                self.associate_with_nearest(p, -1)

            for p in pos_core_positions:
                self.associate_with_nearest(p, 1)

    def associate_with_nearest(self, point, etype):
        if etype == 1:
            idx = self.nearest_idx(point, self.pos_balls)
            self.pos_balls[idx].update_pos(point)
        elif etype == -1:
            idx = self.nearest_idx(point, self.neg_balls)
            self.neg_balls[idx].update_pos(point)

    def nearest_idx(self, point, ball_list):
        dist = 2000000
        idx = -1
        print(len(ball_list))
        for i,b in enumerate(ball_list):
            tmp_dist = point.distance(b.get_pos())
            if tmp_dist < dist:
                dist = tmp_dist
                idx = i
        return idx

    def get_neg_balls(self, get_all=True):
        if get_all :
            return self.neg_balls
        else:
            balls = []
            for b in self.neg_balls:
                if(not b.is_removed()):
                    balls.append(b)
            return balls

    def get_pos_balls(self, get_all=True):
        if get_all:
            return self.pos_balls
        else:
            balls = []
            for b in self.pos_balls:
                if(not b.is_removed()):
                    balls.append(b)
            return balls



def array_coords_to_points(array_coords):
    points = []
    for coord in array_coords:
        points.append(Point(coord[0], coord[1]))
    return points