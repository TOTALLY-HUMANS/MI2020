import socket
import time
import random
import cv2
import numpy as np
import math

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


from detect_aruco_markers_from_image import get_robot_positions
from detect_energy_cores_from_image import get_core_positions


MAX_SPEED = 100
IP = "127.0.0.1"
CONFIG_PORT = 3000
ROBOT_PORT_1 = 3001
ROBOT_PORT_2 = 3002
ROBOT_PORT_3 = 3003
ROBOT_PORT_4 = 3004


class Robot:
    def __init__(self, sock, ip, port, name, idx):
        self.idx = idx
        self.name = name
        self.port = port
        self.ip = ip
        self.socket = sock
        self.prev_pos = Point(2000, 2000)

    def forward(self, speed_percentage):
        left, right = 100, 100
        left = int(left * speed_percentage)
        right = int(right * speed_percentage)
        self.socket.sendto(bytes(f"{left};{right}", "utf-8"), (self.ip, self.port))

    def back(self, speed_percentage):
        left, right = -100, -100
        left = int(left * speed_percentage)
        right = int(right * speed_percentage)
        self.socket.sendto(bytes(f"{left};{right}", "utf-8"), (self.ip, self.port))

    def left(self, speed_percentage):
        left, right = 50, 100
        left = int(left * speed_percentage)
        right = int(right * speed_percentage)
        self.socket.sendto(bytes(f"{left};{right}", "utf-8"), (self.ip, self.port))

    def tight_left(self, speed_percentage):
        left, right = -100, 100
        left = int(left * speed_percentage)
        right = int(right * speed_percentage)
        self.socket.sendto(bytes(f"{left};{right}", "utf-8"), (self.ip, self.port))

    def right(self, speed_percentage):
        left, right = 100, 50
        left = int(left * speed_percentage)
        right = int(right * speed_percentage)
        self.socket.sendto(bytes(f"{left};{right}", "utf-8"), (self.ip, self.port))

    def tight_right(self, speed_percentage):
        left, right = 100, -100
        left = int(left * speed_percentage)
        right = int(right * speed_percentage)
        self.socket.sendto(bytes(f"{left};{right}", "utf-8"), (self.ip, self.port))

    def stop(self):
        left, right = 0, 0
        self.socket.sendto(bytes(f"{left};{right}", "utf-8"), (self.ip, self.port))


GOAL_1 = Polygon([(1080, 0), (840, 0), (1080, 250)])
GOAL_2 = Polygon([(0, 1080), (0, 840), (250, 1080)])
    

def make_random_move(moves):
    speed = 0.5
    i = random.randint(0,len(moves)-1)
    move = moves[i]
    move(speed)

def reset(sock):
    sock.sendto(bytes("reset", "utf-8"), (IP, CONFIG_PORT))


#transforms point to robot coordinate frame. in robot frame positive x is forward and positive y is to left
def point2robotframe(point, robot_pose, rotation):
    yaw = (rotation - 90.0)* math.pi/180.0 
    robot_position = [robot_pose.x, robot_pose.y]
    R = np.matrix([[math.cos(yaw) , -math.sin(yaw) ],[math.sin(yaw), math.cos(yaw)]])
    
    point_in_robot_frame = np.transpose(R)*np.transpose(np.matrix([point.x, point.y]) - robot_position)
    point_in_robot_frame[1] =  point_in_robot_frame[1] * -1

    return Point(point_in_robot_frame[0], point_in_robot_frame[1])

#angle to point in robot frame
def get_angle_to_point(point):
    return math.atan2(point.y, point.x)

#dist to point in robot frame
def get_dist_to_point(point):
    return math.sqrt(point.x**2 + point.y**2)

def closest_ball_coords(robot_pos, robot_angle, ball_coords):
    closest_ball_point = ball_coords[0]
    _, closest_dist = get_angle_dist_to_point(closest_ball_point, robot_pos, robot_angle)
    for point in ball_coords:
        _, dist = get_angle_dist_to_point(point, robot_pos, robot_angle)
        if dist < closest_dist:
            closest_dist = dist
            closest_ball_point = point
        
    #print("CLOSEST", closest_ball_point, "DIST:", closest_dist)
    return closest_ball_point, closest_dist


def get_angle_dist_to_point(goal_point, my_pose, robot_angle):
    goal_in_robot_frame = point2robotframe(goal_point, my_pose, robot_angle)
    angle = get_angle_to_point(goal_in_robot_frame)
    dist = get_dist_to_point(goal_in_robot_frame)
    return angle, dist

#veeeery simple pure pursuit, drives to point in map coordinate
def drive_to_point(goal_point, robot_angle, my_pose, robot_handle, speed_multiplier):
    angle, dist = get_angle_dist_to_point(goal_point, my_pose, robot_angle)
    
    if angle > 1.:
        robot_handle.tight_left(0.2*speed_multiplier)
    elif angle < -1.:
        robot_handle.tight_right(0.2*speed_multiplier)
    elif angle > 0.30:
        robot_handle.left(0.2*speed_multiplier)
    elif angle < -0.30:
        robot_handle.right(0.2*speed_multiplier)
    else:
        robot_handle.forward(0.2*speed_multiplier)
    
    #print('angle: ', angle)
    #print('dist: ' , dist)


def robot_simple_logic(robot_handle, robot_positions, neg_core_positions, tick):
    
    goal = GOAL_1.centroid
    if robot_handle.idx < 2:
        goal = GOAL_2.centroid

    r_pos = robot_positions[robot_handle.idx]
    robot_position_point = Point(r_pos['position'][0], r_pos['position'][1])
    robot_angle = r_pos['rotation']
    target_ball, dist_to_ball = closest_ball_coords(robot_position_point, robot_angle, neg_core_positions)

    if GOAL_1.distance(robot_position_point) < 30:
        drive_to_point(GOAL_2.centroid, robot_angle, robot_position_point, robot_handle, speed_multiplier=1.5)
        return
    if GOAL_2.distance(robot_position_point) < 30:
        drive_to_point(GOAL_1.centroid, robot_angle, robot_position_point, robot_handle, speed_multiplier=1.5)


    if tick % 10 == 0:
        print(robot_position_point, robot_handle.prev_pos)
        dist = robot_position_point.distance(robot_handle.prev_pos)
        if dist < 10:
            robot_handle.back(1.0)
            time.sleep(0.20)
            robot_handle.prev_pos = Point(2000, 2000)
            return
        else:
            robot_handle.prev_pos = robot_position_point

    if dist_to_ball < 90:
        drive_to_point(goal, robot_angle, robot_position_point,
                       robot_handle, speed_multiplier=1.0)
    else:
        drive_to_point(target_ball, robot_angle, robot_position_point, robot_handle, speed_multiplier=1.5)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reset(sock)
    capture = cv2.VideoCapture("http://localhost:8080")
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

    print(width, height)

    r1 = Robot(sock, IP, ROBOT_PORT_1, "R2D2", 2)
    r2 = Robot(sock, IP, ROBOT_PORT_2, "BB8", 3)

    r3 = Robot(sock, IP, ROBOT_PORT_3, "Vader", 1)
    r4 = Robot(sock, IP, ROBOT_PORT_4, "Sidious", 0)

    for i in range(50000):
        #print("A", get_core_positions(capture)[0])
        #print(get_robot_positions(capture))
        robot_positions = get_robot_positions(capture)
        core_positions = get_core_positions(capture)

        
        neg_core_positions = array_coords_to_points(core_positions[0])
        neg_core_positions = remove_finished_cores(neg_core_positions)

        if len(robot_positions) < 4 or len(neg_core_positions) <= 0:
            print("SKIPPED")
            continue

        print(robot_positions)
        print(neg_core_positions)
        

        robot_simple_logic(r1, robot_positions, neg_core_positions, i)
        robot_simple_logic(r2, robot_positions, neg_core_positions, i)

        robot_simple_logic(r3, robot_positions, neg_core_positions, i)
        robot_simple_logic(r4, robot_positions, neg_core_positions, i)

        #time.sleep(0.05)

    r1.stop()
    r2.stop()

def array_coords_to_points(array_coords):
    points = []
    for coord in array_coords:
        points.append(Point(coord[0], coord[1]))
    return points


def remove_finished_cores(ecore_positions):
    filtered = []
    for point in ecore_positions:
        if GOAL_1.contains(point) or GOAL_2.contains(point):
            print("REMOVED", point)
            continue
        filtered.append(point)
        
    return filtered

if __name__ == '__main__':
    main()