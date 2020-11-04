import socket
import time
import random
import cv2
import numpy as np
import math

from detect_aruco_markers_from_image import get_robot_positions
from detect_energy_cores_from_image import get_core_positions


MAX_SPEED = 100
IP = "127.0.0.1"
CONFIG_PORT = 3000
ROBOT_PORT_1 = 3001
ROBOT_PORT_2 = 3002

GOAL_1 = [1080, 0]
GOAL_2 = [0, 1080]


class Robot:
    def __init__(self, sock, ip, port, name):
        self.name = name
        self.port = port
        self.ip = ip
        self.socket = sock

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

    def right(self, speed_percentage):
        left, right = 100, 50
        left = int(left * speed_percentage)
        right = int(right * speed_percentage)
        self.socket.sendto(bytes(f"{left};{right}", "utf-8"), (self.ip, self.port))

    def stop(self):
        left, right = 0, 0
        self.socket.sendto(bytes(f"{left};{right}", "utf-8"), (self.ip, self.port))


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reset(sock)
    capture = cv2.VideoCapture("http://localhost:8080")
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

    print(width, height)


    r1 = Robot(sock, IP, ROBOT_PORT_1, "R2D2")
    r2 = Robot(sock, IP, ROBOT_PORT_2, "BB8")

    r1_moves = [r1.left, r1.right, r1.forward, r1.back]
    r2_moves = [r2.left, r2.right, r2.forward, r2.back]


    for i in range(50000):
        #print(get_core_positions(capture))
        #print(get_robot_positions(capture))
        drive_to_point([GOAL_1], get_robot_positions(capture)[2], r1)
        drive_to_point([GOAL_2], get_robot_positions(capture)[3], r2)

    r1.stop()
    r2.stop()
    

def make_random_move(moves):
    speed = 0.5
    i = random.randint(0,len(moves)-1)
    move = moves[i]
    move(speed)

def reset(sock):
    sock.sendto(bytes("reset", "utf-8"), (IP, CONFIG_PORT))


#transforms point to robot coordinate frame. in robot frame positive x is forward and positive y is to left
def point2robotframe(point, robot_pose):
    yaw = (robot_pose['rotation'] - 90.0)* math.pi/180.0 
    robot_position = robot_pose['position']
    R = np.matrix([[math.cos(yaw) , -math.sin(yaw) ],[math.sin(yaw), math.cos(yaw)]])
    
    point_in_robot_frame = np.transpose(R)*np.transpose(np.matrix(point) - robot_position)
    point_in_robot_frame[1] =  point_in_robot_frame[1] * -1

    print(point_in_robot_frame)
    return point_in_robot_frame

#angle to point in robot frame
def get_angle_to_point(point):
    return math.atan2(point[1], point[0])

#dist to point in robot frame
def get_dist_to_point(point):
    return math.sqrt(point[0]**2 + point[1]**2)

#veeeery simple pure pursuit, drives to point in map coordinate
def drive_to_point(goal_point, my_pose, robot_handle):
        goal_in_robot_frame = point2robotframe(goal_point ,my_pose )
        angle = get_angle_to_point(goal_in_robot_frame) 
        dist = get_dist_to_point(goal_in_robot_frame)

        if angle > 0.2:
            robot_handle.left(0.3)
        elif angle < -0.2:
            robot_handle.right(0.3)
        else:
            robot_handle.forward(0.3)
        
        print('angle: ', angle)
        print("dist: " , dist)

if __name__ == '__main__':
    main()
