import socket
import time
import random
import cv2

from detect_aruco_markers_from_image import get_robot_positions
from detect_energy_cores_from_image import get_core_positions


MAX_SPEED = 100
ROBOT_IP = "127.0.0.1"
ROBOT_PORT_1 = 3001
ROBOT_PORT_2 = 3002


class Robot:
    def __init__(self, ip, port, name):
        self.name = name
        self.port = port
        self.ip = ip
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

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
    capture = cv2.VideoCapture("http://localhost:8080")

    r1 = Robot(ROBOT_IP, ROBOT_PORT_1, "R2D2")
    r2 = Robot(ROBOT_IP, ROBOT_PORT_2, "BB8")

    r1_moves = [r1.left, r1.right, r1.forward, r1.back]
    r2_moves = [r2.left, r2.right, r2.forward, r2.back]


    for i in range(100):
        print(get_core_positions(capture))
        print(get_robot_positions(capture))
        make_random_move(r1_moves)
        make_random_move(r2_moves)
        time.sleep(0.2)

    r1.stop()
    r2.stop()

def make_random_move(moves):
    speed = 0.5
    i = random.randint(0,len(moves)-1)
    move = moves[i]
    move(speed)


if __name__ == '__main__':
    main()