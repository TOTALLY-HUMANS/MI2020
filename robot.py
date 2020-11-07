from shapely.geometry import Point
import numpy as np
import math

MAX_SPEED = 100

class Robot:
    def __init__(self, sock, ip, port, name, idx):
        self.idx = idx
        self.name = name
        self.port = port
        self.ip = ip
        self.socket = sock
        self.prev_pos = Point(2000, 2000)
        self.position = Point(0.0, 0.0)
        self.angle = 0.0

    def forward(self, speed_percentage):
        self._send_move(MAX_SPEED * speed_percentage, MAX_SPEED * speed_percentage)

    def back(self, speed_percentage):
        self._send_move(-MAX_SPEED * speed_percentage, -
                        MAX_SPEED * speed_percentage)

    def left(self, speed_percentage):
        self._send_move(MAX_SPEED * 0.5 * speed_percentage,
                        MAX_SPEED * speed_percentage)

    def tight_left(self, speed_percentage):
        self._send_move(-MAX_SPEED * speed_percentage,
                        MAX_SPEED * speed_percentage)

    def right(self, speed_percentage):
        self._send_move(MAX_SPEED * speed_percentage, -
                        MAX_SPEED * 0.5 * speed_percentage)

    def tight_right(self, speed_percentage):
        self._send_move(MAX_SPEED * speed_percentage, -
                        MAX_SPEED * speed_percentage)

    def stop(self):
        self._send_move(self, 0, 0)

    def _send_move(self, left, right):
        self.socket.sendto(
            bytes(f"{int(left)};{int(right)}", "utf-8"), (self.ip, self.port))

    def update(self, robot_frame_data):
        r_pos = robot_frame_data[self.idx]
        self.position = Point(r_pos['position'][0], r_pos['position'][1])
        self.angle = r_pos['rotation']

    def drive_to_point(self, target, speed_multiplier):
        angle, dist = get_angle_dist_to_point(target, self.position, self.angle)
        if angle > 1.:
            self.tight_left(0.2*speed_multiplier)
        elif angle < -1.:
            self.tight_right(0.2*speed_multiplier)
        elif angle > 0.30:
            self.left(0.2*speed_multiplier)
        elif angle < -0.30:
            self.right(0.2*speed_multiplier)
        else:
            self.forward(0.2*speed_multiplier)

    def drive_to_point_smooth(self, target, speed_multiplier):
        angle, dist = get_angle_dist_to_point(target, self.position, self.angle)
        if angle > 0.40:
            self.left(0.2*speed_multiplier)
        elif angle < -0.40:
            self.right(0.2*speed_multiplier)
        else:
            self.forward(0.2*speed_multiplier)

def get_angle_dist_to_point(goal_point, my_pose, robot_angle):
    goal_in_robot_frame = point2robotframe(goal_point, my_pose, robot_angle)
    angle = get_angle_to_point(goal_in_robot_frame)
    dist = get_dist_to_point(goal_in_robot_frame)
    return angle, dist

#transforms point to robot coordinate frame. in robot frame positive x is forward and positive y is to left


def point2robotframe(point, robot_pose, rotation):
    yaw = (rotation - 90.0) * math.pi/180.0
    robot_position = [robot_pose.x, robot_pose.y]
    R = np.matrix([[math.cos(yaw), -math.sin(yaw)],
                   [math.sin(yaw), math.cos(yaw)]])

    point_in_robot_frame = np.transpose(
        R)*np.transpose(np.matrix([point.x, point.y]) - robot_position)
    point_in_robot_frame[1] = point_in_robot_frame[1] * -1

    return Point(point_in_robot_frame[0], point_in_robot_frame[1])


#angle to point in robot frame
def get_angle_to_point(point):
    return math.atan2(point.y, point.x)

#dist to point in robot frame
def get_dist_to_point(point):
    return math.sqrt(point.x**2 + point.y**2)
