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
        self.target_core = Point(2000, 2000)
        self.target_core_idx = 0
        self.target_core_type = -1
        self.state = 1 # 0=idle, 1=goto_approach, 2=goto_behind, 3=ram_goal, 4=jammed
        self.prev_state = 0
        self.prev_tick = -100
        self.goal = Point(500,500)
        self.unstuck_counter = -1
        self.unstuck_cooldown = -1000

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
        self._send_move(0, 0)

    def _send_move(self, left, right):
        self.socket.sendto(
            bytes(f"{int(left)};{int(right)}", "utf-8"), (self.ip, self.port))

    def update(self, robot_frame_data):
        r_pos = robot_frame_data[self.idx]
        self.position = Point(r_pos['position'][0], r_pos['position'][1])
        self.angle = r_pos['rotation']

    def drive_to_point(self, target, speed_percent):
        speed = np.clip(speed_percent, 0.0, 1.0)
        angle = self.get_angle_to_point(target, self.position, self.angle)
        if angle > 1.:
            self.tight_left(speed * 0.75)
        elif angle < -1.:
            self.tight_right(speed * 0.75)
        elif angle > 0.35:
            self.left(speed)
        elif angle < -0.35:
            self.right(speed)
        else:
            self.forward(speed)

    def jam_turn(self, target, speed_percent):
        speed = np.clip(speed_percent, 0.0, 1.0)
        angle = self.get_angle_to_point(target, self.position, self.angle)
        if angle > 0:
            self.tight_left(speed)
        elif angle < 0.:
            self.tight_right(speed)
   

    def drive_to_point_smooth(self, target, speed_percent):
        speed = np.clip(speed_percent, 0.0, 1.0)
        angle = self.get_angle_to_point(target, self.position, self.angle)
        if angle > 0.40:
            self.left(speed)
        elif angle < -0.40:
            self.right(speed)
        else:
            self.forward(speed)

    #transforms point to robot coordinate frame. in robot frame positive x is forward and positive y is to left
    def get_angle_to_point(self, point, robot_pose, angle):
        yaw = (angle - 90.0) * math.pi/180.0
        robot_position = [robot_pose.x, robot_pose.y]
        R = np.matrix([[math.cos(yaw), -math.sin(yaw)],[math.sin(yaw), math.cos(yaw)]])
        point_in_robot_frame = np.transpose(R)*np.transpose(np.matrix([point.x, point.y]) - robot_position)
        point_in_robot_frame[1] = point_in_robot_frame[1] * -1
        x, y = point_in_robot_frame[0], point_in_robot_frame[1]
        return math.atan2(y, x)
