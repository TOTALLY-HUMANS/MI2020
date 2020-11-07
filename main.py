import socket
import time
import cv2

from robot import Robot
from game import Game
from shapely.geometry import Point

from detect_aruco_markers_from_image import get_robot_positions

IP = "127.0.0.1"
CONFIG_PORT = 3000
ROBOT_PORT_1 = 3001
ROBOT_PORT_2 = 3002
ROBOT_PORT_3 = 3003
ROBOT_PORT_4 = 3004

def reset(sock):
    sock.sendto(bytes("reset", "utf-8"), (IP, CONFIG_PORT))

def closest_ball_coords(robot_pos, ball_coords):
    if not ball_coords:
        return None, 0

    closest_ball_point = ball_coords[0]
    closest_dist = closest_ball_point.distance(robot_pos)
    for point in ball_coords:
        dist = point.distance(robot_pos)
        if dist < closest_dist:
            closest_dist = dist
            closest_ball_point = point
        
    return closest_ball_point, closest_dist


def robot_simple_logic(robot, game):
    goal = game.goal_left.centroid
    if robot.idx < 2:
        goal = game.goal_right.centroid

    target_ball, dist_to_ball = closest_ball_coords(robot.position, game.neg_core_positions)

    if game.goal_left.distance(robot.position) < 30:
        robot.drive_to_point(game.goal_right.centroid, 1.5)
        return
    if game.goal_right.distance(robot.position) < 30:
        robot.drive_to_point(game.goal_left.centroid, 1.5)


    if game.tick % 10 == 0:
        print(robot.position, robot.prev_pos)
        dist = robot.position.distance(robot.prev_pos)
        if dist < 10:
            robot.back(1.0)
            time.sleep(0.20)
            robot.prev_pos = Point(2000, 2000)
            return
        else:
            robot.prev_pos = robot.position

    if dist_to_ball < 90:
        robot.drive_to_point_smooth(goal, 1.0)
    else:
        robot.drive_to_point(target_ball, 1.5)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    capture = cv2.VideoCapture("http://localhost:8080")
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    reset(sock)

    width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

    print(width, height)

    game = Game()

    # Our robots
    r1 = Robot(sock, IP, ROBOT_PORT_1, "RC-1138 Boss", 2)
    r2 = Robot(sock, IP, ROBOT_PORT_2, "RC-1262 Scorch", 3)

    # Opponent robots
    r3 = Robot(sock, IP, ROBOT_PORT_3, "RC-1140 Fixer", 1)
    r4 = Robot(sock, IP, ROBOT_PORT_4, "RC-1207 Sev", 0)

    tick = 0
    while True:
        tick+=1
        robot_frame_data = get_robot_positions(capture)
        game.update(capture)

        game.neg_core_positions = game.remove_finished_cores(game.neg_core_positions)

        if len(robot_frame_data) < 4 or len(game.neg_core_positions) <= 0:
            print("SKIPPED")
            continue

        r1.update(robot_frame_data)
        r2.update(robot_frame_data)
        r3.update(robot_frame_data)
        r4.update(robot_frame_data)
        
        robot_simple_logic(r1, game)
        robot_simple_logic(r2, game)

        robot_simple_logic(r3, game)
        robot_simple_logic(r4, game)

        #time.sleep(0.05)


if __name__ == '__main__':
    main()
