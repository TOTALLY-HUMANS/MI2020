import socket
import time
import cv2

from robot import Robot
from game import Game
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

ROBO_SPEED = 0.55
#ROBO_SPEED = 0.3

#GOAL_RIGHT = Polygon([(0, 0), (280, 0), (0, 280)])
#GOAL_LEFT = Polygon([(1080, 1080), (1080, 800), (800, 1080)])
#GOAL_RIGHT_CORNER = Point(0, 0)
#GOAL_LEFT_CORNER = Point(1080, 1080)

GOAL_RIGHT = Polygon([(1080, 0), (800, 0), (1080, 280)])
GOAL_RIGHT_CORNER = Point(1080, 0)
GOAL_LEFT = Polygon([(0, 1080), (0, 800), (280, 1080)])
GOAL_LEFT_CORNER = Point(0, 1080)

VIDEO_FEED = "rtp://224.1.1.1:5200"
#VIDEO_FEED = "http://localhost:8080"

IP = "127.0.0.1"
CONFIG_PORT = 3000
ROBOT_PORT_1 = 3001
ROBOT_PORT_2 = 3002
ROBOT_PORT_3 = 3003
ROBOT_PORT_4 = 3004

GAME_AREA = Polygon([(0,0), (0, 1080), (1080, 0), (1080, 1080)])

def reset(sock):
    sock.sendto(bytes("reset", "utf-8"), (IP, CONFIG_PORT))
    

def select_core_logic(game, robot, ball_coords):
    target_pos = robot.position
    if not ball_coords:
        return None, 0

    targeted = []
    for robot in game.team_robots:
        targeted.append(robot.target_core)

    closest_ball_point = None
    closest_dist = 10000000
    closest_angle = 10000000
    best_score = -100000
    for point in ball_coords:
        dist = point.distance(target_pos)
        angle = abs(robot.get_angle_to_point(point, robot.position, robot.angle))
        score = closest_ball_objective_funciton(dist, angle, robot.position.distance(robot.goal.centroid))
        if score > best_score and point not in targeted:
            best_score = score
            closest_angle = angle
            closest_dist = dist
            closest_ball_point = point

    if closest_ball_point is None:
        closest_ball_point = ball_coords[0]
        closest_dist = ball_coords[0].distance(target_pos)
    

    return closest_ball_point, closest_dist

def closest_ball_objective_funciton(dist, angle, dist_to_goal):
    dist_scaled = -(dist / 1080.)
    dist_to_goal_scaled = -(dist_to_goal / 1080.)
    angle_scaled = -angle / 3.14
    return 2 * dist_scaled + angle_scaled

def unstuck_logic(robot, game):
    if (game.tick - robot.prev_unstuck_tick) > 35 and game.tick > 50:
        robot.prev_unstuck_tick = game.tick
        if robot.unstuck_counter < 0 and robot.position.distance(robot.prev_pos) < 10 and robot.unstuck_cooldown <= game.tick:
            robot.unstuck_counter = 3
            robot.unstuck_cooldown = game.tick + 100

    if robot.unstuck_counter >= 0:
        print("trying to unstuck")
        robot.jam_turn(robot.goal, 1.0)
        robot.unstuck_counter -= 1
        return True

    robot.prev_pos = robot.position
    return False

def robot_simple_logic(robot, game):
    robot.target_core = None
    target_ball = None

    if robot.target_core_type == -1:
        robot.goal = game.goal_opponent_corner
        target_ball, dist_to_ball = select_core_logic(game, robot, game.get_cores_not_in_goal(game.neg_core_positions))

    elif robot.target_core_type == 1:
        robot.goal = game.goal_own_corner
        target_ball, dist_to_ball = select_core_logic(game, robot, game.get_cores_not_in_goal(game.pos_core_positions))
    
    # ENABLE?
    #robot.target_core = target_ball

    #print(target_ball, robot.target_core_type)

    if target_ball == None:
        robot.none_counter -= 1
    else:
        robot.none_counter = 35
    
    if robot.none_counter < 0:
        robot.target_core_type * -1
        robot.none_counter = 10000000000
        print("SWITCH", robot.idx)

    is_stuck = unstuck_logic(robot, game)
    if is_stuck:
        return

    if game.goal_own.distance(robot.position) < 40:
        robot.drive_to_point(game.goal_opponent.centroid, ROBO_SPEED, 0.65)
        return
    if game.goal_opponent.distance(robot.position) < 40:
        robot.drive_to_point(game.goal_own.centroid, ROBO_SPEED, 0.65)
        return

    if dist_to_ball < 80:
        print("to goal")
        robot.drive_to_point_smooth(robot.goal, ROBO_SPEED)
    
    elif dist_to_ball < 250:
        print("approach ball")
        robot.drive_to_point(target_ball, ROBO_SPEED * 0.75, 0.75)
    else:
        goto_approach_state(robot, game)

def goto_approach_state(robot, game):
    #set controls etc to correspond to this state
    #drive to approach point of current goal ball

    print("approach long")

    if (game.tick - robot.prev_tick) > 10 or robot.target_core == None:
        robot.prev_tick = game.tick
        if robot.target_core_type == -1:
            target_ball, dist_to_ball = select_core_logic(game, robot, game.get_cores_not_in_goal(game.neg_core_positions))
        elif robot.target_core_type == 1:
            target_ball, dist_to_ball = select_core_logic(game, robot, game.get_cores_not_in_goal(game.pos_core_positions))

        robot.target_core = target_ball # target_ball #side_behind_point[1]
        #print("selected: ", target_ball, " dist2selected: ", dist_to_ball)

    if robot.target_core_type == 1:
        _, sides, end = points_side_behind(game.goal_own.centroid, robot.target_core, dists=[150,150,150])
    elif robot.target_core_type == -1:
        _, sides, end = points_side_behind(game.goal_opponent.centroid, robot.target_core, dists=[150,150,150])


    target = end
    if len(sides) > 0:
        target = sides[0]
        
    robot.drive_to_point(target, ROBO_SPEED, 0.65)

    
#computes unit vector between start&end
#uses this unit vector and its normal to
#compute points behind and both sides of the end
#start and end are xy-point, dists is list in form [dist_behind, dist_side1, dist_side2]
#return list of xy-points
def points_side_behind(start, end, dists=[10, 10, 10]):
    dist = start.distance(end)
    vec = [(end.x - start.x)/dist, (end.y - start.y)/dist]

    behind = Point(end.x + vec[0]*dists[0], end.y + vec[1]*dists[0])
    side1 = Point(end.x - vec[1]*dists[1], end.y + vec[0]*dists[1])
    side2 = Point(end.x + vec[1]*dists[2], end.y - vec[0]*dists[2])

    sides = [side1, side2]
    filtered_sides = []

    for p in sides:
        if GAME_AREA.contains(p):
            filtered_sides.append(p)

    return behind, filtered_sides, end

def game_tick(capture, game):
    for robot in game.team_robots:
        try:
            game.update(capture)

            if len(game.get_cores_not_in_goal(game.neg_core_positions)) <= 0 and len(game.get_cores_not_in_goal(game.pos_core_positions)) <= 0:
                print("SKIPPED")
                return

            robot_simple_logic(robot, game)
        except Exception as e:
            print(robot.idx, e, e.with_traceback)
            pass

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    capture = cv2.VideoCapture(VIDEO_FEED)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    reset(sock)

    width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

    print(width, height)

    # Our robots
    #r8 = Robot(sock, IP, ROBOT_PORT_3, "RC-1140 Fixer", 8)
    #r9 = Robot(sock, IP, ROBOT_PORT_4, "RC-1207 Sev", 9)

    r9 = Robot(sock, "192.168.1.61", 3000, "RC-1140 Fixer", 9)
    r8 = Robot(sock, "192.168.1.62", 3000, "RC-1207 Sev", 8)

    r8.target_core_type = 1
    r9.target_core_type = -1
    
    game_A = Game([r8, r9], GOAL_RIGHT, GOAL_RIGHT_CORNER, GOAL_LEFT, GOAL_LEFT_CORNER)
    game_B = Game([r8, r9], GOAL_LEFT, GOAL_LEFT_CORNER, GOAL_RIGHT, GOAL_RIGHT_CORNER)


    while True:
        game_tick(capture, game_B)

if __name__ == '__main__':
    main()
