import socket
import time
import cv2

from robot import Robot
from game import Game
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


#GOAL_RIGHT = Polygon([(0, 0), (280, 0), (0, 280)])
#GOAL_LEFT = Polygon([(1080, 1080), (1080, 800), (800, 1080)])

GOAL_RIGHT = Polygon([(1080, 0), (800, 0), (1080, 280)])
GOAL_LEFT = Polygon([(0, 1080), (0, 800), (280, 1080)])
GAME_AREA = Polygon([(0,0), (0, 1080), (1080, 0), (1080, 1080)])

#VIDEO_FEED = "rtp://224.1.1.1:5200"
VIDEO_FEED = "http://localhost:8080"

IP = "127.0.0.1"
CONFIG_PORT = 3000
ROBOT_PORT_1 = 3001
ROBOT_PORT_2 = 3002
ROBOT_PORT_3 = 3003
ROBOT_PORT_4 = 3004

#ROBO_SPEED = 0.75
ROBO_SPEED = 0.3

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
    for point in ball_coords:
        dist = point.distance(target_pos)
        if dist < closest_dist and point not in targeted:
            closest_dist = dist
            closest_ball_point = point

    if closest_ball_point is None:
        closest_ball_point = ball_coords[0]
        closest_dist = ball_coords[0].distance(target_pos)
        
    return closest_ball_point, closest_dist


def unstuck_logic(robot, game):
    if (game.tick - robot.prev_unstuck_tick) > 15:
        robot.prev_unstuck_tick = game.tick
        if robot.unstuck_counter < 0 and robot.position.distance(robot.prev_pos) < 10 and robot.unstuck_cooldown <= game.tick:
            robot.unstuck_counter = 2
            robot.unstuck_cooldown = game.tick + 10

    if robot.unstuck_counter >= 0:
        print("trying to unstuck")
        robot.jam_turn(robot.goal, ROBO_SPEED)
        robot.unstuck_counter -= 1
        return True

    robot.prev_pos = robot.position
    return False

def is_stuck(robot, game):

    if robot.prev_pos is not None and robot.position.distance(robot.prev_pos) < 10:
        robot.prev_pos = None
    else:
        robot.prev_pos = robot.position
            
    if robot.prev_pos == None:
        return True

    return False

def robot_simple_logic(robot, game):
    robot.target_core = None

    if robot.target_core_type == -1:
        target_ball, dist_to_ball = select_core_logic(game, robot, game.get_cores_not_in_goal(game.neg_core_positions))
        robot.goal = game.goal_opponent.centroid

    elif robot.target_core_type == 1:
        target_ball, dist_to_ball = select_core_logic(game, robot, game.get_cores_not_in_goal(game.pos_core_positions))
        robot.goal = game.goal_own.centroid


    if target_ball == None and game.tick > 15:
        robot.target_core_type = -1*robot.target_core_type
        return

    
    #target_ball, dist_to_ball = select_core_logic(game, robot, game.get_cores_not_in_goal(game.neg_core_positions))
    robot.target_core = target_ball



    is_stuck = unstuck_logic(robot, game)
    if is_stuck:
        return

    if game.goal_own.distance(robot.position) < 40:
        robot.drive_to_point(game.goal_opponent.centroid, ROBO_SPEED)
        return
    if game.goal_opponent.distance(robot.position) < 40:
        robot.drive_to_point(game.goal_own.centroid, ROBO_SPEED)
        return

    if dist_to_ball < 100:
        robot.drive_to_point(robot.goal, ROBO_SPEED)
    
    elif dist_to_ball < 180:
        robot.drive_to_point(target_ball, ROBO_SPEED)
    else:
        goto_approach_state(robot, game)

def idle_state(robot, game):
    robot.stop()
    robot.prev_state = 0

    # select new goal ball
    # change state to goto_approach
    if game.tick % 5 == 0:
        print("approach state")

        robot.state = 1


def goto_approach_state(robot, game):
    #set controls etc to correspond to this state
    #drive to approach point of current goal ball

    #print("approach")

    if (game.tick - robot.prev_tick) > 10 or robot.prev_state == 0:
        robot.prev_tick = game.tick
        if robot.target_core_type == -1:
            target_ball, dist_to_ball = select_core_logic(game, robot, game.get_cores_not_in_goal(game.neg_core_positions))
        elif robot.target_core_type == 1:
            target_ball, dist_to_ball = select_core_logic(game, robot, game.get_cores_not_in_goal(game.pos_core_positions))

        robot.target_core = target_ball # target_ball #side_behind_point[1]
        print("selected: ", target_ball, " dist2selected: ", dist_to_ball)

    if robot.target_core_type == 1:
        _, sides, end = points_side_behind(game.goal_own.centroid, robot.target_core, dists=[150,150,150])
    elif robot.target_core_type == -1:
        _, sides, end = points_side_behind(game.goal_opponent.centroid, robot.target_core, dists=[150,150,150])


    target = end
    if len(sides) > 0:
        target = sides[0]
        
    #selected_driving_point = select_point_next_to_core(robot.target_core, target)
    robot.drive_to_point(target, ROBO_SPEED)
    # print("robot pos: ", robot.position)
    # print("target: ",robot.target_core)
    # print("approach: ", target)
    # print("dist to target: ", robot.position.distance(target))


    robot.prev_state = 1
    if robot.position.distance(target) < 50:
        print("goto_behind state")
        robot.state = 2



    

    #if we are close enough, change to goto_behind_state

def select_point_next_to_core(ecore_position, points_next_to_ecore):
    filtered = []
    for point in points_next_to_ecore:
        point_in_game = GAME_AREA.contains(point) 
        if not point_in_game:
            filtered.append(point)
        else:
            #print("REMOVED", point)
            pass

    if not filtered:
        ecore_position

    return filtered[0]
    
def goto_behind_state(robot, game):
    #drive to behind point

    if robot.target_core_type == 1:
        behind, _, _ = points_side_behind(game.goal_own.centroid, robot.target_core, dists=[150,150,150])
    elif robot.target_core_type == -1:
        behind, _, _ = points_side_behind(game.goal_opponent.centroid, robot.target_core, dists=[150,150,150])

    robot.drive_to_point(behind, ROBO_SPEED)
    # print("robot pos: ", robot.position)
    # print("target: ", robot.target_core)
    # print("behind: ", behind)

    robot.prev_state = 2

    if robot.position.distance(behind) < 80:
        print("ram state")
        robot.state = 3

    


def ram_goal_state(robot, game):
    #drive full speed to goal pushing the ball with us
    if robot.target_core_type == 1:
        robot.target_core = game.goal_own.centroid
    elif robot.target_core_type == -1:
        robot.target_core = game.goal_opponent.centroid

    robot.drive_to_point(robot.target_core, ROBO_SPEED)

    #if we are close enough goal, change to idle state
    if robot.position.distance(robot.target_core) < 100:
        robot.state = 0


def jammed(robot, game):
    print("jammed state")
    #do something
    robot.tight_right(ROBO_SPEED)

    #go to idle
    robot.state = 0
    robot.prev_state = 4
    
    
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


states = {0: idle_state,
          1: goto_approach_state,
          2: goto_behind_state,
          3: ram_goal_state, 
          4: jammed,}

def new_robot_logic(robot, game):
    #check if stuck
    if (game.tick - robot.prev_tick) > 25 and is_stuck(robot, game) and robot.state != 0:
        robot.prev_tick = game.tick
        robot.state = 4
        
    states[robot.state](robot, game)


def game_tick_new(capture, game):
    try:
        game.update(capture)

        if len(game.get_cores_not_in_goal(game.neg_core_positions)) <= 0:
            print("SKIPPED")
            return

        for robot in game.team_robots:
            #robot_simple_logic(robot, game)
            new_robot_logic(robot, game)
    except Exception as e:
        print(e, e.with_traceback)
        pass

def game_tick(capture, game):
    try:
        game.update(capture)

        if len(game.get_cores_not_in_goal(game.neg_core_positions)) <= 0 and len(game.get_cores_not_in_goal(game.pos_core_positions)) <= 0:
            print("SKIPPED")
            return

        for robot in game.team_robots:
                robot_simple_logic(robot, game)
                #new_robot_logic(robot, game)
    except Exception as e:
        print(e, e.with_traceback)
        pass

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    capture = cv2.VideoCapture(VIDEO_FEED)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    reset(sock)

    width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

    print(width, height)

    # Opponent robots
    r1 = Robot(sock, IP, ROBOT_PORT_1, "RC-1138 Boss", 2)
    r2 = Robot(sock, IP, ROBOT_PORT_2, "RC-1262 Scorch", 3)

    # Our robots
    r3 = Robot(sock, IP, ROBOT_PORT_3, "RC-1140 Fixer", 8)
    r4 = Robot(sock, IP, ROBOT_PORT_4, "RC-1207 Sev", 9)

    r3.target_core_type = -1
    r4.target_core_type = 1


    r2.target_core_type = -1
    r1.target_core_type = 1

    game_1 = Game([r1, r2], GOAL_LEFT, GOAL_RIGHT)
    game_2 = Game([r3, r4], GOAL_RIGHT, GOAL_LEFT)

    while True:
        #game_tick_new(capture, game_1)
        game_tick(capture, game_2)

        #time.sleep(0.05)

if __name__ == '__main__':
    main()
