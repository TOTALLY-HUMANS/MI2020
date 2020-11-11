import socket
import time
import cv2

from robot import Robot
from game import Game
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from game import points_side_behind


#GOAL_RIGHT = Polygon([(0, 0), (280, 0), (0, 280)])
#GOAL_LEFT = Polygon([(1080, 1080), (1080, 800), (800, 1080)])

GOAL_RIGHT = Polygon([(1080, 0), (800, 0), (1080, 280)])
GOAL_LEFT = Polygon([(0, 1080), (0, 800), (280, 1080)])

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

prev_tick = 0


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
    if game.tick % 10 == 0:
        if robot.prev_pos is not None and robot.position.distance(robot.prev_pos) < 10:
            robot.prev_pos = None
        else:
            robot.prev_pos = robot.position

    if robot.prev_pos == None:
        robot.tight_right(ROBO_SPEED)
        return True

    return False


def robot_simple_logic(robot, game):
    robot.target_core = None
    target_ball, dist_to_ball = select_core_logic(game, robot, game.get_cores_not_in_goal(game.neg_core_positions))
    robot.target_core = target_ball


    is_stuck = unstuck_logic(robot, game)
    if is_stuck:
        return

    if game.goal_own.distance(robot.position) < 40:
        robot.drive_to_point(game.goal_opponent.centroid, ROBO_SPEED)
        return
    if game.goal_opponent.distance(robot.position) < 40:
        robot.drive_to_point(game.goal_own.centroid, ROBO_SPEED)

    if dist_to_ball < 90:
        robot.drive_to_point_smooth(game.goal_opponent.centroid, ROBO_SPEED)
    else:
        robot.drive_to_point(target_ball, ROBO_SPEED)

def idle_state(robot, game):
    robot.stop()
    robot.prev_state = 0

    print("idle state")
    # select new goal ball
    # change state to goto_approach
    if game.tick % 10 == 0:
        robot.state = 1


def goto_approach_state(robot, game):
    #set controls etc to correspond to this state
    #drive to approach point of current goal ball
    print("approach state")

    global prev_tick


    print("tick: ",game.tick)
    print("tick diff: ",game.tick - prev_tick)
    print("tick condition: ",(game.tick - prev_tick) > 10)
    if (game.tick - prev_tick) > 10 and robot.prev_state == 0:
        prev_tick = game.tick
        target_ball, dist_to_ball = select_core_logic(game, robot, game.get_cores_not_in_goal(game.neg_core_positions))
        print("selected: ", target_ball, " dist2selected: ",dist_to_ball)
        robot.target_core = target_ball # target_ball #side_behind_point[1]
        for p in game.get_cores_not_in_goal(game.neg_core_positions):
            print(p)
        print("target updated")

    side_behind_point =  points_side_behind(game.goal_own.centroid, robot.target_core, dists=[150,150,150])
    robot.drive_to_point(side_behind_point[1], ROBO_SPEED)
    print("robot pos: ", robot.position)
    print("target: ",robot.target_core)
    print("approach: ", side_behind_point[1])

    robot.prev_state = 1
    print("dist to target: ", robot.position.distance(side_behind_point[1]))
    if robot.position.distance(side_behind_point[1]) < 50:
        robot.state = 2


    #if we are close enough, change to goto_behind_state
    
def goto_behind_state(robot, game):
    #drive to behind point
    print("goto_behind state")


    side_behind_point =  points_side_behind(game.goal_own.centroid, robot.target_core, dists=[150,150,150])
    robot.drive_to_point(side_behind_point[0], ROBO_SPEED)
    print("robot pos: ", robot.position)
    print("target: ",robot.target_core)
    print("behind: ", side_behind_point[0])

    robot.prev_state = 2

    if robot.position.distance(side_behind_point[0]) < 50:
        robot.state = 3

    
    #if we are close enough, change to ram_goal_state
    

def ram_goal_state(robot, game):
    #drive full speed to goal pushing the ball with us
    print("ram state")
    print("robot pos: ", robot.position)
    print("target: ",robot.target_core)
    robot.target_core = game.goal_own.centroid
    robot.drive_to_point(robot.target_core, ROBO_SPEED)

    #if we are close enough goal, change to idle state
    if robot.position.distance(robot.target_core) < 80:
        robot.state = 0
    


def new_robot_logic(robot, game):
    #get balls
    state = robot.state

    states = {  0 : idle_state,
                1 : goto_approach_state,
                2 : goto_behind_state,
                3 : ram_goal_state,}

    states[robot.state](robot, game)




def game_tick(capture, game):
    try:
        game.update(capture)

        if len(game.get_cores_not_in_goal(game.neg_core_positions)) <= 0:
            print("SKIPPED")
            return

        for robot in game.team_robots:
                #robot_simple_logic(robot, game)
                new_robot_logic(robot, game)
                break
    except Exception as e:
        #print(e, e.with_traceback)
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

    game_1 = Game([r1, r2], GOAL_LEFT, GOAL_RIGHT)
    game_2 = Game([r3, r4], GOAL_RIGHT, GOAL_LEFT)

    while True:
        game_tick(capture, game_1)
        game_tick(capture, game_2)

        #time.sleep(0.05)

if __name__ == '__main__':
    main()
