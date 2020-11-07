from shapely.geometry.polygon import Polygon
from detect_energy_cores_from_image import get_core_positions
from detect_aruco_markers_from_image import get_robot_positions
from shapely.geometry import Point


class Game:
    def __init__(self, robots):
        self.neg_core_positions = []
        self.pos_core_positions = []

        self.team_robots = robots

        self.goal_left = Polygon([(1080, 0), (840, 0), (1080, 250)])
        self.goal_right = Polygon([(0, 1080), (0, 840), (250, 1080)])

        self.tick = 0

    def update(self, capture):
        robot_frame_data = get_robot_positions(capture)
        for robot in self.team_robots:
            robot.update(robot_frame_data)

        core_positions = get_core_positions(capture)
        self.neg_core_positions = array_coords_to_points(core_positions[0])
        #self.pos_core_positions = array_coords_to_points(core_positions[1])
        self.pos_core_positions = []
        self.tick += 1

    def get_cores_not_in_goal(self, ecore_positions):
        filtered = []
        for point in ecore_positions:
            point_in_goal = self.goal_left.contains(point) or self.goal_left.contains(point)
            if not point_in_goal:
                filtered.append(point)
            else:
                print("REMOVED", point)

        return filtered
    

def array_coords_to_points(array_coords):
    points = []
    for coord in array_coords:
        points.append(Point(coord[0], coord[1]))
    return points
