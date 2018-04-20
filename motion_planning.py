import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, create_edges_from_grid, a_star_graph, prune_path, closest_point, find_start_goal
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global
from skimage.morphology import medial_axis
from skimage.util import invert
import networkx as nx
import numpy.linalg as LA

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            # deadband == 1.5
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 2:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # DONE: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            home_pos_data = f.readline().split(',')
        lat0 = float(home_pos_data[0].strip().split(' ')[1])
        lon0 = float(home_pos_data[1].strip().split(' ')[1])
        
        # DONE: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # DONE: retrieve current global position
        current_global_position = [self._longitude, self._latitude, self._altitude]
        # DONE: convert to current local position using global_to_local()
        current_local_position = global_to_local(current_global_position, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(
            self.global_home, self.global_position, self.local_position))
        print('global home {0}, position {1}, current local position {2}'.format(
            self.global_home, current_global_position, current_local_position))

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset, points = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        edges = create_edges_from_grid(grid, points)
        print("Grid = {}, north offset = {}, east offset = {}".format(grid.shape, north_offset, east_offset))

        # Define starting point on the grid
        # DONE: convert start position to current position rather than map center
        grid_start = (int(current_local_position[0] -north_offset)
                    , int(current_local_position[1] -east_offset))

        # Set goal as some arbitrary position on the grid
        # DONE: adapt to set goal as latitude / longitude position and convert
        # global_goal = local_to_global((750 + north_offset, 370 + east_offset, 0), self.global_home)
        # print('global_goal', global_goal)
        global_goal = (-122.39827335, 37.79639627, 0)
        local_goal = global_to_local(global_goal, self.global_home)
        grid_goal = (int(local_goal[0] - north_offset), int(local_goal[1] - east_offset)) #(750., 370.) #(920, 920)

        # Run A* to find a path from start to goal
        print('Local Start and Goal: ', grid_start, grid_goal)

        # DONE: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        """
        edges = create_edges_from_grid(grid, points)
        G = nx.Graph()
        for e in edges:
            p1 = e[0]
            p2 = e[1]
            dist = LA.norm(np.array(p2) - np.array(p1))
            G.add_edge(p1, p2, weight=dist)

        closest_grid_start = closest_point(G, grid_start)
        closest_grid_goal = closest_point(G, grid_goal)
        """
        skeleton = medial_axis(invert(grid))
        closest_grid_start, closest_grid_goal = find_start_goal(skeleton, grid_start, grid_goal)

        print('Closest Local Start and Goal: ', closest_grid_start, closest_grid_goal)

        # A*
        #path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # A* for Voronoi Graph
        #path, _ = a_star_graph(G, heuristic, closest_grid_start, closest_grid_goal)
        # A* for Medial Axis 
        path, _ = a_star(invert(skeleton).astype(np.int), heuristic, tuple(closest_grid_start), tuple(closest_grid_goal))

        # DONE: prune path to minimize number of waypoints
        # DONE (if you're feeling ambitious): Try a different approach altogether!
        pruned_path = prune_path(path)
        # Convert path to waypoints
        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in pruned_path]
        # Set self.waypoints
        self.waypoints = waypoints
        print(self.waypoints)
        # DONE: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
