#!/usr/bin/env python3
"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
import transforms3d.euler
from numpy import linalg as LA
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# TODO: import as you need
from math import sin, cos, log
from scipy import interpolate
import time
import csv
from tf_transformations import quaternion_matrix, euler_from_quaternion
from scipy.interpolate import splprep, splev
import pathlib


# class def for tree nodes
# It's up to you if you want to use this
class RRTNode(object):
    def __init__(self, x=None, y=None, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = None  # only used in RRT*
        self.is_root = False


def body2world(odom_msg, x, y):
    """
    Transform from body to world frame.
    """
    quaternion = [odom_msg.pose.pose.orientation.x,
                  odom_msg.pose.pose.orientation.y,
                  odom_msg.pose.pose.orientation.z,
                  odom_msg.pose.pose.orientation.w]
    R = quaternion_matrix(quaternion)[:3, :3]
    T = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z])
    transform = np.zeros((4, 4))
    transform[:3, :3] = R
    transform[:3, 3] = T
    transform[-1, -1] = 1
    world_ = np.array([[x], [y], [0], [1]])
    world_ = transform.dot(world_).flatten()
    return world_[0], world_[1]


def world2body(odom_msg, x, y):
    """
    Transform from world to body frame.
    """
    quaternion = [odom_msg.pose.pose.orientation.x,
                  odom_msg.pose.pose.orientation.y,
                  odom_msg.pose.pose.orientation.z,
                  odom_msg.pose.pose.orientation.w]
    R = quaternion_matrix(quaternion)[:3, :3]
    T = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z])
    transform = np.zeros((4, 4))
    transform[:3, :3] = R
    transform[:3, 3] = T
    transform[-1, -1] = 1
    body_ = np.array([[x], [y], [0], [1]])
    body_ = (LA.inv(transform) @ body_).flatten()
    return body_[0], body_[1]

angle_min = -np.pi
angle_increment = 0.005823155
angles = np.array([angle_min + i * angle_increment for i in range(1080)])

# class def for RRT
class RRT(Node):
    def __init__(self):
        super().__init__('rrt_node')
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        pose_topic = "/ego_racecar/odom"
        scan_topic = "/scan"

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        self.pose_sub_ = self.create_subscription(
            Odometry,
            pose_topic,
            self.pose_callback,
            1)
        self.pose_sub_

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            1)
        self.scan_sub_

        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need
        self.drive_pub_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.pp_waypoints_pub_ = self.create_publisher(Marker, '/pp_waypoints', 10)
        self.pp_path_pub_ = self.create_publisher(MarkerArray, '/pp_paths', 10)
        self.rrt_waypoints_pub_ = self.create_publisher(Marker, '/rrt_waypoints', 10)
        self.rrt_path_pub_ = self.create_publisher(MarkerArray, '/rrt_paths', 10)

        # class attributes
        # TODO: maybe create your occupancy grid here
        self._map = OccGrid()
        self.max_steering_dist = 0.5 #TODO: max distance from point to steer
        self.rrt_lookahead_dist = 1.0 #TODO
        self.pure_pursuit_lookahead_dist = 0.3 #TODO
        self.goal_thres = 0.5  # check whether a point is close to goal
        self.rrt_path = []
        self.path = self.create_waypoints() # pure pursuit path
        self.timer = self.create_timer(1.0, self.pure_pursuit_path_callback) # for occ map generation
        self.iter_max = 200  # max iteration of rrt
        self.x_curr = 0  # current position
        self.y_curr = 0
        self.yaw = 0
        self.cell_width = 12
        self.x_off = 15.0 # make obstacle cells larger to improve safety
        self.y_off = 1.0

    def waypoints_callback(self, publisher, x, y, color):
        marker = Marker(type=Marker.SPHERE,
                        scale=Vector3(x=0.3, y=0.3, z=0.3))
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        publisher.publish(marker)

    def map_callback(self, publisher, path, color):
        marker_array = MarkerArray()
        for idx, node in enumerate(path):
            marker = Marker(type=Marker.SPHERE,
                            scale=Vector3(x=0.1, y=0.1, z=0.1))
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.id = idx
            marker.pose.position.x = node[0]
            marker.pose.position.y = node[1]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            pt = Point()
            pt.x = marker.pose.position.x
            pt.y = marker.pose.position.y
            marker.points.append(pt)
            marker_array.markers.append(marker)
        publisher.publish(marker_array)

    def pure_pursuit_path_callback(self):
        self.map_callback(self.pp_path_pub_, self.path, [0.0, 255.0, 0.0, 1.0])

    def create_waypoints(self):
        # Define skeleton waypoints to interpolate between
        # x_skel = [9.24423, 9.54901, 9.45486, 8.95488, 5.41726, -1.9899, -8.85176, -11.8562, -13.0861, -13.725, -13.7381,
        #           -13.3022, -12.3192, -7.07787, 0.785085, 2.83137, 8.51307, 9.71355]
        # y_skel = [0.139301, 5.30634, 8.1212, 8.74803, 8.63542, 8.74653, 8.84947, 8.74803, 8.3686, 7.39398, 4.29553,
        #           0.024683, -0.30985, -0.15942, -0.186643, -0.126611, -0.121112, 1.02859]
        x_skel = []
        y_skel = []
        with open('/home/chris/ese615_ws/lab6_pkg/waypoints/wp.csv', newline='') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
            i = 0
            for row in spamreader:
                if i % 50 == 0:
                    if 1.2 > np.float64(row[0][:-1]) > 1.0 and 19.0 > np.float64(row[1][:-1]) > 18.0:
                        continue
                    x_skel.append(row[0][:-1])
                    y_skel.append(row[1][:-1])

                i+=1
        x_skel = np.array(x_skel, dtype=np.float64)
        y_skel = np.array(y_skel, dtype=np.float64)
        waypoints = []

        tck, u = interpolate.splprep([x_skel, y_skel], s=0, per=True)  # Interpolate the spline
        x, y = interpolate.splev(np.linspace(0, 1, 400), tck)

        for i in range(len(x)):
            waypoints.append([x[i], y[i]])
        # tck, u = splprep(path.transpose(), s=0, per=True)
        # approx_length = np.sum(LA.norm(np.diff(splev(u, tck), axis=0), axis=1))
        #
        # num_waypoints = int(approx_length / 0.1)
        # dense_points = splev(np.linspace(0, 1, num_waypoints), tck)
        # waypoints = np.array([dense_points[0], dense_points[1], dense_points[2]]).transpose()
        waypoints = np.array(waypoints)
        return waypoints

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args:
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        grid = np.ones_like(self._map.grid)
        dist = scan_msg.ranges[179:899]
        lidar_x = self.x_curr + 0.29275*cos(self.yaw)
        lidar_y = self.y_curr + 0.29275*sin(self.yaw)
        curr_angle = self.yaw + angles[179:899]
        x_obs = lidar_x + dist * np.cos(curr_angle)
        y_obs = lidar_y + dist * np.sin(curr_angle)

        x_grid = (x_obs + self.x_off) / self._map.resolution
        y_grid = (y_obs + self.y_off) / self._map.resolution

        x_grid[(x_grid >= 499.5)] = 499
        y_grid[(y_grid >= 199.5)] = 199
        x_grid[(x_grid < 0)] = 0
        y_grid[(y_grid < 0)] = 0

        x_grid = x_grid.round(0).astype(int)
        y_grid = y_grid.round(0).astype(int)

        for i in range(len(dist)):
            x, y = x_grid[i], y_grid[i]
            grid[max(x - self.cell_width // 2, 0):min(x + self.cell_width // 2, self._map.height - 1) + 1,
            max(y - self.cell_width // 2, 0):min(y + self.cell_width // 2, self._map.width - 1) + 1] = 0
        self._map.grid = grid
        return 0

    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args:
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        x_curr = pose_msg.pose.pose.position.x
        y_curr = pose_msg.pose.pose.position.y
        self.x_curr = x_curr
        self.y_curr = y_curr
        quaternion = np.array([pose_msg.pose.pose.orientation.x,
                               pose_msg.pose.pose.orientation.y,
                               pose_msg.pose.pose.orientation.z,
                               pose_msg.pose.pose.orientation.w])
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.yaw = yaw
        start_node = RRTNode(x_curr, y_curr, -1)
        start_node.is_root = True
        rrt_tree = [start_node]
        for i in range(self.iter_max):
            node_rand = self.sample(pose_msg)
            node_near = self.nearest(rrt_tree, node_rand)
            node_new = self.steer(rrt_tree[node_near], node_rand)
            node_new.parent = node_near
            if node_new and not self.check_collision(rrt_tree[node_near], node_new):
                rrt_tree.append(node_new)
                rrt_goal_x, rrt_goal_y = self.find_rrt_goal(pose_msg, self.path, self.rrt_lookahead_dist)
                # visualize rrt goal
                self.waypoints_callback(self.rrt_waypoints_pub_, rrt_goal_x, rrt_goal_y, [255.0, 0.0, 0.0, 1.0])

                if self.is_goal(node_new, rrt_goal_x, rrt_goal_y):
                    print('rrt goal reached!!')
                    self.rrt_path = self.find_path(rrt_tree, node_new)
                    rrt_path_nodes = np.array([[node.x, node.y, 1] for node in self.rrt_path])

                    # visualize rrt paths
                    self.map_callback(self.rrt_path_pub_, rrt_path_nodes, [0.0, 0.0, 255.0, 1.0])

                    pp_goal_x, pp_goal_y = self.find_rrt_goal(pose_msg, rrt_path_nodes,
                                                              self.pure_pursuit_lookahead_dist)
                    # visualize pure pursuit waypoints
                    self.waypoints_callback(self.pp_waypoints_pub_, pp_goal_x, pp_goal_y, [0.0, 255.0, 0.0, 1.0])

                    pp_goal_x_body, pp_goal_y_body = world2body(pose_msg, pp_goal_x, pp_goal_y)
                    #TODO
                    steering_angle = 2 * pp_goal_y_body / self.pure_pursuit_lookahead_dist ** 2
                    # if pp_goal_y_body >= 0:  # TURN LEFT
                    #     steering_angle = 2.0 * np.abs(pp_goal_y_body) / self.pure_pursuit_lookahead_dist ** 2
                    # else:  # TURN RIGHT
                    #     steering_angle = -2.0 * np.abs(pp_goal_y_body) / self.pure_pursuit_lookahead_dist ** 2
                    angle = np.clip(steering_angle, -0.4, 0.4)  # steering angle bound
                    #TODO
                    speed = np.interp(abs(angle), np.array([0.0, 0.4, np.inf]), # possible angle range
                                      np.array([1.0, 0.3, 0.3]))  # desired speed 1.0 and min speed 0.3
                    # speed = 1.0
                    msg = AckermannDriveStamped()
                    msg.drive.steering_angle = angle
                    msg.drive.speed = speed
                    msg.header.stamp = self.get_clock().now().to_msg()
                    self.drive_pub_.publish(msg)
                    print('Driving command published')
                    return 0
        return None

    def sample(self, odom_msg):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        x = np.random.uniform(0, self.rrt_lookahead_dist)
        y = np.random.uniform(-self.rrt_lookahead_dist, self.rrt_lookahead_dist)
        x, y = body2world(odom_msg, x, y)
        return x, y

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        return int(np.argmin(
            [math.hypot(sampled_point[0] - node.x, sampled_point[1] - node.y) for node in tree]))

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        x_dist = sampled_point[0] - nearest_node.x
        y_dist = sampled_point[1] - nearest_node.y
        dist = LA.norm([x_dist, y_dist])
        new_node = RRTNode()
        new_node.x = nearest_node.x + min(dist, self.max_steering_dist) * (x_dist / dist)
        new_node.y = nearest_node.y + min(dist, self.max_steering_dist) * (y_dist / dist)
        return new_node

    def find_rrt_goal(self, odom_msg, path, lookahead_dist):
        position = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y])
        point_dist = LA.norm(path[:, 0:2] - position, axis=1)

        search_index = np.argmin(point_dist)
        while True:
            search_index = (search_index + 1) % np.size(path, axis=0)
            dist_diff = point_dist[search_index] - lookahead_dist
            if dist_diff < 0:
                continue
            elif dist_diff > 0:
                x_goal = np.interp(self.rrt_lookahead_dist,
                                   np.array([point_dist[search_index - 1], point_dist[search_index]]),
                                   np.array([path[search_index - 1, 0], path[search_index, 0]]))
                y_goal = np.interp(self.rrt_lookahead_dist,
                                   np.array([point_dist[search_index - 1], point_dist[search_index]]),
                                   np.array([path[search_index - 1, 1], path[search_index, 1]]))
                break
            else:
                x_goal = path[search_index, 0]
                y_goal = path[search_index, 1]
                break

        return x_goal, y_goal

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.
        Args:
            nearest_node (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        start = np.array([nearest_node.x, nearest_node.y])
        goal = np.array([new_node.x, new_node.y])
        dist = LA.norm(goal - start)
        n_points = int(np.ceil(dist) / self._map.resolution)
        if n_points > 1:
            step = dist / (n_points - 1)
            v = goal - start
            u = v / LA.norm(v)
            for i in range(n_points):
                if self.is_obstacle(start[0], start[1]):
                    return True
                start += u * step
        return False

    def is_obstacle(self, x, y):
        """
        Check if a certain index of the occupancy grid is occupied.
        """
        r, c = self.global_to_grid(x, y)
        return self._map.grid[r, c] == 0

    def global_to_grid(self, x_global, y_global):
        x_grid = (x_global + self.x_off) / self._map.resolution
        y_grid = (y_global + self.y_off) / self._map.resolution
        # process out of bound indices
        if x_grid >= 499.5:
            x_grid = 499
        elif x_grid < 0:
            x_grid = 0
        if y_grid >= 199.5:
            y_grid = 199
        elif y_grid < 0:
            y_grid = 0
        return np.round(x_grid).astype(int), np.round(y_grid).astype(int)

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enough to the goal
        """
        dist = LA.norm([goal_x - latest_added_node.x, goal_y - latest_added_node.y])
        return dist < self.goal_thres

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = [latest_added_node]
        node_now = tree[latest_added_node.parent]
        while not node_now.is_root:
            path.append(node_now)
            node_now = tree[node_now.parent]
        path.append(tree[0])
        path.reverse()
        print('Path found!!!!')
        return path

    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        return neighborhood


class OccGrid(object):
    def __init__(self, resolution=.05, width=200, height=500):
        self.resolution = resolution
        self.width = width
        self.height = height
        # self.origin_x = self.width // 2
        # self.origin_y = 0
        self.grid = np.ones((height, width))


def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
