import rclpy
import time
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

import numpy as np


class ExplorationNode(Node):
    def __init__(self):
        super().__init__(node_name='explorer')

        # initialize variables here

        # subscribe to map (unsure if needed)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map', 
            self.map_callback,
            10 
        )
        self.map_subscription # prevent unused variable warning

        # subscribe to costmap
        self.costmap_subscription = self.create_subscription(
            Costmap,
            'global_costmap',
            self.global_costmap_callback,
            10
        )
        self.costmap_subscription # prevent unused variable warning

        # publisher for goal waypoint
        self.waypoint_publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )
        self.initial_pose_pub = self.create_publisher(
           PoseWithCovarianceStamped,
           'initialpose',
           10 
        )

    def global_costmap_callback(self, msg):
        """
        Processes the data received from the cost map
        """
        return msg
    
    def map_callback(self, msg):
        """
        Processes the data received from the map
        """
        return msg
    
    def explore_map(self):
        """
        Primary function utilizing search algorithm (bfs or dfs)
        Callum
        """
        frontiers = [self.get_start_position()]

        while len(frontiers) > 0:
            frontier = frontiers.pop(-1) # dfs

            # get waypoint from frontier
            waypoint = self.convert_to_waypoint(frontier)

            # move to frontier
            self.send_goal_waypoint(waypoint)
            
            # add newly found frontiers to stack
            frontiers.extend(self.get_frontiers())

        print("Map complete!")
        return
    
    """
    The get initial pose i had to write a bit more because
    otherwise it would just get the current pose of the bot
    every time you run the get_start_pos function. I think to
    access the intial pose you have to use self.init_pose
    """
    def publishInitialPose(self):
        self.initial_pose_pub.publish(self.init_pose)

    def get_start_position(self,pose):
        """
        gets initials coordinates of robot relative to the map
        Jasmine
        """
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.pose.pose.position.x = pose[0]
        self.init_pose.pose.pose.position.y = pose[1]
        self.init_pose.header.frame_id = 'map'
        self.currentPose = self.init_pose.pose.pose
        self.publishInitialPose()
        time.sleep(5)   
    
    def get_global_frontiers(self):
        """
        Adds newly found frontiers to the queue/stack
        Returns a list of frontier points
        Isaac
        """
        return
    
    def find_highest_frontier_density(self, frontier_map, kernel=3):
        """
        Groups frontier points to frontier area
        A grid point of frontier tuple
        Relative to map's origin
        e.g (x,y)
        Callum
        """
        max_density = 0
        max_position = None
        rows = len(frontier_map[0])
        cols = len(frontier_map[1])

        for row in range(rows):
            for col in range(cols):
                sub_map = frontier_map[row:row+kernel, col:col+kernel]
                density = np.sum(sub_map)
                if density > max_density:
                    max_density = density
                    max_position = (row, col)

        return max_position

    
    def convert_to_waypoint(self, inspection_point):
        """
        Converts frontier to waypoint
        Jasmine
        input: tuple of (x,y)
        output: gives a target goal for the bot to reach
        """
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.pose.postion.x = inspection_point[0]
        inspection_pose.pose.position.y = inspection_point[1]
        return inspection_pose
    
    def send_goal_waypoint(self, waypoint):
        """
        Sends goal waypoint to Nav2
        Chen
        """


def main(args=None):
    rclpy.init(args=args)
    exploration_node = ExplorationNode()
    rclpy.spin(exploration_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
