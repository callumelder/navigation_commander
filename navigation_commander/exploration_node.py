import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import PoseStamped

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
    
    def get_start_position(self):
        """
        gets initials coordinates of robot relative to the map
        Jasmine
        """
        return
    
    def get_global_frontiers(self):
        """
        Adds newly found frontiers to the queue/stack
        Returns a list of frontier points
        Isaac
        """
        return
    
    def find_centroid(self):
        """
        Groups frontier points to frontier area
        A grid point of frontier tuple
        Relative to map's origin
        e.g (x,y)
        Callum
        """
        return
    
    def convert_to_waypoint(self):
        """
        Converts frontier to waypoint
        Jasmine
        """
        return
    
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
