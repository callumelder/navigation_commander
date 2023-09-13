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
        return
    
    def map_callback(self, msg):
        """
        Processes the data received from the map
        """
        return
    
    def explore_map(self):
        """
        Primary function utilizing search algorithm (bfs or dfs)
        """
        return
    
    def get_frontiers(self):
        """
        Adds newly found frontiers to the queue/stack
        """
        return
    
    def convert_to_waypoint(self):
        """
        Converts frontier to waypoint
        """
    
    def send_goal_waypoint(self):
        """
        Sends goal waypoint to Nav2
        """


def main(args=None):
    rclpy.init(args=args)
    exploration_node = ExplorationNode()
    rclpy.spin(exploration_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
