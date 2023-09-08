import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_publisher')

        # publish to cmd_vel to move robot
        self.publisher_ = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10
        )
        
        # subscribe to scan for lidar
        self.lidar_subscription = self.create_subscription(
            LaserScan, 
            'scan', 
            self.lidar_callback, 
            10
        )
        self.lidar_subscription # prevent unused variable warning

        # subscribe to map for waypoint config
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map', 
            self.map_callback,
            10 
        )
        self.map_subscription # prevent unused variable warning

    def lidar_callback():
        return None
    
    def map_callback():
        return None

def main(args=None):
    rclpy.init(args=args)
    exploration_node = ExplorationNode()
    rclpy.spin(exploration_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
