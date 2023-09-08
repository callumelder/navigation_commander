import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ExplorationPublisher(Node):
    def __init__(self):
        super().__init__('exploration_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

def main():
    return None

if __name__ == '__main__':
    main()
