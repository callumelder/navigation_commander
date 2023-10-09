import rclpy
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import BehaviorTreeLog

import numpy as np
import threading

import matplotlib.pyplot as plt

# Create a global fig (for testing)
DEBUG_WITH_GRAPH=True
if (DEBUG_WITH_GRAPH):
    fig = plt.figure()

class ExplorationNode(Node):
    def __init__(self):
        super().__init__(node_name='explorer')

        # initialize variables here
        self.IS_FRONTIER_VALUE = 101
        self.IS_UNKNOWN_VALUE = -1
        self.THRESHHOLD_VALUE = 20

        self.width = 0
        self.height = 0
        self.x_2D = 0
        self.y_2D = 0

        self.origin = [0, 0]

        self.grid_data_1D = None
        self.grid_data_2D = None
        self.frontier_map = np.zeros((self.width, self.height))

        self.init_position = (0, 0)

        self.node_name = 'NavigateRecovery'
        self.current_status = 'IDLE'
        self.currentPose = None

        # subscribe to costmap
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.global_costmap_callback,
            10
        )
        self.costmap_subscription # prevent unused variable warning

        # subscribe to behaviour tree log
        self.btl_subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_log_callback,
            10)
        self.btl_subscription # prevent unused variable warning

        # publisher for goal waypoint
        self.waypoint_publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )

        pose_qos = QoSProfile(
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability = QoSReliabilityPolicy.RELIABLE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 1
        )

        self.model_pose_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.poseCallback,
            pose_qos
        )
        


    def global_costmap_callback(self, msg):
        """
        Processes the data received from the cost map
        Written by Isaac
        """
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution

        # Grab the origin from the costmap
        origin = msg.info.origin
        self.origin[0] = origin.position.x
        self.origin[1] = origin.position.y

        self.grid_data_1D = list(msg.data)

        x = np.linspace(0, 1, self.width)
        y = np.linspace(0, 1, self.height)

        self.ax = fig.add_subplot(111)

        # Update costmap coordinates using the origin information
        # WIDTH = abs(2.0*self.origin[0])
        # HEIGHT = abs(2.0*self.origin[1])
        WIDTH = self.resolution*self.width   # width in meters
        HEIGHT = self.resolution*self.height
        x = (x)*WIDTH + self.origin[0]
        y = (y)*HEIGHT + self.origin[1]

        self.x_2D, self.y_2D = np.meshgrid(x,y)

        # Convert this into a grid
        self.grid_data_2D = np.reshape(self.grid_data_1D, (self.height, self.width))


    def bt_log_callback(self, msg:BehaviorTreeLog):
        """
        Process behaviour tree log messag data
        Written by Callum
        """
        latest_event = msg.event_log.pop()
        self.node_name = latest_event.node_name
        self.current_status = latest_event.current_status

    def poseCallback(self, msg):
        """
        gets current pose of robot - u can compare this to the goal pose u want to send
        pose is in header_frame_id <- should be 'map' here
        this is from odometry
        wiki.ros.org/amcl
        """
        self.currentPose = msg.pose.pose

    
    def explore_map(self):
        """
        Primary function utilizing search algorithm (bfs or dfs)
        Written by Callum
        """

        # Make sure we have costmap information before proceeding
        while (self.grid_data_1D == None):
            self.get_logger().info("Polling for costmap information from subscribed lister..")
            time.sleep(1)

        # initialize frontier map and coordinates
        frontier_coords = []
        self.frontier_map = self.get_frontiers()
        max_coordinates = self.find_highest_frontier_density(self.frontier_map)
        frontier_coords.extend(max_coordinates)

        while len(frontier_coords) > 0:
            time.sleep(1)
            self.frontier_map = self.get_frontiers()
            frontier_coord = frontier_coords.pop(0)
            frontier_coords.clear()

            self.debug_plot_map(DEBUG_WITH_GRAPH, self.ax)

            print(f'Current Coordinate: {frontier_coord}')

            waypoint = self.convert_to_waypoint(frontier_coord)

            self.debug_plot_waypoint(DEBUG_WITH_GRAPH, self.ax, waypoint)

            # move to frontier
            self.send_goal_waypoint(waypoint)
            max_coordinates = self.find_highest_frontier_density(self.frontier_map)

            if (len(max_coordinates) == 0):
                self.get_logger().warning('No maximum density found; likely have completed mapping...')
                break

            frontier_coords.extend(max_coordinates)

        self.get_logger().info('Map complete!')
        
        return
    
    def debug_plot_map(self, debugger, ax):
        """
        Plots frontier map
        Written by Isaac
        """
        if (debugger):
                try:
                    # Draw the frontier map and grid data; after rotating to have the view match
                    ax.contour(-self.y_2D, self.x_2D, self.grid_data_2D-200, 10)
                    ax.contour(-self.y_2D, self.x_2D, self.frontier_map, 10, colors=['red'])
                except:
                    self.get_logger().warning('Aborting graphing effort...')

    
    def debug_plot_waypoint(self, debugger, ax, waypoint):
        """
        Plots current waypoint to move to on map
        Written by Isaac
        """
        if (debugger):
                try:
                    # Draw the waypoint on the map; after rotating to have the view match
                    ax.plot(-waypoint.pose.position.y, waypoint.pose.position.x, 0, marker = '^', color='black')
                    plt.show(block=False)
                    plt.pause(1)
                    fig.clear()
                except:
                    self.get_logger().warning('Aborting graphing effort...')
    

    def get_frontiers(self):
        """
        Adds newly found frontiers to the queue/stack
        Returns a list of frontier points
        Written by Isaac
        """
        self.get_logger().info('Getting frontiers...')
        # Now let's generate a frontier map
        frontier_map = np.zeros((self.height, self.width))
        mx = 0.0
        my = 0.0
        number_of_frontier_points = 0
        
        for x in range(self.height):
            for y in range(self.width):
                this_value = self.grid_data_2D[x][y]
                # Check if this value is beneath our threshhold
                if ((this_value >= 0) & (this_value < self.THRESHHOLD_VALUE)): 
                    # Assume this cell is not a frontier
                    is_frontier = False
                    neighbour_values = []
                    if (x == 0):
                        # Do corners, then center
                        if (y == 0):
                            # bottom left corner
                            neighbour_values.append(self.grid_data_2D[x+1][y])
                            neighbour_values.append(self.grid_data_2D[x][y+1])
                            neighbour_values.append(self.grid_data_2D[x+1][y+1])
                        elif (y == self.width-1):
                            # bottom right corner:
                            neighbour_values.append(self.grid_data_2D[x+1][y])
                            neighbour_values.append(self.grid_data_2D[x][y-1])
                            neighbour_values.append(self.grid_data_2D[x+1][y-1])
                        else:
                            # bottom strip
                            neighbour_values.append(self.grid_data_2D[x][y-1])
                            neighbour_values.append(self.grid_data_2D[x+1][y-1])
                            neighbour_values.append(self.grid_data_2D[x+1][y])
                            neighbour_values.append(self.grid_data_2D[x-1][y+1])
                            neighbour_values.append(self.grid_data_2D[x][y+1])
                    elif (x == self.height-1):
                        if (y == 0):
                            # top left corner
                            neighbour_values.append(self.grid_data_2D[x-1][y])
                            neighbour_values.append(self.grid_data_2D[x-1][y+1])
                            neighbour_values.append(self.grid_data_2D[x][y+1])
                        elif (y == self.width-1):
                            # top right corner:
                            neighbour_values.append(self.grid_data_2D[x-1][y])
                            neighbour_values.append(self.grid_data_2D[x-1][y-1])
                            neighbour_values.append(self.grid_data_2D[x][y-1])
                        else:
                            # top strip
                            neighbour_values.append(self.grid_data_2D[x][y+1])
                            neighbour_values.append(self.grid_data_2D[x-1][y+1])
                            neighbour_values.append(self.grid_data_2D[x-1][y])
                            neighbour_values.append(self.grid_data_2D[x-1][y-1])
                            neighbour_values.append(self.grid_data_2D[x][y-1])
                    else:
                        if (y == 0):
                            # left strip
                            neighbour_values.append(self.grid_data_2D[x+1][y])
                            neighbour_values.append(self.grid_data_2D[x+1][y+1])
                            neighbour_values.append(self.grid_data_2D[x][y+1])
                            neighbour_values.append(self.grid_data_2D[x-1][y+1])
                            neighbour_values.append(self.grid_data_2D[x-1][y])
                        elif (y == self.width-1):
                            # right strip:
                            neighbour_values.append(self.grid_data_2D[x-1][y])
                            neighbour_values.append(self.grid_data_2D[x-1][y-1])
                            neighbour_values.append(self.grid_data_2D[x][y-1])
                            neighbour_values.append(self.grid_data_2D[x+1][y-1])
                            neighbour_values.append(self.grid_data_2D[x+1][y])
                        else:
                            # central cell
                            neighbour_values.append(self.grid_data_2D[x-1][y])
                            neighbour_values.append(self.grid_data_2D[x-1][y-1])
                            neighbour_values.append(self.grid_data_2D[x][y-1])
                            neighbour_values.append(self.grid_data_2D[x+1][y-1])
                            neighbour_values.append(self.grid_data_2D[x+1][y])
                            neighbour_values.append(self.grid_data_2D[x][y+1])
                            neighbour_values.append(self.grid_data_2D[x+1][y+1])
                            neighbour_values.append(self.grid_data_2D[x-1][y+1])

                    # Check neighbours
                    is_frontier =  self.IS_UNKNOWN_VALUE in neighbour_values
                    if is_frontier:
                        frontier_map[x][y] = self.IS_FRONTIER_VALUE
                        mx = mx + self.x_2D[x][y]
                        my = my + self.y_2D[x][y]
                        number_of_frontier_points = number_of_frontier_points + 1
        return frontier_map
    

    def find_highest_frontier_density(self, frontier_map, kernel=3):
        """
        Finds coordinates of an area of size kernel that contains the most frontiers
        Callum
        """
        self.get_logger().info('Finding highest frontier density coordinate...')
        if len(frontier_map) == 0:
            return (0, 0)
        
        coordinate_density_pairs = {}
        rows, cols = len(frontier_map), len(frontier_map[0])
        top_coordinate_num = 3
        threshold = 0

        for row in range(rows):
            for col in range(cols):
                sub_map = frontier_map[row:row+kernel, col:col+kernel]
                density = np.sum(sub_map)
                if density > threshold:
                    coordinate = (self.x_2D[row][col], self.y_2D[row][col])
                    coordinate_density_pairs[coordinate] = density

        # sort coordinates based on highest density, adding top 3 to top_coordinates
        sorted_coordinates = sorted(coordinate_density_pairs.items(), key=lambda x: x[1], reverse=True)
        top_coordinates = sorted_coordinates[:top_coordinate_num]
        top_coordinates = [coordinate for coordinate, _ in top_coordinates]


        print(f'Top Coordinates: {top_coordinates}')

        return top_coordinates
    
    def calc_dist(self,point1,point2):
        """
        calculates euclidean distance between two points represented by tuples in form (x,y)
        """
        dx = point1[0] - point2[0]
        dy = point1[1] - point2[1]
        return (dx**2+dy**2)**0.5
    
    def convert_to_waypoint(self, inspection_point):
        """
        Converts frontier to waypoint
        Jasmine
        input: tuple of (x,y)
        output: gives a target goal for the bot to reach
        """
        self.get_logger().info('Converting waypoint...')
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.pose.position.x = float(inspection_point[0])
        inspection_pose.pose.position.y = float(inspection_point[1])
        return inspection_pose
    
    
    def send_goal_waypoint(self, waypoint):
        """
        Sends goal waypoint to Nav2
        Written by Chen and Callum
        """
        while self.node_name != 'NavigateRecovery' and self.current_status != 'IDLE':
            self.get_logger().info('Waiting for goal to finish...')
            time.sleep(3)
        self.get_logger().info('Sending goal waypoint...')
        self.waypoint_publisher.publish(waypoint)


def main(args=None):
    """
    Main function
    Written by Callum
    """
    rclpy.init(args=args)
    exploration_node = ExplorationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(exploration_node)

    try:
        executor_thread = threading.Thread(target=executor.spin)
        executor_thread.start()

        exploration_node.explore_map()

        executor_thread.join()
    finally:
        rclpy.spin(exploration_node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
