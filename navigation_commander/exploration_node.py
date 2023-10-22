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
    """The ROS2 node responsible for exploring the map, by running an exploration algorithm

    Args:
        Node (Node): The ros2 exploration node used to explore the map by finding frontiers
    """
    def __init__(self):
        super().__init__(node_name='explorer')

        # variables
        self.IS_FRONTIER_VALUE = 101    #value to represent frontier as it will display red in rvis
        self.IS_UNKNOWN_VALUE = -1      #value used by Nav2 to represent unknown cells
        self.THRESHHOLD_VALUE = 99      #values above -1 and below this are counted as known free space by get_frontiers
        self.RADIUS_THRESHHOLD = 1.5      #radius around robot for frontiers to be excluded, in meters

        self.dummy = None

        self.frontier_coord = None
        self.frontier_coords = []

        self.width = 0
        self.height = 0
        self.x_2D = 0
        self.y_2D = 0

        self.origin = [0, 0]    

        self.grid_data_1D = None    
        self.grid_data_2D = None   
        self.frontier_map = np.zeros((self.width, self.height)) 

        self.last_coordinate = None

        self.node_name = 'NavigateRecovery' 
        self.current_status = 'IDLE'        
        self.currentPose = None             

        # subscribe to costmap, used to find frontiers
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.global_costmap_callback,
            10
        )
        self.costmap_subscription # prevent unused variable warning

        # subscribe to behaviour tree log, used to find out if waypoint has been reached
        self.btl_subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_log_callback,
            10
        )
        self.btl_subscription # prevent unused variable warning

        # publisher for goal waypoint
        self.waypoint_publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )

        # settings for pose Quality of service
        pose_qos = QoSProfile(
            durability = QoSDurabilityPolicy.VOLATILE,
            reliability = QoSReliabilityPolicy.RELIABLE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 5
        )

        # subscribe to odom to get robots current position
        self.model_pose_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_callback,
            pose_qos
        )


    def global_costmap_callback(self, msg):
        """The callback function of the global costmap, used to populate the 2D grid data array

        Written by Isaac

        Args:
            msg (Global Costmap Message): The message data from the global costmap topic
        """
        self.width = msg.info.width             
        self.height = msg.info.height           
        self.resolution = msg.info.resolution   

        # Grab the origin from the costmap, in meters
        origin = msg.info.origin
        self.origin[0] = origin.position.x
        self.origin[1] = origin.position.y

        self.grid_data_1D = list(msg.data)  

        x = np.linspace(0, 1, self.width)   
        y = np.linspace(0, 1, self.height) 

        # Update costmap coordinates using the origin information
        WIDTH = self.resolution*self.width      # width in meters
        HEIGHT = self.resolution*self.height    # height in meters
        x = (x)*WIDTH + self.origin[0]          # makes an array in meters of the x coordinates of the costmap
        y = (y)*HEIGHT + self.origin[1]         # makes an array in meters of the y coordinates of the costmap

        self.x_2D, self.y_2D = np.meshgrid(x,y) # making a mesh for displaying in debug graph

        # Convert 1D data into a grid, note: this is in grid reference, not meters
        self.grid_data_2D = np.reshape(self.grid_data_1D, (self.height, self.width))

        self.dummy = 1

    def bt_log_callback(self, msg:BehaviorTreeLog):
        """callback function for retrieving the behaviour tree information

        Written by Callum

        Args:
            msg (BehaviorTreeLog Message): the message from the behaviour log tree topic
        """
        latest_event = msg.event_log.pop(-1)
        self.node_name = latest_event.node_name
        self.current_status = latest_event.current_status

    def pose_callback(self, msg):
        """gets current pose of robot - u can compare this to the goal pose u want to send
        pose is in header_frame_id <- should be 'map' here
        this is from odometry

        Written by Jasmine

        Args:
            msg (Pose Message): call back message of the current pose of the robot
        """
        self.currentPose = msg.pose.pose
        self.robot_position_meters = [self.currentPose.position.x, self.currentPose.position.y]
    
    
    def explore_map(self):
        """
        Primary loop for exploring the map. 
        Continuously loops while there are still frontiers being found.
        Ends when all frontiers are found (map complete).

        Written by Callum and Isaac
        """
        # Make sure we have costmap information before proceeding
        while (self.dummy == None):
            self.get_logger().info("Polling for costmap information from subscribed lister...")
            time.sleep(3)

        # initialize frontier map and coordinates
        self.frontier_coords = []                                                        
        self.frontier_map = self.get_frontiers()                                    
        max_coordinates = self.find_highest_frontier_density(self.frontier_map)     
        self.frontier_coords.extend(max_coordinates)                                    

        while len(self.frontier_coords) > 0:

            self.frontier_map = self.get_frontiers()
            self.frontier_coord = self.frontier_coords.pop(0)

            if self.last_coordinate == self.frontier_coord:
                try:
                    self.get_logger().info("Found same coordinate, skipping...")
                    self.frontier_coord = self.frontier_coords.pop(0)
                except:
                    continue

            waypoint = self.convert_to_waypoint(self.frontier_coord)

            if (DEBUG_WITH_GRAPH):
                try:
                    # Draw the frontier map and grid data; after rotating to have the view match
                    ax = fig.add_subplot(111)
                    ax.contour(-self.y_2D, self.x_2D, self.grid_data_2D-200, 10)
                    ax.contour(-self.y_2D, self.x_2D, self.frontier_map, 10, colors=['red'])
                    ax.plot(-waypoint.pose.position.y, waypoint.pose.position.x, 0, marker = '^', color='black')
                    plt.show(block=False)
                    plt.pause(1)
                    fig.clear()
                except:
                    self.get_logger().warning('Aborting graphing effort...')

            # move to frontier
            self.send_goal_waypoint(waypoint)
            self.wait_for_goal()

            self.frontier_coords.clear()

            max_coordinates = self.find_highest_frontier_density(self.frontier_map)

            if (len(max_coordinates) == 0): #check to see if their are any more frontiers
                self.get_logger().warning('No maximum density found; likely have completed mapping...')
                break

            self.frontier_coords.extend(max_coordinates)

            self.last_coordinate = self.frontier_coord

        self.get_logger().info('Map complete!')
    
    
    def wait_for_goal(self):
        """
        Waits for goal to finish, if stuck, moves to next frontier

        Written by Callum
        """
        start_time = time.time()
        break_time = 10
        while self.node_name != 'NavigateRecovery' and self.current_status != 'IDLE':
            elapsed_time = time.time() - start_time
            if elapsed_time >= break_time:
                try:
                    self.get_logger().warn('Took too long to get to goal, moving to next frontier...')
                    self.frontier_coord = self.frontier_coords.pop(0)
                    waypoint = self.convert_to_waypoint(self.frontier_coord)
                    self.send_goal_waypoint(waypoint)
                    break
                except:
                    self.get_logger().warn('Ran out of frontiers, calculating new ones...')
                    break
            self.get_logger().info('Waiting for goal to finish...')
            time.sleep(3)
    

    def find_highest_frontier_density(self, frontier_map, kernel=5):
        """Iterates through the frontier map using a kernel, 
        to sum up densities of frontiers over a threshold, storing the 
        coordinates of the high density frontiers in a list
        Then sorts the list by distance from the robot.

        Written by Callum and Isaac

        Args:
            frontier_map (list): 2D map of all frontiers
            kernel (int, optional): size of the kernel to sum frontiers. Defaults to 5.

        Returns:
            list: list of tuple coordinates of highest density frontiers ranked by distance from robot
        """
        self.get_logger().info('Finding highest frontier density coordinate...')
        if len(frontier_map) == 0:
            return (0, 0)
        
        half_kernel = int(kernel/2)
        coordinate_dictionary = {}
        rows, cols = len(frontier_map), len(frontier_map[0])    
        top_coordinate_num = 3                                  
        threshold = int(kernel*kernel*self.IS_FRONTIER_VALUE*0.1)         
        robot_position = self.robot_position_meters
        # iterate through frontier map, with stride of kernel size
        for row in range(0, rows, kernel):
            for col in range(0, cols, kernel):
                try:
                    # find highest density frontiers
                    sub_map = frontier_map[row-half_kernel:row+half_kernel, col-half_kernel:col+half_kernel]
                    density = np.sum(sub_map)

                    # calculate distance and store values
                    if density > threshold:
                        distance = self.calc_dist([self.x_2D[row][col], self.y_2D[row][col]], robot_position)   
                        if distance < self.RADIUS_THRESHHOLD:   
                            distance = 100 # set large distance to move to end of list
                        coordinate = (self.x_2D[row][col], self.y_2D[row][col])
                        coordinate_dictionary[coordinate] = (density, distance)
                except:
                    continue
    
        # sort coordinates so that the closet frontier (but further then 1 meter) of the heigest density is on the top of the list
        sorted_coordinate_dictionary = sorted(coordinate_dictionary.items(), key=lambda item: item[1][1])

        sorted_coordinates = [key for key, _ in sorted_coordinate_dictionary]

        top_coordinates = sorted_coordinates[:top_coordinate_num]

        return top_coordinates
    
    def calc_dist(self, point1, point2):
        """calculates euclidean distance between two points represented by tuples in form (x,y)

        Written by Jasmine

        Args:
            point1 (tuple): coordinate of first point
            point2 (tuple): coordinate of second point

        Returns:
            float: distance between two coordinates
        """
        dx = point1[0] - point2[0]
        dy = point1[1] - point2[1]
        return (dx**2+dy**2)**0.5
    
    def convert_to_waypoint(self, inspection_point):
        """Converts frontier coordinate to waypoint message structure

        Written by Jasmine

        Args:
            inspection_point (tuple): coordinate of frontier

        Returns:
            <class 'tuple'>: gives a target goal for the bot to reach
        """
        self.get_logger().info('Converting waypoint...')
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.pose.position.x = float(inspection_point[0])
        inspection_pose.pose.position.y = float(inspection_point[1])
        return inspection_pose
    
    
    def send_goal_waypoint(self, waypoint):
        """Sends goal waypoint to goal pose topic, 
        where Nav2 will handle the robot's routing and movement to the goal pose

        Written by Chen and Callum

        Args:
            waypoint (Waypoint Message): The current waypoint to be sent to the goal_pose topic
        """
        self.get_logger().info('Sending goal waypoint...')
        self.waypoint_publisher.publish(waypoint)
        time.sleep(3)


    def get_frontiers(self):
        """Adds newly found frontiers to the queue/stack
        
        Written by Isaac

        Returns:
            list: 2D array frontier map
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


def main(args=None):
    """Creates threads to spin the node and to explore the map in parallel

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
