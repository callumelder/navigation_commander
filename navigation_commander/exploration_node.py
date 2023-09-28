import rclpy
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

import numpy as np
import threading


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

        self.grid_data_1D = None
        self.grid_data_2D = None
        self.frontier_map = np.zeros((self.width, self.height))

        self.init_position = (0, 0)

        # # subscribe to map (unsure if needed)
        # self.map_subscription = self.create_subscription(
        #     OccupancyGrid,
        #     'map', 
        #     self.map_callback,
        #     10 
        # )
        # self.map_subscription # prevent unused variable warning

        # subscribe to costmap
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid,
            #'/map',
            '/global_costmap/costmap',
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
        Isaac and Callum
        """
        self.width = msg.info.width
        self.height = msg.info.height

        self.grid_data_1D = list(msg.data)

        x = np.linspace(0, 1, self.width)
        y = np.linspace(0, 1, self.height)
        self.x_2D, self.y_2D = np.meshgrid(x,y)

        # Convert this into a grid
        self.grid_data_2D = np.reshape(self.grid_data_1D, (self.height, self.width))

        self.frontier_map = self.get_frontiers()

        print('got to the callback function')
        
    
    # def map_callback(self, msg):
    #     """
    #     Processes the data received from the map
    #     """
    #     return msg
    
    def explore_map(self):
        """
        Primary function utilizing search algorithm (bfs or dfs)
        Callum
        """
        frontier_coords = [self.get_initial_position()]
        self.frontier_map = self.get_frontiers()

        while len(frontier_coords) > 0:
            self.frontier_map = self.get_frontiers()
            frontier_coord = frontier_coords.pop()
            frontier_coords.clear()
            print(self.frontier_map)

            # skips first waypoint
            if frontier_coord == self.get_initial_position():
                print('1st loop')
                self.frontier_map = self.get_frontiers()
                print('got here')
                frontier_coords.append(self.find_highest_frontier_density(self.frontier_map))
                print('got here 2')
                time.sleep(1.0)
                continue

            # get waypoint from frontier
            waypoint = self.convert_to_waypoint(frontier_coord)

            # move to frontier
            self.send_goal_waypoint(waypoint)
            
            frontier_coords.append(self.find_highest_frontier_density(self.frontier_map))

        print("Map complete!")
        return
    
    def get_initial_position(self):
        return self.init_position
    
    def publishInitialPose(self):
        """
        The get initial pose i had to write a bit more because
        otherwise it would just get the current pose of the bot
        every time you run the get_start_pos function. I think to
        access the intial pose you have to use self.init_pose
        """
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
    
    def get_frontiers(self):
        """
        Adds newly found frontiers to the queue/stack
        Returns a list of frontier points
        Isaac
        """
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
                            print("This value: {}".format(this_value))
                            print("Left corner neighbours: {}".format(neighbour_values))
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
                        #print("Found one")
                        frontier_map[x][y] = self.IS_FRONTIER_VALUE
                        mx = mx + self.x_2D[x][y]
                        my = my + self.y_2D[x][y]
                        number_of_frontier_points = number_of_frontier_points + 1
        return frontier_map
    
    def find_highest_frontier_density(self, frontier_map, kernel=3):
        """
        Groups frontier points to frontier area
        A grid point of frontier tuple
        Relative to map's origin
        e.g (x,y)
        Callum
        """
        if len(frontier_map) == 0:
            return (0, 0)
        
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

        print('got to end of highest frontier')
        print(f'Max Position: {max_position}')

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
