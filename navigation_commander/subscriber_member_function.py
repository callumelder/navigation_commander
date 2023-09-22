# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg  import OccupancyGrid
from array import array
import numpy as np
import matplotlib.pyplot as plt
import time

# Create a global fig
fig = plt.figure()

# class CustomMessage():
#     def __init__(self):
#         self.header = None
#         self.info = None
#         self.data = None

# message_to_publish = CustomMessage()

# message_to_publish = OccupancyGrid # hoping this will be the struct of occupancygrid

# Generate a threshold
THRESHHOLD_TYPE = "costmap"  # Will either by costmap or probability
THRESHHOLD_VALUE = 20

IS_FRONTIER_VALUE = 101
IS_UNKNOWN_VALUE = -1



""" class MinimalCustomPublisher(Node):

    def __init__(self):
        print('starting publisher')
        super().__init__('minimal_custom_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid(), 'CustomTopic', 10)
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Starting Publishing custom data')

    def timer_callback(self):
        # Could check to make sure message_to_publish has something useful
        # but I'm not yet.
        msg = message_to_publish
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing custom data')
 """


class MinimalSubscriber(Node):

    def __init__(self):
        print('starting subscriber')
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            #'/map',
            '/global_costmap/costmap',
            self.listener_callback,
            10)
        #self.message_to_publish = 0
        self.message_to_publish = OccupancyGrid
        #print(self.subscription)
        #self.subscription  # prevent unused variable warning
        #time.sleep(10) # trying to test if geting a subscription data will help publisher to work


        print('starting publisher')
        #super().__init__('minimal_custom_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'CustomTopic', 10)
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Starting Publishing custom data')

    def listener_callback(self, msg):
        print("listner callback")
        #self.get_logger().info("Meow")
        #self.get_logger().info('I heard: "%s"' % msg.data)
        #self.get_logger().info('I heard: "%s"' % msg.header)
        #self.get_logger().info('I heard: "%s"' % msg.info)
        width = msg.info.width
        height = msg.info.height
        #print("Got width, height = {}, {}".format(width, height))
        # Let's try pulling data out first
        grid_data_1D = list(msg.data)
        # Compute range of data
        min_data = min(grid_data_1D)
        max_data = max(grid_data_1D)
        #print("Min, Max = {}, {}".format(min_data, max_data))
        #print(grid_data_1D)
        # Convert this into a grid
        grid_data_2D = np.reshape(grid_data_1D, (height, width))
        #print(grid_data_2D)
        # Assume spacing of 1; this is not correct.
        x = np.linspace(0,1,width)
        y = np.linspace(0,1,height)
        x_2D, y_2D = np.meshgrid(x,y)
        #print("Size x,y : {}".format(x_2D.shape))
        #print("Size data : {}".format(grid_data_2D.shape))


        # Now let's generate a frontier map
        frontier_map = np.zeros((height, width))
        mx = 0.0
        my = 0.0
        number_of_frontier_points = 0
        
        for x in range(height):
            for y in range(width):
                this_value = grid_data_2D[x][y]
                # Check if this value is beneath our threshhold
                if ((this_value >= 0) & (this_value < THRESHHOLD_VALUE)): 
                    # Assume this cell is not a frontier
                    is_frontier = False
                    neighbour_values = []
                    if (x == 0):
                        # Do corners, then center
                        if (y == 0):
                            # bottom left corner
                            neighbour_values.append(grid_data_2D[x+1][y])
                            neighbour_values.append(grid_data_2D[x][y+1])
                            neighbour_values.append(grid_data_2D[x+1][y+1])
                            print("This value: {}".format(this_value))
                            print("Left corner neighbours: {}".format(neighbour_values))
                        elif (y == width-1):
                            # bottom right corner:
                            neighbour_values.append(grid_data_2D[x+1][y])
                            neighbour_values.append(grid_data_2D[x][y-1])
                            neighbour_values.append(grid_data_2D[x+1][y-1])
                        else:
                            # bottom strip
                            neighbour_values.append(grid_data_2D[x][y-1])
                            neighbour_values.append(grid_data_2D[x+1][y-1])
                            neighbour_values.append(grid_data_2D[x+1][y])
                            neighbour_values.append(grid_data_2D[x-1][y+1])
                            neighbour_values.append(grid_data_2D[x][y+1])
                    elif (x == height-1):
                        if (y == 0):
                            # top left corner
                            neighbour_values.append(grid_data_2D[x-1][y])
                            neighbour_values.append(grid_data_2D[x-1][y+1])
                            neighbour_values.append(grid_data_2D[x][y+1])
                        elif (y == width-1):
                            # top right corner:
                            neighbour_values.append(grid_data_2D[x-1][y])
                            neighbour_values.append(grid_data_2D[x-1][y-1])
                            neighbour_values.append(grid_data_2D[x][y-1])
                        else:
                            # top strip
                            neighbour_values.append(grid_data_2D[x][y+1])
                            neighbour_values.append(grid_data_2D[x-1][y+1])
                            neighbour_values.append(grid_data_2D[x-1][y])
                            neighbour_values.append(grid_data_2D[x-1][y-1])
                            neighbour_values.append(grid_data_2D[x][y-1])
                    else:
                        if (y == 0):
                            # left strip
                            neighbour_values.append(grid_data_2D[x+1][y])
                            neighbour_values.append(grid_data_2D[x+1][y+1])
                            neighbour_values.append(grid_data_2D[x][y+1])
                            neighbour_values.append(grid_data_2D[x-1][y+1])
                            neighbour_values.append(grid_data_2D[x-1][y])
                        elif (y == width-1):
                            # right strip:
                            neighbour_values.append(grid_data_2D[x-1][y])
                            neighbour_values.append(grid_data_2D[x-1][y-1])
                            neighbour_values.append(grid_data_2D[x][y-1])
                            neighbour_values.append(grid_data_2D[x+1][y-1])
                            neighbour_values.append(grid_data_2D[x+1][y])
                        else:
                            # central cell
                            neighbour_values.append(grid_data_2D[x-1][y])
                            neighbour_values.append(grid_data_2D[x-1][y-1])
                            neighbour_values.append(grid_data_2D[x][y-1])
                            neighbour_values.append(grid_data_2D[x+1][y-1])
                            neighbour_values.append(grid_data_2D[x+1][y])
                            neighbour_values.append(grid_data_2D[x][y+1])
                            neighbour_values.append(grid_data_2D[x+1][y+1])
                            neighbour_values.append(grid_data_2D[x-1][y+1])

                    # Check neighbours
                    is_frontier =  IS_UNKNOWN_VALUE in neighbour_values
                    if is_frontier:
                        #print("Found one")
                        frontier_map[x][y] = IS_FRONTIER_VALUE
                        mx = mx + x_2D[x][y]
                        my = my + y_2D[x][y]
                        number_of_frontier_points = number_of_frontier_points + 1
                    # Compute the centroid of the frontier

        # Centroid
        if (number_of_frontier_points == 0):
            print("No more frontier points")
        else:
            cx = mx / number_of_frontier_points
            cy = my / number_of_frontier_points
            print("Centroid = {}, {}".format(cx, cy))



        #ax = fig.add_subplot(111, projection='3d')
        ax = fig.add_subplot(111)
        ax.contour(x_2D, y_2D, grid_data_2D-200, 10)
        ax.contour(x_2D, y_2D, frontier_map, 10, colors=['red'])
        # plot the centroid
        if (number_of_frontier_points > 0):
            ax.plot(cx, cy, 0, marker = 'o')
        # Contour the cost map with a single black line


        plt.show(block=False)
        plt.pause(1)
        fig.clear()

        # Now let's also publish a result
        #result_2D = grid_data_2D
        #result_1D = np.reshape
        self.message_to_publish = msg
        # message_to_publish.header = msg.header
        # message_to_publish.info = msg.info
        # message_to_publish.data = msg.data

    def timer_callback(self):
        print("publisher callback")
        # Could check to make sure message_to_publish has something useful
        # but I'm not yet.
        self.message_to_publish.data[0] = 101
        msg = self.message_to_publish
        #self.publisher_.publish(msg)
        self.get_logger().info('Publishing custom data')



def main(args=None):

    rclpy.init(args=args)

    # This publishes stuff
    # minimal_custom_publisher = MinimalCustomPublisher()
    # rclpy.spin(minimal_custom_publisher)

    # This listens to the map and gets events as it updates
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    # minimal_custom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
