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
import time
from rclpy.node import Node
from std_msgs.msg import String
from holon_msgs.msg import AdjacencyList 
from holon_msgs.action import Storage
from rclpy.action import ActionServer

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(AdjacencyList, 'graph_node_network', 10)
        self._action_server = ActionServer(self,Storage,'storage_request_6',self.storage_callback,goal_callback=self.goal_parse_msg)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.create_timer(10,self.contents_callback)

        self.graph = AdjacencyList()
        self.graph.node = 6
        self.graph.adjacent = [3]

        self.contents = []

        print("init.....")
        print("ready")

    def timer_callback(self):
        self.publisher_.publish(self.graph)
        #self.get_logger().info('Publishing: "%x" and "%x"' % (self.graph.node self.graph.adjacent))
    def contents_callback(self):
        print('I contain:', self.contents)

    def goal_parse_msg(self,goal_request):
        self.get_logger().info('Storage Request at position %d, Placing Product ID: %d, Retrieving Tray ID: %d' % (goal_request.entry,goal_request.product_id,goal_request.tray_id))
        self.get_logger().info('Accepting Goal')
        return rclpy.action.GoalResponse(2)
        

    def storage_callback(self, goal_handle):
            self.get_logger().info('Executing goal...')
            self.contents.append(goal_handle.request.product_id)
            goal_handle.succeed()
            result = Storage.Result()
            result.completion = True
            return result

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
