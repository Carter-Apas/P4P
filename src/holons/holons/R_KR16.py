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
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from std_msgs.msg import String
from holon_msgs.msg import AdjacencyList

from holon_msgs.action import Transport 
from holon_msgs.action import Spin

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('R_KR16')
        self.publisher_ = self.create_publisher(AdjacencyList, 'graph_node_network', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Action server declaration
        self._action_server = ActionServer(self,Transport,'transport_request',self.transport_callback)
        self._action_client = ActionClient(self, Spin, 'spin_request')

        self.graph = AdjacencyList()
        self.graph.node = 1
        self.graph.adjacent = [0,2]
        self.conveyor_spinning = 0

    def timer_callback(self):
        self.publisher_.publish(self.graph)
        #self.get_logger().info('Publishing: "%x" and "%x"' % (self.graph.node self.graph.adjacent))
    
     #--------- Action Server Functions Start---------------------

    def goal_parse_msg(self,goal_request):
        self.get_logger().info('Transport request from node %d to %d' % (goal_request.a,goal_request.b))
        if ((goal_request.a == 0 and goal_request.b == 1) or goal_request.a == 1 and goal_request.b == 2):
            self.get_logger().info('Accepting Goal')
            return rclpy.action.GoalResponse(2)
        else:
            self.get_logger().info('Declined Goal')
            return rclpy.action.GoalResponse(1)


    def transport_callback(self, goal_handle):

        result = Transport.Result()
        print(goal_handle.request.a)
        nodea = goal_handle.request.a
        nodeb = goal_handle.request.b
        product_id = goal_handle.request.product_id
        print(nodeb)
        print(product_id)

        if nodea == 0 and nodeb == 1: 
            self.get_logger().info('Transporting to Node 1...')
            feedback_msg = Transport.Feedback()
            feedback_msg.percent = 0
            for i in range(1, 100):
                feedback_msg.percent = i
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(.05)
            goal_handle.succeed()
            result.result = True
            return result
        if nodea == 1 and nodeb == 2: 
            self.get_logger().info('Transporting to Node 2...')
            feedback_msg = Transport.Feedback()
            feedback_msg.percent = 0
            self.conveyor_spinning = 1
            self.send_spin_request(0, product_id)
            while self.conveyor_spinning != 0:
                time.sleep(0.5)
                rclpy.spin_once(self)
            for i in range(1, 100):
                feedback_msg.percent = i
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(.05)
            goal_handle.succeed()
            result.result = True
            return result
        else:
            goal_handle.abort()
            result.completion = False
            return result 

    def send_spin_request(self, entry, product_id):

        goal_msg = Spin.Goal()
        goal_msg.entry = entry
        goal_msg.product_id = product_id

        self.get_logger().info('Sending Spin Request')
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback
        print(feedback)
        self.get_logger().info('Received feedback: {0}'.format(feedback.progress))
        if feedback.progress == 777:
            print("I got to here")
            self.conveyor_spinning = 0

    
    #----------Action Server Functions End------------------------

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
