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
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient 
from std_msgs.msg import String
from holon_msgs.msg import AdjacencyList 

from holon_msgs.action import Transport 
from holon_msgs.action import Spin 



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(AdjacencyList, 'graph_node_network', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self._action_server = ActionServer(self, Spin ,'spin_request', self.spin_execute_callback)

        self.graph = AdjacencyList()
        self.graph.node = 2
        self.graph.adjacent = [1,3]
        self.trays = [1,0,0,0,0,1]
        self.spinning = 0 

    def timer_callback(self):
        self.publisher_.publish(self.graph)
        #self.get_logger().info('Publishing: "%x" and "%x"' % (self.graph.node self.graph.adjacent))

     
    #--------- Action Server Functions Start---------------------

    def goal_parse_msg(self,goal_request):

        self.get_logger().info('Received Spin request')
        return rclpy.action.GoalResponse(2)


    def spin_execute_callback(self, goal_handle):

        product_id = goal_handle.request.product_id
        result = Spin.Result()
        print('Received request to spin')
        entry_spot = goal_handle.request.entry
        print(entry_spot)
        while self.spinning == 1:
            time.sleep(0.5)
            print("Im in the loop")
        new_tray = entry_spot
        num_of_spins = 0
        while self.trays[new_tray] != 0:
            if new_tray == 0:
                new_tray = (len(self.trays)-1)
            else:
                new_tray = new_tray - 1
            num_of_spins = num_of_spins + 1
        feedback_msg = Spin.Feedback()
        feedback_msg.progress = 0
        self.spinning == 1
        print(num_of_spins)
        print("I am here now")
        if num_of_spins >= 1:
            print("Im doing a spin")
            for i in range(1,(num_of_spins)+1):
                self.get_logger().info('completing one spin')
                old_right = self.trays[len(self.trays)-1]
                for j in range(len(self.trays)-1, 0, -1):
                    print(j)
                    print("Im in the for loop")
                    self.trays[j] = self.trays[j-1]
                    print(self.trays)
                self.trays[0]= old_right
                for k in range(1,100):
                    feedback_msg.progress = k
                    self.get_logger().info('Feedback: Spinning')
                    goal_handle.publish_feedback(feedback_msg)
                print(self.trays)
        self.trays[entry_spot] = product_id
        print(self.trays)
        #feedback_msg.progress = 'Completing Spin %d of %d' % (i, num_of_spins)  
        feedback_msg.progress = 777
        print(feedback_msg.progress)
        self.get_logger().info('Feedback: Finished Spinning')
        goal_handle.publish_feedback(feedback_msg)
        goal_handle.succeed()
        result.resultant = True
        print(result)
        return result

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
