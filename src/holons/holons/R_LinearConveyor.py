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
import RPi.GPIO as GPIO
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String
from holon_msgs.msg import AdjacencyList 
from holon_msgs.action import SpinLinear


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('Linear_Conveyor')
        self.publisher_ = self.create_publisher(AdjacencyList, 'graph_node_network', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.graph = AdjacencyList()
        self.graph.node = 0
        self.graph.adjacent = [1]

        #--------GPIO pin setup start-------------
        GPIO.setmode(GPIO.BOARD)
        self.pin_MOVEUP = 11
        self.pin_MOVEDOWN = 13
        GPIO.setup(self.pin_MOVEUP, GPIO.OUT)
        GPIO.setup(self.pin_MOVEDOWN, GPIO.OUT)
        GPIO.output(self.pin_MOVEUP, GPIO.LOW)
        GPIO.output(self.pin_MOVEDOWN, GPIO.LOW)      
        #--------GPIO pin setup end-------------

        self.pos = 0
        self.length_of_movement = .5
        self._action_server = ActionServer(self,SpinLinear,'SpinLinear_Request',self.SpinLinear_callback)

        print("Ensure the conveyor is wired to the controller")
        time.sleep(1)
        print("init.....")
        for i in range(100):
            GPIO.output(self.pin_MOVEDOWN, GPIO.HIGH)
            time.sleep(self.length_of_movement)
        GPIO.output(self.pin_MOVEDOWN, GPIO.LOW)  
        print("ready")
        

    def timer_callback(self):
        self.publisher_.publish(self.graph)
        #self.get_logger().info('Publishing: "%x" and "%x"' % (self.graph.node self.graph.adjacent))

        #-------------Action Server Functions Start---------------------------
    def SpinLinear_callback(self, goal_handle):
            self.get_logger().info('Executing goal...')
            goal_handle.succeed()
            feedback_msg = SpinLinear.Feedback()

            if goal_handle.request.pos == 0 and self.pos != 0:
                for i in range(10):
                    GPIO.output(self.pin_MOVEDOWN, GPIO.HIGH)
                    feedback_msg.percent = i*10
                    goal_handle.publish_feedback(feedback_msg)
                    time.sleep(self.length_of_movement) # Change this variable depending on how long you need to move the conveyor
                GPIO.output(self.pin_MOVEDOWN, GPIO.LOW)
                self.pos = 0
            if (goal_handle.request.pos) == 1 and (self.pos != 1):
                for i in range(10):
                    GPIO.output(self.pin_MOVEUP, GPIO.HIGH)
                    feedback_msg.percent = i*10
                    goal_handle.publish_feedback(feedback_msg)
                    time.sleep(self.length_of_movement) # Change this variable depending on how long you need to move the conveyor
                GPIO.output(self.pin_MOVEUP, GPIO.LOW)
                self.pos = 1
            result = SpinLinear.Result()
            result.resultant = self.pos
            return result

        #-------------Action Server Function End-----------------------------


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
