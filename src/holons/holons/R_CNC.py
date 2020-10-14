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
from std_msgs.msg import String
from holon_msgs.msg import AdjacencyList 
import RPi.GPIO as GPIO

from holon_msgs.srv import MachineRequest5


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(AdjacencyList, 'graph_node_network', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.service_request_5 = self.create_service(MachineRequest5, 'machine_request_5', self.machine_5_callback)

        self.graph = AdjacencyList()
        self.graph.node = 4
        self.graph.adjacent = [3]
        self.Door_opening_pin = 35
        self.Door_closing pin = 36
        self.Interlock_shorting_pin = 37
        self.Chuck_opening_pin = 38

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.Door_opening_pin, GPIO.OUT)
        GPIO.setup(self.Door_closing_pin, GPIO.OUT)
        GPIO.setup(self.Chuck_opening_pin, GPIO.OUT)

    def timer_callback(self):
        self.publisher_.publish(self.graph)
        #self.get_logger().info('Publishing: "%x" and "%x"' % (self.graph.node self.graph.adjacent))

    def cnc_door_open_callback(self, request, response):
        if request.command == 1:
            response.reply = "Door is opening"
            self.get_logger().info('Opening Door')
            GPIO.output(self.Door_opening_pin, GPIO.HIGH)
            time.sleep(5)
            GPIO.output(self.Door_opening_pin, GPIO.LOW)
            return response

        elif request.command == 2:
            response.reply = "Door is closing"
            self.get_logger().info('Closing Door')
            GPIO.output(self.Door_closing_pin, GPIO.HIGH)
            time.sleep(5)
            GPIO.output(self.Door_closing_pin, GPIO.LOW)
            return response

        elif request.command == 3:
            response.reply = "Chuck is opening"
            self.get_logger().info('Opening Chuck')
            GPIO.output(self.Chuck_opening_pin, GPIO.HIGH)
            return response

        else request.command == 4:
            response.reply = "Chuck is closing"
            self.get_logger().info('Closing Chuck')
            GPIO.output(self.Chuck_opening_pin, GPIO.LOW)
            return response



        



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
