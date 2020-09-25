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
import RPi.GPIO as GPIO

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
        self.trays = [0,0,0]
        self.spinning = 0 
        self.conveyor_GPIO_pin = 15
        self.conveyor_IR_pins = [38,40]
        self.pneumatic_stop_pins = [8,7]
        self.pneumatic_pin = 0
        
        self.new_ir_readings = [0,0]
        self.old_ir_readings = [0,0]
        self.kuka_tray_counter = [-1,-1]
        self.debounce_counter = [0,0]
        self.tray_requested = 0
        self.found = 0 
        self.correct_edge_found = 0

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.conveyor_GPIO_pin, GPIO.OUT)
        GPIO.setup(self.conveyor_IR_pins[0], GPIO.IN)
        GPIO.setup(self.conveyor_IR_pins[1], GPIO.IN)
        GPIO.setup(self.pneumatic_stop_pins[0], GPIO.OUT)
        GPIO.setup(self.pneumatic_stop_pins[1], GPIO.OUT)


    def timer_callback(self):
        self.publisher_.publish(self.graph)
        #self.get_logger().info('Publishing: "%x" and "%x"' % (self.graph.node self.graph.adjacent))

     
    #--------- Action Server Functions Start---------------------

    def goal_parse_msg(self,goal_request):

        self.get_logger().info('Received Spin request')
        return rclpy.action.GoalResponse(2)


    def spin_execute_callback(self, goal_handle):
        
        # import the received data
        product_id = goal_handle.request.product_id
        entry_spot = goal_handle.request.entry
        tray_id = goal_handle.request.tray_id
        # create a result 
        result = Spin.Result()
        # communicate that request is received
        print('Received request to spin')
        
        # determine which IR sensor is going to be used
        if entry_spot == 0:
            self.pneumatic_pin = self.pneumatic_stop_pins[0]
            print("working from KR16")
        else:
            self.pneumatic_pin = self.pneumatic_stop_pins[1]
            print("working from KR10")

        # wait for conveyor to finish its job if it is busy
        while self.spinning == 1:
            time.sleep(0.5)
            print("Waiting for conveyor to stop spinning")

        for i in range(0,len(self.trays)):
            if self.trays[i] == tray_id and self.found == 0:
                self.found = 1
                self.tray_requested = i
                print(self.tray_requested)
            #else:
            # return that there is no tray
        self.found = 0

        GPIO.output(self.conveyor_GPIO_pin, GPIO.HIGH)

        while self.correct_edge_found != 1:
            
            for i in range(0,len(self.kuka_tray_counter)):
                if GPIO.input(self.conveyor_IR_pins[i]):
                    self.new_ir_readings[i] = 1
                    self.debounce_counter[i] = 0
                    print("Im on the sensor at")
                    print(i)
                else: 
                    self.debounce_counter[i] = self.debounce_counter[i] + 1
                    if self.debounce_counter[i] > 5:
                        self.new_ir_readings[i] = 0 
                        print("Im off the sensor at") 
                        print(i)

            print(self.new_ir_readings)
            print(self.old_ir_readings)
            for i in range(0,len(self.kuka_tray_counter)):
                print("I am in the edge loop")
                print(i)
                print(self.new_ir_readings[i])
                print(self.old_ir_readings[i])
                
                if self.new_ir_readings[i]  and not self.old_ir_readings[i]:
                    print("I am seeing an edge at")
                    print(i)
                    if self.kuka_tray_counter[i] == 2:
                        self.kuka_tray_counter[i] = 0
                    else:
                        self.kuka_tray_counter[i] = self.kuka_tray_counter[i] + 1
                    
                    if self.kuka_tray_counter[i] == self.tray_requested and i == entry_spot:
                        print("I have found my tray")
                        self.correct_edge_found = 1
                        GPIO.output(self.conveyor_GPIO_pin, GPIO.LOW)
                        GPIO.output(self.pneumatic_pin, GPIO.HIGH)

            print(self.new_ir_readings)
            print(self.old_ir_readings)
            print(self.kuka_tray_counter)  
               

            self.old_ir_readings[0] = self.new_ir_readings[0]
            self.old_ir_readings[1] = self.new_ir_readings[1]

            time.sleep(0.1)
        
        self.correct_edge_found = 0 
        time.sleep(5)

        for i in range(0,len(self.kuka_tray_counter)):
            if GPIO.input(self.conveyor_IR_pins[i]):
                self.old_ir_readings[i] = 1

        GPIO.output(self.pneumatic_pin, GPIO.LOW)
        print(self.tray_requested)
        self.trays[self.tray_requested] = product_id
        print(self.trays)
        self.spinning = 0

        feedback_msg = Spin.Feedback()
        feedback_msg.progress = 99

        self.spinning == 0 
        self.get_logger().info('Feedback: Finished Spinning')
        goal_handle.publish_feedback(feedback_msg)
        goal_handle.succeed()
        result.completion = True
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
    print("cleaning pins")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
