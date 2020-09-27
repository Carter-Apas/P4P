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
import sys
import threading

from threading import Thread 

import RPi.GPIO as GPIO
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from std_msgs.msg import String
from holon_msgs.msg import AdjacencyList
from holon_msgs.action import Transport 
from holon_msgs.action import Storage



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('R_KR16')
        self.publisher_ = self.create_publisher(AdjacencyList, 'graph_node_network', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Action server declaration
        self._action_server = ActionServer(self,Transport,'transport_request',self.transport_callback,goal_callback=self.goal_parse_msg)
        self.action_storage_client_2 = ActionClient(self, Storage, 'storage_request_2')
        self.action_storage_client_0 = ActionClient(self, Storage, 'storage_request_0')

        self.graph = AdjacencyList()
        self.graph.node = 1
        self.graph.adjacent = [0,2]
        self.conveyor_ready = 0
        self.linear_conveyor_ready = 0
        
        print("init.....")

        #--------GPIO pin setup start-------------
        GPIO.setmode(GPIO.BOARD)
        self.pin_start_01 = 36
        self.pin_start_12 = 37
        self.pin_progress = 31
        self.pin_finished = 33
        GPIO.setup(self.pin_start_01, GPIO.OUT)
        GPIO.setup(self.pin_start_12, GPIO.OUT)
        GPIO.setup(self.pin_progress, GPIO.IN)
        GPIO.setup(self.pin_finished, GPIO.IN)
        GPIO.output(self.pin_start_01, GPIO.LOW)    
        GPIO.output(self.pin_start_12, GPIO.LOW) 
        #--------GPIO pin setup end-------------
        print("ready")

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
        nodea = goal_handle.request.a
        nodeb = goal_handle.request.b
        product_id = goal_handle.request.product_id
        GPIO.output(self.pin_start_01, GPIO.LOW)    
        GPIO.output(self.pin_start_12, GPIO.LOW)
        kuka_state = 0

        if nodea == 0 and nodeb == 1: 
            self.get_logger().info('Transporting to Node 1...')
            feedback_msg = Transport.Feedback()
            feedback_msg.percent = 0
            
            #need to add a timout counter returning a failure should there be no input.
            self.send_storage_request_0(1,0,0) #Only entry matters for this function
            self.linear_conveyor_ready = 0
            while kuka_state != 4:
                rclpy.spin_once(self,executor=None, timeout_sec=0) #either get rid of this spin or somehow fix return
                if kuka_state == 0: 
                    if self.linear_conveyor_ready == 1:
                        kuka_state = 1
                elif kuka_state == 1:
                    GPIO.output(self.pin_start_01, 1) # Start arm movement for 01
                    kuka_state = 2
                elif kuka_state == 2: # Check for progress before moving on
                    if GPIO.input(self.pin_progress):
                        kuka_state = 3
                        feedback_msg.percent = 50
                        goal_handle.publish_feedback(feedback_msg)
                elif kuka_state == 3: # Check for movement finishing
                    if GPIO.input(self.pin_finished):
                        GPIO.output(self.pin_start_01, GPIO.LOW)
                        kuka_state = 4

            feedback_msg.percent = 99
            goal_handle.publish_feedback(feedback_msg)
            goal_handle.succeed()

            result = Transport.Result()
            result.completion = True
            return result #this return results totally zucks it
        elif nodea == 1 and nodeb == 2: 

            self.get_logger().info('Transporting to Node 2...')
            feedback_msg = Transport.Feedback()
            feedback_msg.percent = 0
            self.conveyor_spinning = 1
            self.send_storage_request_2(0, product_id, 0)
            self.spin_thread = Thread(target=self.new_thread_check)
            self.spin_thread.start()
            while self.conveyor_spinning != 0:
                time.sleep(0.5)
                print("I am waiting for the conveyor to stop spinning")
            self.spin_thread.join()

            while kuka_state != 3:
                rclpy.spin_once(self,executor=None, timeout_sec=0) #either get rid of this spin or somehow fix return
                if kuka_state == 0:
                    print("sure")
                    GPIO.output(self.pin_start_12, 1) # Start arm movement for 01
                    kuka_state = 1
                elif kuka_state == 1:
                    if GPIO.input(self.pin_progress):
                        print("nani")
                        kuka_state = 2
                        feedback_msg.percent = 50
                        goal_handle.publish_feedback(feedback_msg)
                elif kuka_state == 2:
                    if GPIO.input(self.pin_finished):
                        print("questiion")
                        GPIO.output(self.pin_start_12, GPIO.LOW)
                        kuka_state = 3
            
            feedback_msg.percent = 99
            goal_handle.publish_feedback(feedback_msg)
            goal_handle.succeed()
            result.result = True
            return result
            print("I returned the result")
        else:
            goal_handle.abort()
            result.completion = False
            return result 
        

    
    #----------Action Server Functions End------------------------
    #----------Action Storage Client Node 2 Functions Start-----------------

    def send_storage_request_2(self, entry, product_id, tray_id):

        goal_msg = Storage.Goal()
        goal_msg.entry = entry
        goal_msg.product_id = product_id
        goal_msg.tray_id = tray_id

        self.get_logger().info('Sending Storage Request to Node 2')
        self.action_storage_client_2.wait_for_server()
        self.send_goal_future_storage_2 = self.action_storage_client_2.send_goal_async(goal_msg, feedback_callback=self.feedback_callback_storage_2)
        self.send_goal_future_storage_2.add_done_callback(self.goal_response_callback_storage_2)

    def feedback_callback_storage_2(self, feedback_msg):

        feedback = feedback_msg.feedback
        print(feedback)
        self.get_logger().info('Received feedback: {0}'.format(feedback.progress))

    def new_thread_check(self):
        while self.conveyor_spinning != 0:
            rclpy.spin_once(self)

    def goal_response_callback_storage_2(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.get_result_future_storage_2 = goal_handle.get_result_async()
        self.get_result_future_storage_2.add_done_callback(self.get_request_result_callback_storage_2)

    def get_request_result_callback_storage_2(self, future):
        result = future.result().result
        if result.completion == True:
            self.conveyor_spinning = 0
        self.get_logger().info('Result: {0}'.format(result.completion))

    
    #----------Action Storage Client Node 2 Functions End----------------

    #----------Action Storage Client Node 0 Client Functions Start-------
    def send_storage_request_0(self, entry, product_id, tray_id):
        goal_msg = Storage.Goal()
        goal_msg.entry = 1

        self.get_logger().info('Sending Linear Spin Request')
        self.action_storage_client_0.wait_for_server()
        self.send_goal_future_storage_0 = self.action_storage_client_0.send_goal_async(goal_msg, feedback_callback=self.feedback_callback_storage_0)
        self.send_goal_future_storage_0.add_done_callback(self.goal_response_callback_storage_0)

    def feedback_callback_storage_0(self, feedback_msg):
        print("nani")
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.percent))
    
    def goal_response_callback_storage_0(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.get_result_future_storage_0 = goal_handle.get_result_async()
        self.get_result_future_storage_0.add_done_callback(self.get_result_callback_storage_0)

    def get_result_callback_storage_0(self, future):
        result = future.result().result
        if result.completion == 1:
            self.linear_conveyor_ready = 1
        self.get_logger().info('Result: {0}'.format(result.completion))
    #----------Action Storage Client Node 0 Functions End-------

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    while(1):
        rclpy.spin_once(minimal_publisher)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
