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
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from std_msgs.msg import String
from holon_msgs.msg import AdjacencyList
from holon_msgs.action import Transport 
from holon_msgs.action import Storage


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('R_KR10')
        self.publisher_ = self.create_publisher(AdjacencyList, 'graph_node_network', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Action server declaration
        self._action_server = ActionServer(self,Transport,'transport_request',self.transport_callback,goal_callback=self.goal_parse_msg)
        self.action_storage_client_2 = ActionClient(self, Storage, 'storage_request_2')
        self.action_storage_client_5 = ActionClient(self, Storage, 'storage_request_5')
        self.action_storage_client_6 = ActionClient(self, Storage, 'storage_request_6')


        self.graph = AdjacencyList()
        self.graph.node = 3
        self.graph.adjacent = [2,4,5,6]

        self.storage_2_ready = 0

        print("init.....")
        #--------GPIO pin setup start-------------
        GPIO.setmode(GPIO.BOARD)
        self.pin_start_23 = 33
        self.pin_start_32 = 35
        self.pin_start_34 = 36  
        self.pin_start_43 = 37
        self.pin_start_35 = 38
        self.pin_start_36 = 40
        self.pin_progress = 31
        self.pin_finished = 33
        GPIO.setup(self.pin_start_23, GPIO.OUT)
        GPIO.setup(self.pin_start_32, GPIO.OUT)
        GPIO.setup(self.pin_start_34, GPIO.OUT)
        GPIO.setup(self.pin_start_43, GPIO.OUT)
        GPIO.setup(self.pin_start_35, GPIO.OUT)
        GPIO.setup(self.pin_start_36, GPIO.OUT)
        GPIO.setup(self.pin_progress, GPIO.IN)
        GPIO.setup(self.pin_finished, GPIO.IN)
        GPIO.output(self.pin_start_23, GPIO.LOW)    
        GPIO.output(self.pin_start_32, GPIO.LOW)
        GPIO.output(self.pin_start_34, GPIO.LOW) 
        GPIO.output(self.pin_start_43, GPIO.LOW) 
        GPIO.output(self.pin_start_35, GPIO.LOW) 
        GPIO.output(self.pin_start_36, GPIO.LOW)  
        #--------GPIO pin setup end-------------
        print("ready")

    def timer_callback(self):
        self.publisher_.publish(self.graph)
        #self.get_logger().info('Publishing: "%x" and "%x"' % (self.graph.node self.graph.adjacent))
    
    #--------- Action Server Functions Start---------------------
    def goal_parse_msg(self,goal_request):
        self.get_logger().info('Transport request from node %d to %d' % (goal_request.a,goal_request.b))
        if ((goal_request.a == 2 and goal_request.b == 3) or (goal_request.a == 3 and goal_request.b == 2) or (goal_request.a == 3 and goal_request.b == 4)or (goal_request.a == 4 and goal_request.b == 3) or (goal_request.a == 3 and goal_request.b == 5) or (goal_request.a == 3 and goal_request.b == 6)):
            print("Accepting Goal")
            return rclpy.action.GoalResponse(2)
        else:
            print("Declined Goal")
            return rclpy.action.GoalResponse(1)

    def transport_callback(self, goal_handle):

        result = Transport.Result()
        nodea = goal_handle.request.a
        nodeb = goal_handle.request.b
        product_id = goal_handle.request.product_id

        kuka_state = 0

        if nodea == 2 and nodeb == 3: 
            self.get_logger().info('Transporting to Node 3...')
            feedback_msg = Transport.Feedback()
            feedback_msg.percent = 0
            goal_handle.publish_feedback(feedback_msg)
            self.storage_2_ready = 0
            self.send_storage_request_2(1, 0, product_id)

            while kuka_state != 4:
                rclpy.spin_once(self,executor=None, timeout_sec=0)
                if kuka_state == 0: 
                    if self.storage_2_ready == 1:
                        kuka_state = 1
                elif kuka_state == 1:
                    GPIO.output(self.pin_start_23, 1) # Start arm movement for 01
                    kuka_state = 2
                elif kuka_state == 2: # Check for progress before moving on
                    if GPIO.input(self.pin_progress):
                        kuka_state = 3
                        feedback_msg.percent = 50
                        goal_handle.publish_feedback(feedback_msg)
                elif kuka_state == 3: # Check for movement finishing
                    if GPIO.input(self.pin_finished):
                        GPIO.output(self.pin_start_23, GPIO.LOW)
                        kuka_state = 4
            
            feedback_msg.percent = 99
            goal_handle.publish_feedback(feedback_msg)
            goal_handle.succeed()
            result.completion = True
            return result

        if nodea == 3 and nodeb == 2: 
            self.get_logger().info('Transporting to Node 2...')
            feedback_msg = Transport.Feedback()
            feedback_msg.percent = 0
            goal_handle.publish_feedback(feedback_msg)
            self.storage_2_ready = 0
            self.send_storage_request_2(1, product_id, 0)

            while kuka_state != 4:
                rclpy.spin_once(self,executor=None, timeout_sec=0) #either get rid of this spin or somehow fix return
                if kuka_state == 0: 
                    if self.storage_2_ready == 1:
                        kuka_state = 1
                elif kuka_state == 1:
                    GPIO.output(self.pin_start_32, 1) # Start arm movement for 01
                    kuka_state = 2
                elif kuka_state == 2: # Check for progress before moving on
                    if GPIO.input(self.pin_progress):
                        kuka_state = 3
                        feedback_msg.percent = 50
                        goal_handle.publish_feedback(feedback_msg)
                elif kuka_state == 3: # Check for movement finishing
                    if GPIO.input(self.pin_finished):
                        GPIO.output(self.pin_start_32, GPIO.LOW)
                        kuka_state = 4
            
            feedback_msg.percent = 99
            goal_handle.publish_feedback(feedback_msg)
            goal_handle.succeed()
            result.completion = True
            return result

        if nodea == 3 and nodeb == 4: 
            self.get_logger().info('Transporting to Node 4...')
            feedback_msg = Transport.Feedback()
            feedback_msg.percent = 0
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(.05)
            goal_handle.succeed()
            result.completion = True
            return result
        if nodea == 4 and nodeb == 3: 
            self.get_logger().info('Transporting to Node 3...')
            feedback_msg = Transport.Feedback()
            feedback_msg.percent = 0
            for i in range(1, 100):
                feedback_msg.percent = i
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(.05)
            goal_handle.succeed()
            result.completion = True
            return result
        if nodea == 3 and nodeb == 5: 
            self.get_logger().info('Transporting to Node 5...')
            feedback_msg = Transport.Feedback()
            feedback_msg.percent = 0
            goal_handle.publish_feedback(feedback_msg)

            while kuka_state != 3:
                rclpy.spin_once(self,executor=None, timeout_sec=0) #either get rid of this spin or somehow fix return

                if kuka_state == 0:
                    GPIO.output(self.pin_start_35, 1) # Start arm movement for 01
                    kuka_state = 1
                elif kuka_state == 1: # Check for progress before moving on
                    if GPIO.input(self.pin_progress):
                        kuka_state = 2
                        feedback_msg.percent = 50
                        goal_handle.publish_feedback(feedback_msg)
                elif kuka_state == 2: # Check for movement finishing
                    if GPIO.input(self.pin_finished):
                        GPIO.output(self.pin_start_35, GPIO.LOW)
                        kuka_state = 3

            self.send_storage_request_5(0, product_id, 0)
            feedback_msg.percent = 99
            goal_handle.publish_feedback(feedback_msg)
            goal_handle.succeed()

            result.completion = True
            return result
        if nodea == 3 and nodeb == 6: 
            self.get_logger().info('Transporting to Node 6...')
            feedback_msg = Transport.Feedback()
            feedback_msg.percent = 0
            goal_handle.publish_feedback(feedback_msg)

            while kuka_state != 3:
                rclpy.spin_once(self,executor=None, timeout_sec=0) #either get rid of this spin or somehow fix return
                if kuka_state == 0:
                    GPIO.output(self.pin_start_36, 1) # Start arm movement for 01
                    kuka_state = 1
                elif kuka_state == 1: # Check for progress before moving on
                    if GPIO.input(self.pin_progress):
                        kuka_state = 2
                        feedback_msg.percent = 50
                        goal_handle.publish_feedback(feedback_msg)
                elif kuka_state == 2: # Check for movement finishing
                    if GPIO.input(self.pin_finished):
                        GPIO.output(self.pin_start_36, GPIO.LOW)
                        kuka_state = 3

            self.send_storage_request_6(0, product_id, 0)
            feedback_msg.percent = 99
            goal_handle.publish_feedback(feedback_msg)
            goal_handle.succeed()

            result.completion = True
            return result 
    
    #----------Action Server Functions End------------------------

    #----------Action Client Storage 2 Start----------------------
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
        while self.storage_2_ready != 1:
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
            self.storage_2_ready = 1
        self.get_logger().info('Result: {0}'.format(result.completion))

    #---------Action Client Storage 2 End-------------------------
    #----------Action Client Storage 5 Start----------------------
    def send_storage_request_5(self, entry, product_id, tray_id):

        goal_msg = Storage.Goal()
        goal_msg.entry = entry
        goal_msg.product_id = product_id
        goal_msg.tray_id = tray_id

        self.get_logger().info('Sending Storage Request to Node 5')
        self.action_storage_client_5.wait_for_server()
        self.send_goal_future_storage_5 = self.action_storage_client_5.send_goal_async(goal_msg, feedback_callback=self.feedback_callback_storage_5)
        self.send_goal_future_storage_5.add_done_callback(self.goal_response_callback_storage_5)

    def feedback_callback_storage_5(self, feedback_msg):

        feedback = feedback_msg.feedback
        print(feedback)
        self.get_logger().info('Received feedback: {0}'.format(feedback.progress))

    def goal_response_callback_storage_5(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.get_result_future_storage_5 = goal_handle.get_result_async()
        self.get_result_future_storage_5.add_done_callback(self.get_request_result_callback_storage_5)

    def get_request_result_callback_storage_5(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.completion))

    #---------Action Client Storage 5 End-------------------------
        #----------Action Client Storage 6 Start----------------------
    def send_storage_request_6(self, entry, product_id, tray_id):

        goal_msg = Storage.Goal()
        goal_msg.entry = entry
        goal_msg.product_id = product_id
        goal_msg.tray_id = tray_id

        self.get_logger().info('Sending Storage Request to Node 6')
        self.action_storage_client_6.wait_for_server()
        self.send_goal_future_storage_6 = self.action_storage_client_6.send_goal_async(goal_msg, feedback_callback=self.feedback_callback_storage_6)
        self.send_goal_future_storage_6.add_done_callback(self.goal_response_callback_storage_6)

    def feedback_callback_storage_6(self, feedback_msg):

        feedback = feedback_msg.feedback
        print(feedback)
        self.get_logger().info('Received feedback: {0}'.format(feedback.progress))

    def goal_response_callback_storage_6(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.get_result_future_storage_6 = goal_handle.get_result_async()
        self.get_result_future_storage_6.add_done_callback(self.get_request_result_callback_storage_6)

    def get_request_result_callback_storage_6(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.completion))

    #---------Action Client Storage 6 End-------------------------

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
