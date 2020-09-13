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
import sys #For services and accepting argv from command line
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from holon_msgs.msg import AdjacencyList 
from holon_msgs.srv import ResourceMove
from holon_msgs.action import Transport
from collections import deque

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('product_node')
        self.subscription = self.create_subscription(AdjacencyList,'graph_node_network',self.list_callback,10)
        self.subscription  # prevent unused variable warning

        # self.transport_srv = self.create_client(ResourceMove, 'transport_service')
        # self.req = ResourceMove.Request() #This sets up variable
        
        #Action client declaration
        self._action_client = ActionClient(self, Transport, 'transport_request')

        #Path Finding variables
        self.graph = {}
        self.findPath = False # This variable means its okay to find a path
        self.num_of_resource_holons = 0

        #Useful global variables
        self.currentNode = int(sys.argv[1])
        self.endingNode = int(sys.argv[2])
        self.product_id = int(sys.argv[3])
        self.goalNode = 99
        self.path = []
        self.flag = False
        self.accepted_by_something = 0
        self.currently_moving = 0

        while(1): 
            rclpy.spin_once(self)
            if self.findPath and not self.currently_moving: 
                print("You may find a path") 
                self.fix_graph()
                print(self.graph)
                self.path = self.findShortestPath(self.currentNode,self.endingNode)
                print(self.path)
                try:
                    self.goalNode = self.path[1]
                    self.send_transport_request(self.currentNode,self.goalNode,self.product_id)
                    self.currently_moving = True
                except:
                    self.currently_moving = 0
                    print("No Path to be found.")
            elif self.accepted_by_something:
                        self.currently_moving = 0
                        self.currentNode = self.goalNode
                        self.accepted_by_something = 0
            if self.currentNode == self.endingNode:
                print("I made it baby")
                break
               
                
    #--------------------------Action based functions------------------  
   
    # Initial Request         
    def send_transport_request(self, a, b, product_id):
        goal_msg = Transport.Goal()
        goal_msg.a = a
        goal_msg.b = b
        goal_msg.product_id = product_id

        self.get_logger().info('Sending Transport Request for %d from node %d to node %d' % (goal_msg.product_id, goal_msg.a, goal_msg.b))
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.transport_callback)
        #self._send_goal_future.add_done_callback(self.transport_response_callback)



    #Feedback Response
    def transport_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.percent))
        if feedback.percent == 99:
            self.accepted_by_something = 1
    

    #------------------------- Action based functions end -----------------   

    #------------------------- Path based functions start ------------------
    def fix_graph(self):
        #Function ensures that all nodes are fully connected. Removes hanging nodes.
        tempNodes = []
        tempKeys = []
        tempDict = {}
        tempDif = []
        for i in self.graph:
            tempNodes.append(i)
        for i in self.graph:
            tempKeys = []
            for j in self.graph.get(i):
                tempKeys.append(j)
            tempDif = set(tempNodes) & set(tempKeys)
            tempDict[i] = tempDif
        # These lines fill in the dictionary to fix indexing issues
        for i in range(max(tempNodes)):
            if i not in tempNodes:
                tempDict[i] = []
        self.graph = tempDict



    def findShortestPath(self,s,e):
        prev = self.solve(s)
        #print(prev)
        return self.reconstructPath(s,e,prev)
    
    def solve(self,s):
        q = deque()
        q.append(s)
        visited = [0]*len(self.graph) 
        visited[s] = 1
        #print(visited)

        prev = [None]*len(self.graph)

        while  len(q) != 0:
            node = q.popleft()
            neighbours = self.graph.get(node)
            try: 
                for n in neighbours:
                    #if its not available will have issues all nodes must be initialized
                    if visited[n] == 0:
                        q.append(n)
                        visited[n] = 1
                        prev[n] = node
            except: return([])
        return prev
    


    def reconstructPath(self, s,e,prev):
        path = []
        path.append(e)  
        at = e
        counter = 0
        while at != s:
            counter = counter + 1
            try:
                at = prev[at]
                path.append(at)
            except:
                return([])

        rev_path = [0]*len(path)
        for i in range(len(path)):
            rev_path[i] = path[(len(path))-i-1]
        return rev_path
        #If s and e are connected return the path
        

    def list_callback(self, msg):
        temp_num_of_resource_holons = self.count_publishers('graph_node_network')
        #print(temp_num_of_resource_holons)
        self.graph[msg.node] = msg.adjacent
        if self.num_of_resource_holons != temp_num_of_resource_holons:
            self.graph = {}
            self.num_of_resource_holons = temp_num_of_resource_holons
        else:
            self.graph[msg.node] = msg.adjacent
        if len(self.graph) == temp_num_of_resource_holons:
            self.findPath = True
        else:
            self.findPath = False
        

#------------------------- Path based functions end ------------------

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
