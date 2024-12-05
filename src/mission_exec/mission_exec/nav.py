import rclpy
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml
import sys
import os
import json

class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('nav_to_pose_action_client')
        self._action_client = ActionClient(self, NavigateToPose, '/Turtlebot_02490/navigate_to_pose')
        self.create_subscription(String,'/mytest',self.test_callback,10)
        # self.create_subscription(String, '/execute', self.execute_mission, 10)
        self.get_logger().info('Initializing the navigator')
        self.goal_poses = [[0.5,0.5]]
        self.tgoal = [-4.0,-0.4, 0.0]
        self.send_goal()


    def execute_mission(self, msg):
        self.get_logger().info('Received: ' + msg.data)
        # goal = json.loads(msg.data)
        goal = msg.data.split('_')
        #self.tgoal = [float(goal[0]),float(goal[1]), 0.0]
        self.tgoal = [-2.0,0.0, 0.0]
        self.send_goal()

    def test_callback(self,msg):
        self.get_logger().info('Data is: '+msg.data)

    def send_goal(self):
        # Set up the goal pose
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        # self.get_logger().info('Sending: ' + ' '.join(self.tgoal))
        goal_pose.pose.pose.position.x = self.tgoal[0]
        goal_pose.pose.pose.position.y = self.tgoal[1]
        goal_pose.pose.pose.orientation.z = self.tgoal[2]
        # goal_pose.pose.pose.orientation.w = spot['orientation']['w']

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_pose,feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Goal was rejected.{goal_handle}')
            return
        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal result received. {result}')
        #rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback: {feedback_msg.feedback}")



def main(args=None):
    rclpy.init(args=args)
    action_client = NavToPoseActionClient()
    # Send the goal to the spot specified in the command line
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()