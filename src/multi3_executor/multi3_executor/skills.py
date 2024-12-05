import rclpy
import time
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from rclpy.action import ActionClient
from threading import Event
import json
import numpy as np
# It includes always wait and send as base skills

class Navigator():

    def __init__(self, node, finish_event) -> None:
        self.node = node
        self.finish_event = finish_event
        self.nav_success = False
        self._action_client = ActionClient(self.node, NavigateToPose, f'/{self.node.robot_name}/navigate_to_pose', callback_group=self.node.callback_group)
        self.node.get_logger().info('Initializing the navigator')

    def send_goal(self, tgoal):
        # Set up the goal pose
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        self.node.get_logger().info(f'Sending to Nav: {str(tgoal[0])}|{str(tgoal[0])}')
        goal_pose.pose.pose.position.x = tgoal[0]
        goal_pose.pose.pose.position.y = tgoal[1]
        goal_pose.pose.pose.orientation.z = tgoal[2]
        # goal_pose.pose.pose.orientation.w = spot['orientation']['w']

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_pose,feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info(f'Goal was rejected.{goal_handle}')
            self.finish_event.set()
            return
        self.node.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.nav_success = True
        self.node.get_logger().info(f'Goal result received.')
        self.finish_event.set()
        #rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        self.node.get_logger().info(f"Feedback: {feedback_msg.feedback}")

# Skills
class WaitSkill():
    def __init__(self,node, params,finish_event) -> None:
        self.node = node
        self.finish_event = finish_event
        self.params = params
        self.success = False
        self.subs = self.node.create_subscription(String, '/signal_states', self.update_flags, 10)
        wait_str = self.params["target"]
        self.wait_flags = wait_str.split('&')
        self.wait_for_all = Event()
        
    
    def update_flags(self,msg):
        self.flags = json.loads(msg.data)
        if self.check_flags(self.wait_flags):
            self.wait_for_all.set()

    def check_flags(self, wf):
        for f in wf:
            if f not in self.flags:
                return False
        return True

    def exec(self):
        self.wait_for_all.wait()
        self.success = True
        self.finish_event.set()
        return True


class SendSkill():
    def __init__(self,node, params, finish_event) -> None:
        self.node = node
        self.finish_event = finish_event
        self.params = params
        self.success = False
        self.publisher = self.node.create_publisher(String, "/mission_signals",10)
    
    def exec(self):
        # It needs params["target"]
        task_id = self.params["target"]
        msg = String()
        msg.data = task_id
        self.publisher.publish(msg)
        self.success = True
        self.finish_event.set()
        return True
        

class MopSkill():
    def __init__(self, node, params, finish_event) -> None:
        # Starting skill : vmop
        self.params = params
        self.node = node
        self.finished_event = finish_event
        self.success = False

        # Create a navigator obj
        self.wait_for_nav = Event()
        self.nav = Navigator(self.node, self.wait_for_nav)
        self.node.get_logger().info("Starting up skill: Mop")
    
    def exec(self):
        # It needs: params["room"]["size"]
        print("Received the params: ")
        goal_pos = [self.params["door"]["x"],self.params["door"]["y"],0.0]
        self.nav.send_goal(goal_pos)
        self.wait_for_nav.wait()
        self.success = self.nav.nav_success
        self.finished_event.set()
        self.node.get_logger().info("Finishing up running skill: Mop")
        return True



class VacuumSkill():
    def __init__(self, node, params, finish_event) -> None:
        # Starting skill : vmop
        self.params = params
        self.node = node
        self.finished_event = finish_event
        self.success = False

        # Create a navigator obj
        self.wait_for_nav = Event()
        self.nav = Navigator(self.node, self.wait_for_nav)
        self.node.get_logger().info("Starting up skill: Vacuum")
    
    def exec(self):
        # It needs: params["room"]["size"]
        print("Received the params: ")
        goal_pos = [self.params["door"]["x"],self.params["door"]["y"],0.0]
        self.nav.send_goal(goal_pos)
        self.wait_for_nav.wait()
        self.success = self.nav.nav_success
        self.finished_event.set()
        self.node.get_logger().info("Finishing up running skill: Vacuum")
        return True

# Skill Manager


class SkillManager():
    def __init__(self, skill_mask) -> None:
        """
        skill_mask: if specified, then only those skills are created inside the SKmanager
        """
        self.sk_map = {
            "wait_until": WaitSkill,
            "send_signal": SendSkill,
            "mop": MopSkill,
            "vacuum": VacuumSkill
        }
        self.sk_map = self.filter_skills(self.sk_map,skill_mask)
    
    def filter_skills(self, sk_map, mask):
        if mask == "-": # If no skill mask provided, we left it intact
            return sk_map
        new_sk_map = {}
        sk_list = mask.split(",")
        for k,v in sk_map.items():
            if k in sk_list or k=="send_signal" or k=="wait_until":
                new_sk_map[k] = v
        return new_sk_map

    def skill_map(self):
        return self.sk_map
