import rclpy
import time
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from rclpy.action import ActionClient
from threading import Event
import json
import numpy as np
import math
# It includes always wait and send as base skills
#FIXME: Add a meta-class for all the Skills to have a common interface (SE-FIX)

def estimate_mov_time(pos_a, pos_b, velocity):
    dist = math.sqrt((pos_a[0] - pos_b[0])**2 + (pos_a[0] - pos_b[1])**2)
    t = dist / velocity
    return t


class Nav2Navigator():

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
        self.node.get_logger().info(f'Sending to Nav: {str(tgoal[0])}|{str(tgoal[1])}')
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


class NativeNavigator():
    
    def __init__(self, node, finish_event) -> None:
        self.node = node
        self.finish_event = finish_event
        self.cmd_pub = self.create_publisher(Twist, f'/{self.node.robot_name}/cmd_vel', 10)
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.node.robot_name}/odom',
            self.odom_callback,
            qos,
            callback_group=self.callback_group
        )

        # Control parameters
        self.control_params = {
            "k_linear": 0.12,
            "k_angular": 0.34,
            "max_linear": 0.7,
            "dist_threshold": 0.8,
            "sample_period": 0.35
        }

        self.current_pose = None
        self.goal_pose = None

        self.timer = self.create_timer(self.control_params["sample_period"], self.control_loop, callback_group=self.callback_group)
        self.get_logger().info("Navigator ready for goal commands")


    def set_goal_pose(self, goal):
        self.goal_pose = goal
        self.get_logger().info(f"Executing go to <goal: {goal}>")
        self.active_goal_handle = True



    # Robot Handling functions
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        if self.goal_pose is None or self.current_pose is None or self.active_goal_handle is None:
            return
        
        p_yaw = 0.0
        #FIXME: Add the current state object reference here
        x,y,yaw = self.get_current_state()

        #Take a single control step
        yaw = self.wrap_angle(p_yaw, yaw)
        p_yaw = yaw

        # Linear Velocity
        k_linear = self.control_params['k_linear']
        dx = self.goal_pose["x"] - self.current_pose.position.x
        dy = self.goal_pose["y"] - self.current_pose.position.y
        distance = hypot(dx, dy)
        linear_speed = min(self.control_params['max_linear'], distance * k_linear)

        # Angular Velocity
        k_angular = self.control_params['k_angular']
        desired_angle_goal = self.wrap_angle(p_ang, math.atan2(y_goal-y,x_goal-x))
        p_ang = desired_angle_goal
        # pos_msg = f"The current angle: {math.degrees(yaw)}  | Desired: {math.degrees(desired_angle_goal)} | Da: {math.degrees(desired_angle_goal-yaw)}"
        # self.get_logger().info(pos_msg)
        angular_speed = (desired_angle_goal-yaw)*k_angular

        vel_msg = Twist()
        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = angular_speed
        
        
        self.get_logger().info(f"The distance to the goal is: {distance}")
        if distance < self.control_params["dist_threshold"]:
        self.cmd_pub.publish(vel_msg)
        








    # helper functions
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    


# Skills
class WaitSkill():
    def __init__(self,node, params,finish_event, skill_name="") -> None:
        self.node = node
        self.finish_event = finish_event
        self.params = params
        self.success = False
        self.subs = self.node.create_subscription(String, '/signal_states', self.update_flags, 10)
        wait_str = self.params["target"]
        self.wait_flags = wait_str.split('&')
        self.wait_for_all = Event()
        self.node.get_logger().info(f"Starting up skill: {self.__class__.__name__}")
        
    
    def update_flags(self,msg):
        self.flags = json.loads(msg.data)
        if self.check_flags(self.wait_flags):
            self.wait_for_all.set()

    def check_flags(self, wf):
        for f in wf:
            if f not in self.flags:
                return False
        return True

    def exec(self, virtual_state=None,virtual_effort=None):
        self.wait_for_all.wait()
        self.success = True
        self.node.get_logger().info(f"Finishing up skill: {self.__class__.__name__}")
        self.finish_event.set()
        return virtual_state


class SendSkill():
    def __init__(self,node, params, finish_event, skill_name="") -> None:
        self.node = node
        self.finish_event = finish_event
        self.params = params
        self.success = False
        self.publisher = self.node.create_publisher(String, "/mission_signals",10)
    
    def exec(self, virtual_state,virtual_effort=None):
        # It needs params["target"]
        task_id = self.params["target"]
        msg = String()
        msg.data = task_id
        self.publisher.publish(msg)
        self.success = True
        self.finish_event.set()
        return virtual_state
        

class MopSkill():
    def __init__(self, node, params, finish_event, skill_name="") -> None:
        # Starting skill : vmop
        self.params = params
        self.node = node
        self.finished_event = finish_event
        self.success = False

        # Create a navigator obj
        self.wait_for_nav = Event()
        self.nav = Navigator(self.node, self.wait_for_nav)
        self.node.get_logger().info(f"Starting up skill: {self.__class__.__name__}")
    
    def exec(self,virtual_state=None):
        # It needs: params["room"]["size"]
        print("Received the params: ")
        goal_pos = [self.params["location"]["x"],self.params["location"]["y"],0.0]
        self.nav.send_goal(goal_pos)
        self.wait_for_nav.wait()
        self.success = self.nav.nav_success
        self.finished_event.set()
        self.node.get_logger().info(f"Finishing up skill: {self.__class__.__name__}")
        return virtual_state


class VacuumSkill():
    def __init__(self, node, params, finish_event, skill_name="") -> None:
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
        goal_pos = [self.params["location"]["x"],self.params["location"]["y"],0.0]
        self.nav.send_goal(goal_pos)
        self.wait_for_nav.wait()
        self.success = self.nav.nav_success
        self.finished_event.set()
        self.node.get_logger().info("Finishing up running skill: Vacuum")
        return True

class PolishSkill():
    def __init__(self, node, params, finish_event, skill_name="") -> None:
        # Starting skill : vmop
        self.params = params
        self.node = node
        self.finished_event = finish_event
        self.success = False

        # Create a navigator obj
        self.wait_for_nav = Event()
        self.nav = Navigator(self.node, self.wait_for_nav)
        self.node.get_logger().info("Starting up skill: Polish")
    
    def exec(self):
        # It needs: params["room"]["size"]
        print("Received the params: ")
        goal_pos = [self.params["location"]["x"],self.params["location"]["y"],0.0]
        self.nav.send_goal(goal_pos)
        self.wait_for_nav.wait()
        self.success = self.nav.nav_success
        self.finished_event.set()
        self.node.get_logger().info("Finishing up running skill: Polish")
        return True

# Virtual Skills
#SE-FIX: Create a single class VirtualNavigate skill, and since the skill manager instantiates them,
#        you can provide a specific skill name in the constructor

class VMopSkill():
    def __init__(self, node, params, finish_event, skill_name="") -> None:
        # Starting skill : vmop
        self.params = params
        self.node = node
        self.finished_event = finish_event
        self.success = False
        self.node.get_logger().info(f"Starting up skill: {self.__class__.__name__}")

    
    def exec(self, virtual_state=None, virtual_effort=None):
        goal_pos = [self.params["location"]["x"],self.params["location"]["y"],0.0]
        vpos = [virtual_state["x"],virtual_state["y"],virtual_state["z"]]
        
        time_to_goal = estimate_mov_time(vpos, goal_pos, velocity=0.3)
        self.node.get_logger().info(f"Simulating going from [{str(vpos[0])},{str(vpos[1])}] to [{str(goal_pos[0])},{str(goal_pos[1])}]... [{time_to_goal} secs]")
        time.sleep(time_to_goal)
        self.node.get_logger().info(f"Simulating effort in {self.__class__.__name__}  [{virtual_effort} secs]")
        time.sleep(virtual_effort)
        self.success = True
        self.finished_event.set()
        self.node.get_logger().info(f"Finishing up skill: {self.__class__.__name__}")
        virtual_state = {
            "x": goal_pos[0],
            "y": goal_pos[1],
            "z": goal_pos[2]
        }
        return virtual_state

class VVacuumSkill():
    def __init__(self, node, params, finish_event, skill_name="") -> None:
        # Starting skill : vmop
        self.params = params
        self.node = node
        self.finished_event = finish_event
        self.success = False
        self.node.get_logger().info(f"Starting up skill: {self.__class__.__name__}")

    
    def exec(self, virtual_state=None, virtual_effort=None):
        goal_pos = [self.params["location"]["x"],self.params["location"]["y"],0.0]
        vpos = [virtual_state["x"],virtual_state["y"],virtual_state["z"]]

        time_to_goal = estimate_mov_time(vpos, goal_pos, velocity=0.3)

        self.node.get_logger().info(f"Simulating going from [{str(vpos[0])},{str(vpos[1])}] to [{str(goal_pos[0])},{str(goal_pos[1])}]... [{time_to_goal} secs]")
        time.sleep(time_to_goal)
        self.node.get_logger().info(f"Simulating effort in {self.__class__.__name__}  [{virtual_effort} secs]")
        time.sleep(virtual_effort)
        self.success = True
        self.finished_event.set()
        self.node.get_logger().info(f"Finishing up skill: {self.__class__.__name__}")
        virtual_state = {
            "x": goal_pos[0],
            "y": goal_pos[1],
            "z": goal_pos[2]
        }
        return virtual_state

class VPolishSkill():
    def __init__(self, node, params, finish_event, skill_name="") -> None:
        # Starting skill : vmop
        self.params = params
        self.node = node
        self.finished_event = finish_event
        self.success = False
        self.node.get_logger().info(f"Starting up skill: {self.__class__.__name__}")

    
    def exec(self, virtual_state=None, virtual_effort=None):
        goal_pos = [self.params["location"]["x"],self.params["location"]["y"],0.0]
        vpos = [virtual_state["x"],virtual_state["y"],virtual_state["z"]]
        time_to_goal = estimate_mov_time(vpos, goal_pos, velocity=0.3)
        self.node.get_logger().info(f"Simulating going from [{str(vpos[0])},{str(vpos[1])}] to [{str(goal_pos[0])},{str(goal_pos[1])}]... [{time_to_goal} secs]")
        time.sleep(time_to_goal)
        self.node.get_logger().info(f"Simulating effort in {self.__class__.__name__}  [{virtual_effort} secs]")
        time.sleep(virtual_effort)
        self.success = True
        self.finished_event.set()
        self.node.get_logger().info(f"Finishing up skill: {self.__class__.__name__}")
        virtual_state = {
            "x": goal_pos[0],
            "y": goal_pos[1],
            "z": goal_pos[2]
        }
        return virtual_state
    
class VirtualSKill():
    def __init__(self, node, params, finish_event, skill_name="") -> None:
        # Starting skill : vmop
        self.params = params
        self.node = node
        self.skill_name = skill_name
        self.finished_event = finish_event
        self.success = False
        self.node.get_logger().info(f"Starting up skill: {self.skill_name}")
    
    def exec(self, virtual_state=None, virtual_effort=None):
        self.node.get_logger().info(f"Simulating effort in {self.skill_name}  [{virtual_effort} secs]")
        time.sleep(virtual_effort)
        self.success = True
        self.finished_event.set()
        self.node.get_logger().info(f"Finishing up skill: {self.skill_name}")
        return virtual_state



# Skill Manager
# SE-FIX: Move the instantiation of the classes here, to avoid the need to rebring up the navigators
class SkillManager():
    def __init__(self, skill_mask) -> None:
        """
        skill_mask: if specified, then only those skills are created inside the SKmanager
        """
        self.sk_map = {
            "wait_until": WaitSkill,
            "send_signal": SendSkill,
            "mop": VMopSkill,
            "vacuum": VVacuumSkill,
            "polish": VPolishSkill,
        }
        general_skills = ["scan_perimeter", "thermal_scan","record_video", "analyze_surveillance_data", "alert_security","follow_movement","capture_image","drone_scan","soil_moisture_analysis","data_analysis","apply_treatment","fertilizer_application","pest_control_spray","irrigation_adjustment","drone_recheck","soil_nutrient_test"]
        for skill in general_skills:
            self.sk_map[skill] = VirtualSKill

        
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
