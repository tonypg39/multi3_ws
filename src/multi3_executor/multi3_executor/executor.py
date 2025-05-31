import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, HistoryPolicy
from std_msgs.msg import String
from .skills import SkillManager
from geometry_msgs.msg import Pose
from threading import Event
import json
import time
from tf_transformations import quaternion_from_euler
from ament_index_python import get_package_prefix
from multi3_interfaces.srv import Fragment
from irobot_create_msgs.srv import ResetPose

class FragmentExecutor(Node):
    def __init__(self) -> None:
        super().__init__(f'fragment_exec_node')
        self.robot_name = "robot1"
        self.declare_parameter("skill_list", "-")
        self.declare_parameter("name", "robot")
        self.declare_parameter("mode", "virtual")
        self.declare_parameter("test_id", "")
        self.declare_parameter("tbot_mapping", "")
        self.declare_parameter("sample_id", "")
        self.callback_group = ReentrantCallbackGroup()

        self.tbot_mapping = self.get_parameter("tbot_mapping").value
        self.get_logger().info(f"The value of TbotMapping is: {self.tbot_mapping}")
        self.skill_list = self.get_parameter("skill_list").value
        self.robot_name = self.get_parameter("name").value
        test_id = self.get_parameter("test_id").value
        sample_id = self.get_parameter("sample_id").value[1:]
        self.virtual_mode = self.get_parameter("mode").value == "virtual"
        self.initial_position = None
        self.env_states = None
        self.get_logger().info(f"Starting an exec node [{self.robot_name}] ")

        if self.tbot_mapping != "":
            tbots = self.tbot_mapping[3:].split(",")
            bot_idx = self.robot_name.split("_")[1]
            tbot_name = tbots[int(bot_idx) - 1]
            # FIXME: SE-Fix generalize the namespacing to any application
            self.real_robot_namespace = f"Turtlebot_024{tbot_name}"
        else:
            self.real_robot_namespace = None
        
        #FIXME: Add to json
        self.settings = {
            "heartbeat_period": 3.0
        }
        self.info_task = ""
        if self.virtual_mode:
            self.virtual_state = {
                "x": .0,
                "y": .0,
                "z": .0
            }
            if test_id == "" or sample_id == "":
                self.get_logger().fatal("Test Id and Sample Id needed in Virtual mode")
                return
            self.env_states = self.read_env_states(test_id, int(sample_id))
        else:
            self.virtual_state = None
            self.initial_position = self.read_initial_position(test_id)
            self.start_reset_srv()
            self.reset_odometry(self.initial_position)


        sk_mg = SkillManager(skill_mask=self.skill_list, virtual_mode=self.virtual_mode)
        self.sk_map = sk_mg.skill_map()
        # print(self.sk_map)
        # Create the service 
        self._start_srv()

        # Reset initial position service

        # Create service client for reset_pose
        
        # Create General communication channels 
        self.hearbeat_pub = self.create_publisher(String, "/hb_broadcast", 10)
        self.signal_subscription = self.create_subscription(String, '/signal_states', self.check_signals, 10)
        self.signal_pub_timer = self.create_timer(self.settings['heartbeat_period'],self._send_heartbeat)
        self.busy = False
    
    def read_initial_position(self, test_id):
        package_path = get_package_prefix("multi3_tests").replace("install","src")
        # print(package_path)
        with open(f"{package_path}/multi3_tests/tests/{test_id}/initial_positions.json") as f:
            positions = json.load(f)
        initial_pos = positions[self.robot_name]
        return initial_pos

    def start_reset_srv(self):
        qos_profile = QoSProfile(
            depth=10,  # increase depth to allow larger history buffer
            history=HistoryPolicy.KEEP_LAST
        )
        service_name = f'/{self.real_robot_namespace}/reset_pose'
        self.reset_pose_client = self.create_client(ResetPose, service_name, qos_profile=qos_profile)

        #Debugging service discovery
        # self.list_services()

        services = self.get_service_names_and_types()
        self.get_logger().info(f'Waiting for service {service_name}...')
        # while not self.reset_pose_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info(f'Service {service_name} not available, waiting...')
    
    def list_services(self):
        services = self.get_service_names_and_types()
        self.get_logger().info('--- Available services ---')
        for (srv_name, srv_types) in services:
            self.get_logger().info(f'Service: {srv_name}, Types: {srv_types}')

    def reset_odometry(self, initial_position):
        request = ResetPose.Request()
        quat = quaternion_from_euler(0.0, 0.0, initial_position["yaw"])
        initial_pose = Pose()
        initial_pose.position.x = initial_position["x"]
        initial_pose.position.y = initial_position["y"]
        initial_pose.position.z = 0.0
        initial_pose.orientation.x = quat[0]
        initial_pose.orientation.y = quat[1]
        initial_pose.orientation.z = quat[2]
        initial_pose.orientation.w = quat[3]
        request.pose = initial_pose

        self.get_logger().info(f'Calling {self.real_robot_namespace}/reset_pose with initial pose...')

        future = self.reset_pose_client.call_async(request)

        # Spin until the future is done
        time.sleep(2)

        # try:
        #     response = future.result()
        #     self.get_logger().info('Odometry reset successfully!')
        # except Exception as e:
        #     self.get_logger().error(f'ResetPose service call failed: {e}')
        

    def check_signals(self, msg):
        self.flags = json.loads(msg.data)
        if "_SHUTDOWN_" in self.flags:
            self.destroy_node()

    def read_env_states(self, test_id, sample_id):
        package_path = get_package_prefix("multi3_tests").replace("install","src")
        # print(package_path)
        with open(f"{package_path}/multi3_tests/tests/{test_id}/env_states.json") as f:
            envs = json.load(f)
        env_states = envs[sample_id]
        return env_states


    def _send_heartbeat(self):
        m = String()
        state = "idle" if not self.busy else "busy " + self.info_task
        m.data = self.robot_name + "=" + state
        self.hearbeat_pub.publish(m)
        # self.get_logger().info(f"\n\n\nSending Heartbeat!! = Current Task: {self.info_task}")

    def _start_srv(self):
        # self.create_service()
        # print("h1")
        self.get_logger().info("Advertising the service...")
        self.srv = self.create_service(Fragment, f'/{self.robot_name}/get_fragment', self.exec, callback_group=self.callback_group)

    
    def _stop_srv(self):
        self.srv.destroy()
        self.srv = None


    def exec(self, request, response):
        # self._stop_srv()
        self.busy = True
        self._send_heartbeat()
        failure = False

        # self.get_logger().info("Received fragment: " + request.fragment)
        frag = json.loads(request.fragment)
        for t in frag["tasks"]:
            self.info_task = t["id"]
            sep = t["id"].find("|")
            mi_sep = t["id"].find("^")
            virtual_effort = None
            if sep > -1: # if it a wait or send, augment the call with the target task(s)
                t["vars"]["target"] = t["id"][sep+1:]
                t["id"] = t["id"][:sep]
            
            elif mi_sep > -1:
                core_task = t["id"][:mi_sep]
                if self.virtual_mode:
                    self.get_logger().info(f"The actual core task is: {core_task}")
                    self.get_logger().info(f"The id is: {int(t['id'][mi_sep+1:])}")
                    self.get_logger().info(f"The env_states are is: {self.env_states}")
                    virtual_effort = self.env_states[core_task][int(t["id"][mi_sep+1:])]
                t["id"] = core_task

            elif t["id"] in self.env_states:
                virtual_effort = self.env_states[t["id"]]
            wait_for_skill = Event()
            self.get_logger().info(f"Starting the skill {t['id']}  with params {t['vars']} and v. effort {virtual_effort}")
            sk = self.sk_map[t["id"]](self, t["vars"],wait_for_skill,skill_name=self.info_task)

            self.virtual_state = sk.exec(self.virtual_state, virtual_effort)
            wait_for_skill.wait()
            failure |= sk.success

        response.execution_code = 0 if not failure else 1
        self.busy = False
        self.info_task = ""
        return response
    
    

def main(args=None):
    rclpy.init(args=args)
    bot_exec = FragmentExecutor()
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(bot_exec)
    mt_executor.spin()
    # rclpy.spin(bot_exec)
    bot_exec.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
