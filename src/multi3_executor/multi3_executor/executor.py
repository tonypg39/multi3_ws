import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from .skills import SkillManager
from threading import Event
import json
import time
from ament_index_python import get_package_prefix
from multi3_interfaces.srv import Fragment

class FragmentExecutor(Node):
    def __init__(self) -> None:
        super().__init__(f'fragment_exec_node')
        self.robot_name = "robot1"
        self.declare_parameter("skill_list", "-")
        self.declare_parameter("name", "robot")
        self.declare_parameter("mode", "virtual")
        self.declare_parameter("test_id", "")
        self.declare_parameter("sample_id", "")
        self.callback_group = ReentrantCallbackGroup()
        self.skill_list = self.get_parameter("skill_list").value
        self.robot_name = self.get_parameter("name").value
        test_id = self.get_parameter("test_id").value
        sample_id = self.get_parameter("sample_id").value[1:]
        self.virtual_mode = self.get_parameter("mode").value == "virtual"
        self.env_states = None
        self.get_logger().info(f"Starting an exec node [{self.robot_name}] with skills: " + self.skill_list)

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
        

        sk_mg = SkillManager(skill_mask=self.skill_list)
        self.sk_map = sk_mg.skill_map()
        # print(self.sk_map)
        # Create the service 
        self._start_srv()
        
        # Create General communication channels 
        self.hearbeat_pub = self.create_publisher(String, "/hb_broadcast", 10)
        self.signal_pub_timer = self.create_timer(self.settings['heartbeat_period'],self._send_heartbeat)
        self.busy = False

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
                    virtual_effort = self.env_states[core_task][int(t["id"][mi_sep+1:])]
                t["id"] = core_task

            wait_for_skill = Event()
            self.get_logger().info(f"Starting the skill {t['id']}  with params {t['vars']} and v. effort {virtual_effort}")
            sk = self.sk_map[t["id"]](self, t["vars"],wait_for_skill)
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
    
