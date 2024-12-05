import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .skills import SkillManager
from threading import Event
import json
import time
from multi3_interfaces.srv import Fragment

class FragmentExecutor(Node):
    def __init__(self) -> None:
        super().__init__(f'fragment_exec_node')
        self.robot_name = "robot1"
        self.declare_parameter("skill_list", "-")
        self.declare_parameter("name", "robot")
        
        self.skill_list = self.get_parameter("skill_list").value
        self.robot_name = self.get_parameter("name").value
        self.get_logger().info(f"Starting an exec node [{self.robot_name}] with skills: " + self.skill_list)

        #FIXME: Add to json
        self.settings = {
            "heartbeat_period": 3.0
        }

        sk_mg = SkillManager(skill_mask=self.skill_list)
        self.sk_map = sk_mg.skill_map()
        print(self.sk_map)
        # Create the service 
        self._start_srv()
        
        # Create General communication channels 
        self.hearbeat_pub = self.create_publisher(String, "/hb_broadcast", 10)
        self.signal_pub_timer = self.create_timer(self.settings['heartbeat_period'],self._send_heartbeat)
        self.busy = False

    def _send_heartbeat(self):
        m = String()
        state = "idle" if not self.busy else "busy"
        m.data = self.robot_name + "=" + state
        self.hearbeat_pub.publish(m)

    def _start_srv(self):
        # self.create_service()
        # print("h1")
        self.get_logger().info("Advertising the service...")
        self.srv = self.create_service(Fragment, f'/{self.robot_name}/get_fragment', self.exec)

    
    def _stop_srv(self):
        self.srv.destroy()
        self.srv = None


    def exec(self, request, response):
        # self._stop_srv()
        self.busy = True
        self._send_heartbeat()
        failure = False

        self.get_logger().info("Received fragment: " + request.fragment)
        frag = json.loads(request.fragment)
        for t in frag["tasks"]:
            sep = t["id"].find("|")
            if sep > -1: # if it a wait or send, augment the call with the target task(s)
                t["vars"]["target"] = t["id"][sep+1:]
                t["id"] = t["id"][:sep]
            
            wait_for_skill = Event()
            sk = self.sk_map[t["id"]](self, t["vars"],wait_for_skill)
            sk.exec()
            wait_for_skill.wait()
            failure |= sk.success

        response.execution_code = 0 if not failure else 1
        self.busy = False
        return response
    
    

def main(args=None):
    rclpy.init(args=args)
    bot_exec = FragmentExecutor()
    rclpy.spin(bot_exec)
    bot_exec.destroy_node()
    bot_exec.shutdown()

if __name__ == '__main__':
    main()
    
