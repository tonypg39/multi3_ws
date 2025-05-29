# Coordinator Tasks
# - Listen to /mission_comms and update the flag_table
# assign the available fragment to idle robots
import json
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from multi3_interfaces.srv import Fragment
from ament_index_python import get_package_prefix
import sys

class CoordinatorNode(Node):
    def __init__(self):
        super().__init__("multi3_coordinator")
        self.coord_settings = {
            "signal_states_period": 1.0,
            "assignment_period": 1.5
        }
        self.shutdown_count = -1
        self.idle_robots = {}
        self.robot_states = {}
        self.signal_states = ['SYSTEM_START']
        
        # Declare Parameters
        self.declare_parameter("test_id", "")
        self.declare_parameter("mode", "")

        test_id = self.get_parameter("test_id").value
        mode = self.get_parameter("mode").value
        self.get_logger().info("$$**MISSION_START**$$")
        self.get_logger().info(f"Starting the Coordinator node with params ==> test_id = {test_id} || mode = {mode}")
        if test_id == "":
            self.get_logger().fatal("No test_id specified!!")
            return
        
        if test_id == "":
            self.get_logger().fatal("No mode specified!!")
            return
    
        self.robot_inventory = self.read_inventory(test_id)
        fragments = self.read_fragments(test_id, mode)

        self.create_subscription(String, "/mission_signals",self.update_signal_state,10)
        self.create_subscription(String, "/hb_broadcast",self.update_hb,10)
        
        
        self.signal_publisher = self.create_publisher(String, '/signal_states', 10)
        self.signal_pub_timer = self.create_timer(self.coord_settings['signal_states_period'],self.broadcast_signal_states)
        self.assignment_timer = self.create_timer(self.coord_settings['assignment_period'],self.assign)
        self.fragments = self.load_fragments(fragments)
        # self.get_logger().info(f"The loaded fragments: {self.fragments}")
        # Dict to keep track of the states of executed fragments 
        # (For a frag to have a key here, the fragment must've been sent )
        self.fragments_futures = {} 
        
        

    def read_fragments(self, test_id, mode):
        package_path = get_package_prefix("multi3_tests").replace("install","src")
        # print(package_path)
        with open(f"{package_path}/multi3_tests/tests/{test_id}/tasks_{mode}.json") as f:
            frags = json.load(f)
        return frags

    def read_inventory(self, test_id):
        package_path = get_package_prefix("multi3_tests").replace("install","src")
        # print(package_path)
        with open(f"{package_path}/multi3_tests/tests/{test_id}/inventory.json") as f:
            inventory = json.load(f)
        return inventory
    


    def load_fragments(self, fragments):
        c = 0
        F = {}
        for fr in fragments:
            fr_obj = {
                "status": "waiting",
                "fragment_id": f"fr_{c}",
                "age": 0,
            }
            fr_obj.update(fr)
            F[fr_obj["fragment_id"]] = fr_obj
            c += 1
        return F

    def update_hb(self, msg):
        st = msg.data.split("=")
        if not st[0] in self.robot_inventory:
            self.get_logger().warning(f"Ignoring robot '{st[0]}' as it is not in inventory")
            return 
        if st[1] == "idle":
            self.idle_robots[st[0]] = True
        else:
            self.idle_robots[st[0]] = False
        
        self.robot_states[st[0]] = st[1]
        


    def update_signal_state(self, msg):
        new_signal = msg.data
        if new_signal not in self.signal_states:
            self.signal_states.append(new_signal)
    
    
    # Publish the signal_states periodically using a Timer object
    def broadcast_signal_states(self):
        message = String()
        signal_list = self.signal_states
        if self.shutdown_count > -1:
            if self.shutdown_count == 0:
                self.get_logger().info("$$*MISSION_STOPPED*$$")
                self.assignment_timer.cancel()
                self.signal_pub_timer.cancel()
                self.shutdown_count = 5
                self.destroy_node()
                rclpy.shutdown()
                sys.exit(0)
            signal_list.append("_SHUTDOWN_")
            self.shutdown_count -= 1
        message.data = json.dumps(signal_list)
        # self.get_logger().info(f"Sending signals info: {message.data}")
        self.signal_publisher.publish(message)
    
    def check_active_fragments(self):
        active_frags = []
        for frag in self.fragments:
            if frag['status'] == "waiting":
                active_frags.append(frag)
            if frag['status'] == "blocked":
                w_flags = frag["initial_wait"].split('&')
                missing_flag = False
                for f in w_flags:
                    if f not in self.signal_states:
                        missing_flag = True
                if not missing_flag:
                    self.fragments[frag["fragment_id"]]["status"] = "waiting"
                    active_frags.append(frag)
        return active_frags
    
    def check_eligibility(self, robot,fragment):
        able = True        
        for t in fragment["tasks"]:
            if t["id"].find("|") > -1: # If it is either a signal or a wait 
                continue
            if self.get_core_task(t["id"]) not in self.robot_inventory[robot]:
                able = False
        self.get_logger().info(f"Checking elegibility for {robot} and tasks {fragment['tasks']} = {able}")
        return able
    
    def get_core_task(self, task):
        sep = task.find("^")
        if sep > -1:
            return task[:sep]
        return task
    
    def generate_assigments(self, robots, fragments):
        assignment_dict = {}
        sorted_frags = sorted(fragments, key= lambda x: x["age"], reverse=True)
        used_robots = set()
        for f in sorted_frags:
            # self.get_logger().warning(f"Inside sorted frags {robots}")

            if "robot" in f.keys():
                # STATIC ASSIGNMENT, we expect the keyword robot associated with each fragment
                # The fragments are a construct with all of the tasks assigned to a specific robot
                if f["robot"] in robots:
                    self.fragments[f["fragment_id"]]["status"] = "executed"
                    assignment_dict[f["robot"]] = f.copy()
                continue
            for r in robots:
                if r in used_robots:
                    continue
                if self.check_eligibility(r,f):
                    self.fragments[f["fragment_id"]]["status"] = "executed"
                    assignment_dict[r] = f.copy()
                    used_robots.add(r)
                    break
        return assignment_dict
            

        
    def get_idle_robots(self):
        # get available services 
        ir = []
        for k,v in self.idle_robots.items():
            if v:
                ir.append(k)
        return ir
    
    def get_active_fragments(self):
        active_frags = []
        for frag in self.fragments.values():
            if frag["status"] == "waiting":
                w_flags = frag['initial_wait'].split('&')
                missing_signal = any(fl not in self.signal_states for fl in w_flags)
                if not missing_signal:
                    active_frags.append(frag)
        return active_frags
    
    def send_assignment(self, robot, fragment):
        req = Fragment.Request()
        req.fragment = json.dumps(fragment)
        
        cli = self.create_client(Fragment, f'/{robot}/get_fragment')
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.fragments_futures[fragment["fragment_id"]] = cli.call_async(req)
        
    def log_fragments(self, fragments, label):
        for f in fragments:
            tasks = []
            for t in f["tasks"]:
                tasks.append(t["id"])
            # print(tasks)
            self.get_logger().info(f'{label} [{f["fragment_id"]}] ==> [{",".join(tasks)}]')

    def log_futures(self):
        st = "Futures: "
        for k,v in self.fragments_futures.items():
            st += f"Fragment: {k} | State: {v.done()}||"
        self.get_logger().info(st)

    def log_robots(self):
        s = "$$**ROBOTS_STATE**$$|| "
        for k,v in self.robot_states.items():
            s += f"{k} => {v} || "
        self.get_logger().info(s)

    def check_finished(self):
        # return False
        for f_id,frag in self.fragments.items():
            executed = frag["status"] == "executed"
            finished = False
            if executed and self.fragments_futures[f_id].done():
                finished = True
            # self.get_logger().info(f"\n{f_id} -> executed = {str(executed)} | finished = {str(finished)}")
            if not executed or not finished:
                # self.get_logger().info(f"{f_id} -> executed = {str(executed)} | finished = {str(finished)}")
                return False
        return True
        


    def assign(self):
        if self.shutdown_count > -1:
            return
        if self.check_finished():
            self.get_logger().info("$$*MISSION_COMPLETED*$$")
            self.shutdown_count = 5
            # self.destroy_node()
        self.get_logger().info("------Assignment window------")
        robots = self.get_idle_robots()
        fragments = self.get_active_fragments()
        
        self.log_fragments(fragments, "Active fragment")
        # self.log_futures()
        self.log_robots()
        # self.get_logger().info(f"The active robots are: {robots}")
        assignments = self.generate_assigments(robots, fragments)
        # print(assignments)
        
        for k,v in assignments.items():
            # print("Sending assignment: ", k,v)
            self.send_assignment(k,v)
        # Increase the age of the unpicked fragments
        for f in fragments:
            if self.fragments[f["fragment_id"]]["status"] == "waiting":
                self.fragments[f["fragment_id"]]["age"] += 1
        


# def read_fragments():
#     package_path = get_package_prefix("multi3_coordinator").replace("install","src")
#     # print(package_path)
#     with open(f"{package_path}/multi3_coordinator/tasks.json") as f:
#         frags = json.load(f)
#     return frags

def main(args=None):
    rclpy.init(args=args)
    # fragments = read_fragments()
    coord = CoordinatorNode()
    rclpy.spin(coord)
    coord.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()