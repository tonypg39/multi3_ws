import random
import os
import json
import time
from collections import defaultdict
import rclpy
from rclpy.node import Node
from ament_index_python import get_package_prefix
from .gen_fragments import generate_fragmented_plans, generate_multi3_plans, generate_bl_plans


def generate_inventory(skills, num_robots):

    """
    Generate an inventory that assigns skills to robots, ensuring all skills are covered.

    :param skills: List of skills to be assigned.
    :param num_robots: Number of robots.
    :return: Dictionary with robot IDs as keys and their skills as values.
    """
    if num_robots <= 0:
        raise ValueError("Number of robots must be greater than 0.")
    if not skills:
        raise ValueError("Skills list cannot be empty.")

    # Shuffle skills to randomize assignment
    random.shuffle(skills)
    n_skills = len(skills)

    # Assign skills to robots
    robot_inventory = defaultdict(list)
    for i, skill in enumerate(skills):
        robot_id = f"robot_{i % num_robots + 1}"
        robot_inventory[robot_id].append(skill)

    # Augment the skills per robot randomly
    for i in range(1,num_robots+1):
        s_num = random.randint(1,(1<<n_skills)-1)
        robot_id = f"robot_{i}"
        for j in range(n_skills):
            if s_num & (1<<j) and skills[j] not in robot_inventory[robot_id]:
                robot_inventory[robot_id].append(skills[j])

    # Convert defaultdict to regular dictionary
    return dict(robot_inventory)

def generate_mission_data(mission_size, room_limits):
    d = {
        "instances" : {
            "vacuum": [],
            "mop": [],
            "polish": [],
        }
    }
    for i in range(mission_size):
        s_x = room_limits[0] + (room_limits[1]-room_limits[0])*random.random()
        s_y = room_limits[0] + (room_limits[1]-room_limits[0])*random.random()
        d["instances"]["vacuum"].append({"door":{"x":s_x,"y":s_y}})
        d["instances"]["mop"].append({"door":{"x":s_x,"y":s_y}})
        d["instances"]["polish"].append({"door":{"x":s_x,"y":s_y}})
    return d

def generate_env_state(mission_size, effort_limits):
    time_ranges = {
        "mop": effort_limits,
        "vacuum": effort_limits,
        "polish": effort_limits
    }
    env_states = {
        "mop": [],
        "vacuum": [],
        "polish": []
    }
    for i in range(mission_size):
        for k,v in time_ranges.items():
            x = v[0] + (v[1]-v[0])*random.random()
            env_states[k].append(x)
    return env_states

def generate_validation_specs(node,path,test_config):
    sampling_size = test_config["sampling_size"]
    mission_sizes = test_config["mission_sizes"]
    robot_counts = test_config["robot_counts"]
    room_limits = test_config["room_limits"]
    effort_limits = test_config["effort_limits"] 

    for ms in mission_sizes:
        for rc in robot_counts:
            node.get_logger().info(f"Working on Test : {rc}|{ms}")
            inv = generate_inventory(["mop","vacuum","polish"], rc)
            data = generate_mission_data(ms,room_limits)
            env_states = []
            for si in range(sampling_size):
                env_st = generate_env_state(ms, effort_limits)
                env_states.append(env_st)
            
            plans = generate_fragmented_plans(path, ms, data, inv)
            validation_path = f"{path}/tests/test_{rc}_{ms}"
            os.popen(f"mkdir -p {validation_path}")
            time.sleep(1)
            file_obj = [
                ("inventory.json",inv),
                ("data.json",data),
                ("env_states.json",env_states),
                ("tasks_baseline.json",plans["baseline"]),
                ("tasks_multi3.json",plans["multi3"]),
            ]
            for filename,obj in file_obj:
                with open(validation_path + "/" + filename,"w") as f:
                    json.dump(obj,f)

def generate_multi_mission_specs(node, path, test_config):
    mission_sizes = test_config["mission_sizes"]
    robot_counts = test_config["robot_counts"]
    tasks = ["vacuum", "mop", "polish"]
    node.get_logger().info(f"Starting in the multi-mission mode")

    with open(f"{path}/tasks/extra_room_template.json") as f:
        xm_template = f.read()
    
    for rc in robot_counts:
        
        inventory_test_folder = f"test_{rc}_2"
        with open(f"{path}/tests/{inventory_test_folder}/inventory.json") as f:
            inv = json.load(f)
        for ms in mission_sizes:
            node.get_logger().info(f"Working on Test : {rc}|{ms}")
            test_folder_name = f"test_{rc}_{ms}"
            interleaving_file_obj = []
            addition_file_obj = []
            # Create the mmfolders

            mx_validation_path = f"{path}/tests/Mtest_{rc}_{ms}"
            os.popen(f"mkdir -p {mx_validation_path}")
            time.sleep(1)

            m0_validation_path = f"{path}/tests/M0test_{rc}_{ms}"
            os.popen(f"mkdir -p {m0_validation_path}")
            time.sleep(1)

            # Interleaving Mode
            
            x_mission_str = xm_template.replace("$ROOM$", str(ms))
            x_mission_m3 = json.loads(x_mission_str)
            x_plans_m3 = generate_multi3_plans(x_mission_m3,{}) # No multi-instance tasks inside of the extra_room_template
            with open(f"{path}/tests/{test_folder_name}/tasks_multi3.json") as f:
                orig_plans_m3 = json.load(f)
            all_plans_m3 = orig_plans_m3 + x_plans_m3
            

            with open(f"{path}/tests/{test_folder_name}/env_states.json") as f:
                env_states_m3 = json.load(f)[0]
            with open(f"{path}/tests/{test_folder_name}/data.json") as f:
                orig_data = json.load(f)
            for t in tasks:
                env_states_m3[t].append(test_config["extra_mission_effort"])
            interleaving_file_obj.append(("tasks_multi3.json",all_plans_m3))
            interleaving_file_obj.append(("env_states.json",[env_states_m3]))
            interleaving_file_obj.append(("inventory.json",inv))

            for fname, obj in interleaving_file_obj:
                with open(mx_validation_path + "/" + fname,"w") as f:
                    json.dump(obj,f)
            
            # Baseline Mode ==> Done by addition, we run the extra mission withe the original inventory
            env_states_bl = {}
            for t in tasks:
                env_states_bl[t] = [test_config["extra_mission_effort"]]
            x_mission_str = xm_template.replace("$ROOM$", str(0))
            # node.get_logger().info(f"The xmission str is {x_mission_str}")
            x_mission_bl = json.loads(x_mission_str)
            x_plans_bl = generate_bl_plans(x_mission_bl,orig_data,inv)
            addition_file_obj.append(("env_states.json",[env_states_bl]))
            addition_file_obj.append(("inventory.json",inv))
            addition_file_obj.append(("tasks_baseline.json",x_plans_bl))

            for fname, obj in addition_file_obj:
                with open(m0_validation_path + "/" + fname,"w") as f:
                    json.dump(obj,f)


            


class TestGenerator(Node):
    def __init__(self, test_config):
        super().__init__("multi3_test_generator")
        self.test_config = test_config
        self.package_path = get_package_prefix("multi3_tests").replace("install","src") + "/multi3_tests"
        # Declare Parameters
        self.declare_parameter("mode", "standard")

        mode = self.get_parameter("mode").value
        self.multi_mission = mode == "multi-mission"
        
    def generate(self):
        if self.multi_mission:
            generate_multi_mission_specs(self,self.package_path,self.test_config)
        else:
            generate_validation_specs(self,self.package_path,self.test_config)

def main(args=None):
    base_path = get_package_prefix("multi3_tests").replace("install","src")
    with open(base_path + "/config/test_config.json") as f:
        test_config = json.load(f)
    
    rclpy.init(args=args)
    test_gen = TestGenerator(test_config)
    test_gen.generate()
    test_gen.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()