import random
import os
import json
import time
from collections import defaultdict
import rclpy
from rclpy.node import Node
from ament_index_python import get_package_prefix
from .gen_fragments import generate_fragmented_plans


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

def generate_mission_data(mission_size):
    room_limits = [-10,10]
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

def generate_env_state(mission_size):
    time_ranges = {
        "mop": [1,10],
        "vacuum": [1,10],
        "polish": [1,10]
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

def generate_validation_specs(node,path,sampling_size, mission_sizes, robot_counts):

    for ms in mission_sizes:
        for rc in robot_counts:
            node.get_logger().info(f"Working on Test : {ms}|{rc}")
            inv = generate_inventory(["mop","vacuum","polish"], rc)
            data = generate_mission_data(ms)
            env_states = []
            for si in range(sampling_size):
                env_st = generate_env_state(ms)
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
            
class TestGenerator(Node):
    def __init__(self, sampling_size, mission_sizes, robot_counts):
        super().__init__("multi3_test_generator")
        self.sampling_size = sampling_size
        self.mission_sizes = mission_sizes
        self.robot_counts = robot_counts
        self.package_path = get_package_prefix("multi3_tests").replace("install","src") + "/multi3_tests"
        
    def generate(self):
        generate_validation_specs(self,self.package_path,self.sampling_size, self.mission_sizes, self.robot_counts)



def main(args=None):
    sampling_size = 20
    mission_sizes = [2,10,15,20]
    robot_counts = [2,3,5,10]
    rclpy.init(args=args)
    test_gen = TestGenerator(sampling_size,mission_sizes,robot_counts)
    test_gen.generate()
    test_gen.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()