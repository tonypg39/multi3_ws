import random
import os
import json
import time
from collections import defaultdict
import rclpy
from rclpy.node import Node
from ament_index_python import get_package_prefix
from .baseline_utils import sample_static_assignments
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

def get_mission_skills(task):
    skills = []
    for t in task["subtasks"]:
        if t["type"] == "concrete":
            skills.append(t["task_id"])
        else:
            skills += get_mission_skills(t)
    return skills

def get_mi_tasks(task):
    mi_list = []
    for t in task["subtasks"]:
        # print(f"task_id = {t['task_id']} ==> type ({t['type']})")
        if t["type"] == "multi-instance":
            mi_list.append(t)
        elif t["type"] == "abstract":
            mi_list += get_mi_tasks(t)
    return mi_list

def generate_mission_data(tasks, mission_size, area_limits):
    d = {
        "instances": {}
    }
    mi_tasks = get_mi_tasks(tasks)
    # ASSUME: That in the tree structure, the mi-tasks' children are all concrete task
    for t in mi_tasks:
        for st in t["subtasks"]:
            d["instances"][st["task_id"]] = []

    for i in range(mission_size):
        s_x = area_limits[0] + (area_limits[1]-area_limits[0])*random.random()
        s_y = area_limits[0] + (area_limits[1]-area_limits[0])*random.random()
        for task in d["instances"].keys():
            d["instances"][task].append({"location":{"x":s_x,"y":s_y}})
    return d 


def generate_env_state(tasks, mission_size, effort_limits):
    all_tasks = get_mission_skills(tasks)
    mi_tasks = get_mi_tasks(tasks)
    mi_children = []
    time_ranges = {}
    env_states = {}

    for t in mi_tasks:
        for st in t["subtasks"]:
            mi_children.append(st["task_id"])
            time_ranges[st["task_id"]] = effort_limits
            env_states[st["task_id"]] = []

    for t in all_tasks:
        if t not in mi_children:
            x = effort_limits[0] + (effort_limits[1]-effort_limits[0])*random.random()
            env_states[t] = x

    for i in range(mission_size):
        for k,v in time_ranges.items():
            x = v[0] + (v[1]-v[0])*random.random()
            env_states[k].append(x)
    return env_states


# def get_scenario_skills(scenario):
def load_scenario(path, scenario, mission_size):
    with open(f"{path}/tasks/{scenario}.json","r") as f:
        scenario_txt = f.read()
        # Find the $MISSION_SIZE marker and replace it with the actual mission size parameter
        scenario_txt = scenario_txt.replace('"$MISSION_SIZE"',str(mission_size))
        tasks = json.loads(scenario_txt)
    return tasks


def generate_validation_specs(node,path,test_config):
    sampling_size = test_config["sampling_size"]
    mission_sizes = test_config["mission_sizes"]
    robot_counts = test_config["robot_counts"]
    area_limits = test_config["area_limits"]
    effort_limits = test_config["effort_limits"] 
    scenarios = test_config["scenarios"]
    bl_assign_sample_size = test_config["bl_assignment_sample_size"]

    for ms in mission_sizes:
        for rc in robot_counts:
            for scenario in scenarios:
                node.get_logger().info(f"Working on Test : {rc}|{ms}|{scenario}")
                mission = load_scenario(path, scenario, ms)
                skills = get_mission_skills(mission)
                inv = generate_inventory(skills, rc) 
                data = generate_mission_data(mission,ms,area_limits)

                # FIXME: Test this integration into the whole pipeline
                static_assignments = sample_static_assignments(mission,inv,bl_assign_sample_size)                
                # print("Static Assignments:=> ",static_assignments)
                env_states = []
                for _ in range(sampling_size):
                    env_st = generate_env_state(mission, ms, effort_limits)
                    env_states.append(env_st)
                

                plans = generate_fragmented_plans(mission, data, static_assignments)
                validation_path = f"{path}/tests/test_{rc}_{ms}_{scenario}"
                os.popen(f"mkdir -p {validation_path}")
                time.sleep(1)
                file_obj = [
                    ("inventory.json",inv),
                    ("data.json",data),
                    ("env_states.json",env_states),
                    # ("tasks_baseline.json",plans["baseline"]),
                    ("tasks_multi3.json",plans["multi3"]),
                ]
                for i, plan_bl in enumerate(plans["baseline"]):
                    file_obj.append((f"tasks_bl_{i}.json", plan_bl))

                for filename,obj in file_obj:
                    with open(validation_path + "/" + filename,"w") as f:
                        json.dump(obj,f)


# FIXME: Multimission Generalization to the Multi-Scenario validation
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