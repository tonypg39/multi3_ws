import json
import numpy as np
# Global variables
building_sequences = {}
bseq_bl = {}
fragments = []
data = None
# robot_inventory = {}
active_bl_assignment = {}


# def preassign_bl(ctask):
#     # Iterate over the robots to see who can perform this skill
#     candidates = []
    # sep = ctask.find("^")
    # if sep >-1:
    #     ctask = ctask[:sep]
#     for robot,skills in robot_inventory.items():
#         if ctask in skills:
#             candidates.append(robot)
#     if len(candidates) > 0:
#         # print(candidates, ctask)
#         rnd_idx = np.random.choice(len(candidates))
#         return candidates[rnd_idx]
#     return None

def preassign_bl(ctask):
    # sep = ctask.find("^")
    # if sep >-1:
    #     ctask = ctask[:sep]
    # print(f"Active assignemnt: {active_bl_assignment} | ctask: {ctask}")
    return active_bl_assignment[ctask]


def flatten(A):
    Af = []
    for li in A:
        Af += li
    return Af

def disconnect(nodes):
    for node in nodes:
        if node["sequence"] != "NIL":
            send_task_node = create_send_task(node["task_id"])
            if building_sequences[node["sequence"]][-1] != send_task_node: # If no node disconnected this sequence
                building_sequences[node["sequence"]].append(send_task_node)

def disconnect_bl(node):
    if node["sequence"] != "NIL":
        send_task_node = create_send_task(node["task_id"])
        if bseq_bl[node["sequence"]][-1] != send_task_node: # If no node disconnected this sequence
            bseq_bl[node["sequence"]].append(send_task_node)

def create_wait_task(id):
    return {
        "id": f"wait_until|{id}",
        "vars": {}
    }

def create_send_task(id):
    return {
        "id": f"send_signal|{id}",
        "vars": {}
    }


def generate_fragments(t, prev, mi_idx):

    global data,building_sequences, fragments
    if t["type"] == "concrete":
        itask = {"id": t["task_id"], "vars": t["variables"].copy()}
        if mi_idx > -1:
            core_id = t["task_id"][:t["task_id"].find("^")]
            itask["vars"].update(data["instances"][core_id][mi_idx])
        
        if len(prev) == 1 and t["sequence"] != "NIL" and prev[0]["sequence"] == t["sequence"]:
            building_sequences[t["sequence"]].append(itask)
            return [t]
        

        # Generate a new fragment or start/restart a sequence
        li = [p["task_id"] for p in prev]
        initial_wait = "&".join([p["task_id"] for p in prev])
        disconnect(prev)

        if t["sequence"] != "NIL":
            if not t["sequence"] in building_sequences:
                building_sequences[t["sequence"]] = []
            building_sequences[t["sequence"]] += [create_wait_task(initial_wait), itask]   
        else:
            frag = {
            "initial_wait": initial_wait,
            "tasks": [itask, create_send_task(t["task_id"])]
            }
            fragments.append(frag)
        return [t]

    if t["type"] == "abstract":
        pt = prev
        if t["method"] == "sequential":
            for st in t["subtasks"]:
                pt = generate_fragments(st, pt,mi_idx)
            return [pt]
        elif t["method"] == "parallel":
            all_prevs = []
            for st in t["subtasks"]:
                x = generate_fragments(st, pt,mi_idx)
                all_prevs.append(x)
            return flatten(all_prevs)
    
    if t["type"] == "multi-instance":
        all_prevs = []
        pt = prev.copy()
        for idx in range(t["variables"]["loopCount"]):
            if t["variables"]["loopType"] == "parallel":
                pt = prev.copy()
            for st in t["subtasks"]:
                st_i = st.copy()
                st_i["task_id"] = st_i["task_id"] + f"^{idx}"
                if st_i["sequence"] != "NIL":
                    st_i["sequence"] = st_i["sequence"] + f"^{idx}"
                pt = generate_fragments(st_i,pt,idx)
            if t["variables"]["loopType"] == "parallel":
                all_prevs.append(pt)

        return flatten(all_prevs) if t["variables"]["loopType"] == "parallel" else [pt]
               
def gen_seq_bl(t, prev, mi_idx):
    global data,bseq_bl
    if t["type"] == "concrete":
        itask = {"id": t["task_id"], "vars": t["variables"].copy()}
        if mi_idx > -1:
            core_id = t["task_id"][:t["task_id"].find("^")]
            itask["vars"].update(data["instances"][core_id][mi_idx])
        
        robot_seq = preassign_bl(t["task_id"])
        if robot_seq is None:
            print(f'Fatal!! No robot w/ skill to perform {t["task_id"]}')
            return t
        t["sequence"] = robot_seq
        if  prev["sequence"] == robot_seq:
            bseq_bl[robot_seq].append(itask)
            return t

        # Generate a new fragment or start/restart a sequence
        initial_wait = prev["task_id"]
        disconnect_bl(prev)

        if not t["sequence"] in bseq_bl:
            bseq_bl[t["sequence"]] = []
        bseq_bl[t["sequence"]] += [create_wait_task(initial_wait), itask]   
        return t
    
    pt = prev.copy()
    if t["type"] == "abstract":
        for st in t["subtasks"]:
            pt = gen_seq_bl(st, pt,-1)
        return pt
    
    if t["type"] == "multi-instance":
        for idx in range(t["variables"]["loopCount"]):
            for st in t["subtasks"]:
                st_i = st.copy()
                st_i["task_id"] = st_i["task_id"] + f"^{idx}"
                if st_i["sequence"] != "NIL":
                    st_i["sequence"] = st_i["sequence"] + f"^{idx}"
                pt = gen_seq_bl(st_i,pt,idx)

        return pt
    
def sequence2fragments(sequences, preassigned=False):
    frags = []
    for s_id,s in sequences.items():
        x = s[0]["id"].find("|")
        wait = s[0]["id"][x+1:]
        obj = {}
        if preassigned:
            obj["robot"] = s_id
        obj["initial_wait"] = wait
        obj["tasks"] = s[1:]
        frags.append(obj)
    return frags

def reset():
    global data, building_sequences, bseq_bl, fragments
    building_sequences = {}
    bseq_bl = {}
    fragments = []
    


def generate_fragmented_plans(mission, pdata, bl_assignments):
    # Read always simple_rooms.json
    global data, building_sequences, bseq_bl, fragments, active_bl_assignment
    tasks = mission
    data = pdata
    reset()
    prev = {
        "task_id": "SYSTEM_START",
        "type": "concrete",
        "roles": "STARTER",
        "sequence": "NIL",
        "variables": {}
    }
    ##  Multi3 generation
    generate_fragments(tasks,[prev.copy()],-1)
    D_m3 = fragments + sequence2fragments(building_sequences)

    ## Baseline 
    # gen_seq_bl(tasks,prev.copy(),-1)
    # D_bl = sequence2fragments(bseq_bl,preassigned=True)
    baseline_plans = []
    # Return a list of baseline fragment plans (One for every assignment)
    for assign in bl_assignments:
        reset()
        prev = {
            "task_id": "SYSTEM_START",
            "type": "concrete",
            "roles": "STARTER",
            "sequence": "NIL",
            "variables": {}
        }
        active_bl_assignment = assign
        gen_seq_bl(tasks,prev.copy(),-1)
        D_bl = sequence2fragments(bseq_bl,preassigned=True)
        baseline_plans.append(D_bl)

    # Call both generators and  create tasks_baseline.json and tasks_multi3.json and put them inside the test folder
    return {
        "multi3": D_m3,
        "baseline": baseline_plans
    }


def generate_bl_plans(mission, pdata, inventory):
    global bseq_bl, fragments, robot_inventory, data
    tasks = mission

    data = pdata
    robot_inventory = inventory
    bseq_bl = {}
    fragments = []
    prev = {
        "task_id": "SYSTEM_START",
        "type": "concrete",
        "roles": "STARTER",
        "sequence": "NIL",
        "variables": {}
    }
    gen_seq_bl(tasks, prev.copy(), -1)
    plans_bl = sequence2fragments(bseq_bl, preassigned=True)
    return plans_bl

def generate_multi3_plans(mission, pdata):
    global building_sequences, fragments, robot_inventory, data 
    tasks = mission
    data = pdata
    building_sequences = {}
    fragments = []
    prev = {
        "task_id": "SYSTEM_START",
        "type": "concrete",
        "roles": "STARTER",
        "sequence": "NIL",
        "variables": {}
    }
    generate_fragments(tasks, [prev.copy()],-1)
    plans_multi3 = fragments + sequence2fragments(building_sequences)
    return plans_multi3

if __name__ == "__main__":
    file_name = "simple_rooms"
    with open(f"tasks/{file_name}.json","r") as f:
        tasks = json.load(f)
    
    with open("data.json","r") as f:
        data = json.load(f)


    robot_inventory = {
        "robot1": ["mop","clean_mop"],
        "robot2": ["vacuum"],
        "robot3": ["vacuum"]
    }
    prev = {
        "task_id": "SYSTEM_START",
        "type": "concrete",
        "roles": "STARTER",
        "sequence": "NIL",
        "variables": {}
    }

    ##  Multi3 generation
    generate_fragments(tasks,[prev],-1)
    print("Generating the Multi3... Fragments")
    D = fragments + sequence2fragments(building_sequences)
    with open(f"tasks_output/{file_name}_output_multi3.json","w") as f:
        json.dump(D,f)

    ## Baseline 
    gen_seq_bl(tasks,prev,-1)
    # print(building_sequences)
    print("Generating the Baseline... Fragments")
    D = sequence2fragments(bseq_bl,preassigned=True)
    with open(f"tasks_output/{file_name}_output_bl.json","w") as f:
        json.dump(D,f)