import random
import math
import json

def sample_combinations(data, N):
    lens = [len(group) for group in data]
    max_total = 10000
    total = min(math.prod(lens),max_total)

    if N > total:
        raise ValueError(f"Cannot sample {N} unique combinations: only {total} possible.")
    # Sample N unique indices
    indices = random.sample(range(total), N)

    # Convert index to combination using mixed-radix conversion
    result = []
    for idx in indices:
        combo = []
        for l in reversed(lens):
            idx, r = divmod(idx, l)
            combo.append(r)
        combo = list(reversed(combo))
        selected = tuple(data[i][j] for i, j in enumerate(combo))
        result.append(selected)

    return result


def sample_static_assignments(mission, inventory, sample_size):
    concrete_tasks = get_concrete(mission, -1)
    data = []
    for t in concrete_tasks:
        candidates = []
        ctask = t
        sep = ctask.find("^")
        if sep >-1:
            ctask = ctask[:sep]

        for robot,skills in inventory.items():
            r_num = int(robot.split("_")[1])
            if ctask in skills:
                candidates.append(robot)
        data.append(candidates)
    
    sample_assignments = sample_combinations(data, sample_size)
    # print(sample_assignments)
    assigments = []
    for i in range(sample_size):
        D = {}
        for j,ct in enumerate(concrete_tasks):
            D[ct] = sample_assignments[i][j]
        assigments.append(D)
    return assigments

def  get_concrete(task, mi_idx):
    if task["type"] == "concrete":
        mi_tag = "" if mi_idx == -1 else f"^{mi_idx}"
        return [task["task_id"] + mi_tag]
    concrete = []
    if task["type"] == "multi-instance":
        for k in range(task["variables"]["loopCount"]):
            for st in task["subtasks"]:
                concrete += get_concrete(st, k)
    else:
        for st in task["subtasks"]:
                concrete += get_concrete(st, mi_idx)
    return concrete
                

if __name__ == "__main__":
    path = "/home/tony/Documents/gssi/RESEARCH/seams25_extension/multi3_ws/src/multi3_tests/multi3_tests"
    scenario = "t_agriculture"
    with open(f"{path}/tasks/{scenario}.json","r") as f:
        scenario_txt = f.read()
        # Find the $MISSION_SIZE marker and replace it with the actual mission size parameter
        scenario_txt = scenario_txt.replace('"$MISSION_SIZE"',str(3))
        mission = json.loads(scenario_txt)

    print(get_concrete(mission,-1))