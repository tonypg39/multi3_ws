import json
import os

def compute_mission_times(lines, markers):
    # detect start time
    start_time = None
    end_time = None
    for l in lines:
        if l.find(markers["start_label"]) > -1:
            start_time = float(l.split()[2][1:-1])
        if l.find(markers["end_label"]) > -1:
            end_time = float(l.split()[2][1:-1])
    
    mission_times = {
        "start_time": start_time,
        "end_time": end_time,
        "completed_time": end_time - start_time
    }
    return mission_times


def compute_idle_times(lines, markers, mission_times, robot_count):
    idle_evolution = {}
    idle_times = {}
    for i in range(robot_count):
        robot_name = f"robot_{i+1}"
        idle_evolution[robot_name] = [(True,mission_times["start_time"])]
    
    for l in lines:
        if l.find(markers["status_label"]) > -1:
            sections = l.split("||")
            if sections[1] == " \n":
                continue
            for sec in sections[1:-1]:
                ws = sec.split()
                idle_evolution[ws[0]].append((ws[2] == "idle", float(l.split()[2][1:-1])))
    
    for i in range(robot_count):
        robot_name = f"robot_{i+1}"
        idle_evolution[robot_name].append((True,mission_times["end_time"]))
    
    for robot, idle_evol in idle_evolution.items():
        idle_time_total = 0.0
        for i in range(1,len(idle_evol)):
            if idle_evol[i-1][0] and idle_evol[i][0]:
                idle_time_total += (idle_evol[i][1] - idle_evol[i-1][1])
        idle_times[robot] = idle_time_total
    return idle_times



def main():
    markers = {
        "start_label": "$$**MISSION_START**$$",
        "end_label": "MISSION_COMPLETED",
        "status_label": "ROBOTS_STATE",
    }
    
    ws_path = "./"
    results_path = "./results/"


    global_results = {}
    for filename in os.listdir(results_path):
        file_path = os.path.join(results_path, filename)
        if os.path.isfile(file_path):  # Check if it's a file
            print(f"Processing File: {filename}")
            result_file = filename
            with open(results_path + result_file, "r") as f:
                log_lines = f.readlines()
            
            robot_count = int(result_file.split("_")[2])
            mission_times = compute_mission_times(log_lines,markers)
            print(f"The mission times are: {mission_times}")
            idle_times = compute_idle_times(log_lines,markers,mission_times,robot_count)
            print(f"The idle times are: {idle_times}")
            global_results[filename] = {
                "mission_times": mission_times,
                "idle_times": idle_times
            }
    with open("./metrics.json","w") as f:
        json.dump(global_results, f)
    


if __name__ == "__main__":
    main()