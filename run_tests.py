import json
import os
import time
import psutil
from datetime import datetime

def kill_ros2_nodes():
    # executors
    cmd = "ps aux | grep executor"
    r = os.popen(cmd).read()
    lines = r.splitlines()

    # executors
    cmd = "ps aux | grep coordinator"
    r = os.popen(cmd).read()
    lines += r.splitlines()

    for l in lines:
        # print("Original line: "+ l)
        cmd = f"kill -2 {l.split()[1]}"
        print(f"Running the kill command :{cmd}")
        os.popen(cmd)
    


def check_finished(filename):
    with open(filename, "r") as f:
        lines = f.read()
    return lines.find("$$*MISSION_STOPPED*$$") > -1

def main():
    ws_path = "./"
    output_path = "./results/"
    with open(ws_path + "src/multi3_tests/config/test_config.json", "r") as f:
        test_config = json.load(f)

    for rc in test_config["robot_counts"]:
        for ms in test_config["mission_sizes"]:
            for si in range(test_config["sample_size"]):
                test_id = f"test_{rc}_{ms}"
                print(f"Processing test:{test_id} | Sample {si}")
                timestamp = datetime.now().strftime("%Y%m%d%H%M")
                output_filename = f"{output_path}/{timestamp}_{test_id}_{si}.log"
                cmd = f"ros2 launch multi3_tests test.launch.py test_id:={test_id} > {output_filename} 2>&1" 
                os.popen(cmd)
                
                while True:
                    time.sleep(1)
                    if check_finished(output_filename):
                        time.sleep(3)
                        break
                kill_ros2_nodes()


if __name__ == "__main__":
    # kill_ros2_nodes()
    main()