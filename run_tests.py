import json
import os
import time
import psutil
from datetime import datetime
import argparse

def kill_ros2_nodes():
    print("Killing active node(s) if any...")
    # executors
    cmd = "ps aux | grep executor"
    r = os.popen(cmd).read()
    lines = r.splitlines()

    # executors
    cmd = "ps aux | grep coordinator"
    r = os.popen(cmd).read()
    lines += r.splitlines()
    for l in lines:
        if l.find("grep") > -1:
            continue
        # print("Original line: "+ l)
        cmd = f"kill -2 {l.split()[1]}"
        # print(f"Killed node in :{cmd}")
        os.popen(cmd)
    


def check_finished(filename):
    with open(filename, "r") as f:
        lines = f.read()
    return lines.find("$$*MISSION_STOPPED*$$") > -1


def wait_and_kill(log_file):
    while True:
        time.sleep(1)
        if check_finished(log_file):
            time.sleep(3)
            break
    kill_ros2_nodes()


def main():
    ws_path = "./"
    output_path = "./results"
    with open(ws_path + "src/multi3_tests/config/test_config.json", "r") as f:
        test_config = json.load(f)

    for rc in test_config["robot_counts"]:
        for ms in test_config["mission_sizes"]:
            for si in range(test_config["sampling_size"]):
                for scenario in test_config["scenarios"]:
                    # FIXME: Add the scenario loop
                    test_id = f"test_{rc}_{ms}_{scenario}"

                    print(f"Processing test:{test_id} | Sample {si}")
                    timestamp = datetime.now().strftime("%Y%m%d%H%M")
                    output_filename = f"{output_path}/{timestamp}_{test_id}_{si}_m3.log"
                    cmd = f"ros2 launch multi3_tests test.launch.py test_id:={test_id} sample_id:=s{si} > {output_filename} 2>&1" 
                    print(f"Running [multi3] => {cmd} ...")
                    os.popen(cmd)
                    wait_and_kill(output_filename)
                    
                    # FIXME: Add here the baseline id as the mode
                    # output_filename = f"{output_path}/{timestamp}_{test_id}_{si}_bl.log"
                    # cmd = f"ros2 launch multi3_tests test.launch.py test_id:={test_id} sample_id:=s{si} mode:=baseline > {output_filename} 2>&1" 
                    # print(f"Running [baseline] => {cmd} ...")
                    # os.popen(cmd)

                    for bl_i in range(test_config["bl_assignment_sample_size"]):
                        output_filename = f"{output_path}/{timestamp}_{test_id}_{si}_bl_{bl_i}.log"
                        cmd = f"ros2 launch multi3_tests test.launch.py test_id:={test_id} sample_id:=s{si} mode:=bl_{bl_i} > {output_filename} 2>&1" 
                        print(f"Running [baseline {bl_i}/{test_config['bl_assignment_sample_size']}] => {cmd}")
                        os.popen(cmd)
                        wait_and_kill(output_filename)



def multi_main():
    ws_path = "./"
    output_path = "./results"
    with open(ws_path + "src/multi3_tests/config/test_config.json", "r") as f:
        test_config = json.load(f)
    
    # Fixed mission size at 10
    ms = 10
    si = 0

    for rc in test_config["robot_counts"]:
        test_id = f"test_{rc}_{ms}"
        print(f"Processing test:{test_id} | Sample {si}")
        timestamp = datetime.now().strftime("%Y%m%d%H%M")
        output_filename = f"{output_path}/{timestamp}_M{test_id}_{si}_m3.log"
        cmd = f"ros2 launch multi3_tests test.launch.py test_id:=M{test_id} sample_id:=s{si} > {output_filename} 2>&1" 
        print(f"Running [multi3] => {cmd} ...")
        os.popen(cmd)
        wait_and_kill(output_filename)
        output_filename = f"{output_path}/{timestamp}_M0{test_id}_{si}_bl.log"
        cmd = f"ros2 launch multi3_tests test.launch.py test_id:=M0{test_id} sample_id:=s{si} mode:=baseline > {output_filename} 2>&1" 
        print(f"Running [baseline] => {cmd} ...")
        os.popen(cmd)
        wait_and_kill(output_filename)
                

if __name__ == "__main__":
    # kill_ros2_nodes()
    cmdline = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    cmdline.add_argument('--mode', default="generate")
    # cmdline.add_argument('--id', default="")
    flags, unk_args = cmdline.parse_known_args()
    f_mode = flags.mode

    # Add another mode to run the multi-mission 
    if f_mode == "generate":
        main()
    elif f_mode == "multi-mission":
        multi_main()
    else:
        kill_ros2_nodes()