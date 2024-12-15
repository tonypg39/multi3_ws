import pandas as pd
import json

def process_metrics(metrics):
    data = {
        "test_id": [],
        "robot_count": [],
        "mission_size": [],
        "sample_id": [],
        "method": [],
        "running_mode": [],
        "completion_time": [],
        "average_idle_time": []
    } 

    for k,v in metrics.items():
        name = k[:k.find(".")]
        labels = name.split("_")
        test_id = k
        robot_count = int(labels[2])
        mission_size = int(labels[3])
        sample_id = int(labels[4])
        print("The detected method is : "+ labels[5])
        method = "multi3" if labels[5] == "m3" else "baseline"
        running_mode = "standard"
        if labels[1] == "Mtest":
            running_mode = "mm_interleaved"
        elif labels[1] == "M0test":
            running_mode = "extra_mission"
        
        completion_time = v["mission_times"]["completed_time"]
        total_idle = 0
        for ri in v["idle_times"].values():
            total_idle += ri
        avg_idle = total_idle/len(v["idle_times"])

        data["test_id"].append(test_id)
        data["robot_count"].append(robot_count)
        data["mission_size"].append(mission_size)
        data["sample_id"].append(sample_id)
        data["method"].append(method)
        data["running_mode"].append(running_mode)
        data["completion_time"].append(completion_time)
        data["average_idle_time"].append(avg_idle)
    
    df = pd.DataFrame(data)
    df.to_csv("data.csv",index=False)


if __name__ == "__main__":
    with open("metrics.json", "r") as f:
        metrics = json.load(f)
    process_metrics(metrics)
    
 