import time
import json
import multiprocessing
import statistics
import subprocess
from pathlib import Path
from visualize import visualize
from visualize import visualize_comparison
from tqdm import tqdm

algo = "GUST"
iter = 50

def worker(i: int):
    sTime = time.time()
    args = ["./GUSTOut.out", "Map20", f"output-{i}.txt"]
    if algo == "RRT": args.append("-rrt")
    
    output = subprocess.run(args, capture_output=True)
    stdout = output.stdout.decode("utf-8")
    duration = 0
    for line in stdout.split("\n"):
        if line.endswith("milliseconds"):
            duration = int(line.split()[-2])
            # visualize(i)
        if line.endswith("nodes"):
            nodes = int(line.split()[-2])
            eTime = time.time()
            return (duration,nodes, eTime - sTime)

if __name__ == "__main__":
    subprocess.run(["./compile.sh"], shell=True)

    with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
        result = pool.starmap(worker, [(i,) for i in range(iter)])
        duration_list = [x[0] for x in result]
        nodes_list = [x[1] for x in result]
        GUST_times_list = [x[2] for x in result]

    data = {}
    data["mean"] = statistics.mean(duration_list)
    data["median"] = statistics.median(duration_list)
    data["std"] = statistics.stdev(duration_list)
    data["min"] = min(duration_list)
    data["max"] = max(duration_list)
    data["#nodes"] = statistics.median(nodes_list)

    Path("logs").mkdir(parents=True, exist_ok=True)
    with open(f"logs/duration_{algo}.json", "w") as f:
        json.dump(data, f, indent=4)

    algo = "RRT"
    with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
        result = pool.starmap(worker, [(i,) for i in range(iter)])
        duration_list = [x[0] for x in result]
        nodes_list = [x[1] for x in result]
        RRT_times_list = [x[2] for x in result]

    data = {}
    data["mean"] = statistics.mean(duration_list)
    data["median"] = statistics.median(duration_list)
    data["std"] = statistics.stdev(duration_list)
    data["min"] = min(duration_list)
    data["max"] = max(duration_list)
    data["#nodes"] = statistics.median(nodes_list)

    visualize_comparison(GUST_times_list, RRT_times_list)


    Path("logs").mkdir(parents=True, exist_ok=True)
    with open(f"logs/duration_{algo}.json", "w") as f:
        json.dump(data, f, indent=4)

