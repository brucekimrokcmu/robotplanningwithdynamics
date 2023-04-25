import json
import multiprocessing
import statistics
import subprocess
from pathlib import Path
from visualize import visualize

algo = "GUST"
iter = 10

def worker(i: int):
    args = ["./GUSTOut.out", "Map20", f"output-{i}.txt"]
    if algo == "RRT": args.append("-rrt")
    
    output = subprocess.run(args, capture_output=True)
    stdout = output.stdout.decode("utf-8")
    for line in stdout.split("\n"):
        if line.endswith("milliseconds"):
            duration = int(line.split()[-2])
            visualize(i)
            return duration

if __name__ == "__main__":
    subprocess.run(["./compile.sh"], shell=True)

    with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
        duration_list = pool.starmap(worker, [(i,) for i in range(iter)])
    algo = "RRT"
    with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
        duration_list = pool.starmap(worker, [(i,) for i in range(iter)])

    data = {}
    data["mean"] = statistics.mean(duration_list)
    data["median"] = statistics.median(duration_list)
    data["std"] = statistics.stdev(duration_list)
    data["min"] = min(duration_list)
    data["max"] = max(duration_list)

    Path("logs").mkdir(parents=True, exist_ok=True)
    with open(f"logs/duration_{algo}.json", "w") as f:
        json.dump(data, f, indent=4)
