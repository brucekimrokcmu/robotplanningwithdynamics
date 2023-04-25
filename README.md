# robotplanningwithdynamics

## Compilation

```shell
./compile.sh
```

After installing the controller, modify this compile file

## Test

`/test/testGUST.cpp `: a 20 x 20 map and 4 obstacles, random control space and motion function, can modify this file to test more cases

Need to run:

```shell
./GUSTOut.out Map20 output.txt
```

`/test/drawMap.ipynb`: Python jupyter notebook to visualize output file.

Grids represent regions,

Red triangles are obstacles.

Transparency represents the heuristic value: darker means far away from goal or have obstacles in the region

Blue dots are nodes in the path.

## Comparison

If you want to see the comparison between RRT and GUST, you can run:

```sh
python3 compare.py
```

It will save 10 graphs for each planner in `/fig` and their statistics in the json files in `/logs`. You can change the number of iteration in `compare.py`

This file will use multiprocessing, so it will take up all of processes of your laptop

## Configuration

You can change the start point, goal point, size of map in `/src/Constants.hpp`
