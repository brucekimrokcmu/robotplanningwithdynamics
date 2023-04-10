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
