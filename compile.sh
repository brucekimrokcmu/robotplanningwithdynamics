g++ -g src/WorkSpace.cpp src/GUST.cpp src/MotionTree.cpp src/ControlSpace.cpp test/testGUST.cpp -o GUSTOut.out
# clang++ -std=c++11 -stdlib=libc++ -g -o GUSTOut.out src/WorkSpace.cpp src/GUST.cpp src/MotionTree.cpp src/ControlSpace.cpp test/testGUST.cpp