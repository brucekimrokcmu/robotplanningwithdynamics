#pragma once
#include <vector>
#include <limits>
#include <set>
#include "MotionTree.h"

struct Region {
    Region(double x_, double y_, double size)
        : x_start(x_), y_start(y_), size(size), 
            h_value(std::numeric_limits<double>::max()), expanded(false) {}

    double x_start;
    double y_start;
    double size;
    double h_value;
    bool expanded;
    std::set<MotionTree::Node> vertices;
};

class WorkSpace{
private:
    std::vector<std::vector<double>> map;
    std::vector<Region> regions;
public:
    WorkSpace(){};
    // TODO: Need a Parameterized Constructor
    bool check_collision(double x, double y);
    Region getRegion(int id);

    
    void decompose();
    void calculateHeuristic();

};