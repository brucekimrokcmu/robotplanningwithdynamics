#pragma once
#include <vector>
#include <limits>

#define SMALLESTEXTENT 1

struct Region {
    Region(){};
    Region(double x_, double y_, double x_extent,double y_extent)
        : x_start(x_), y_start(y_), x_extent(x_extent), y_extent(y_extent), splitted(false),
            h_value(std::numeric_limits<double>::max()), expanded(false) {}

    double x_start;
    double y_start;
    double x_extent;
    double y_extent;
    bool splitted;

    bool inRegion(double x, double y){
        return x >= x_start && x < x_start+x_extent && y >= y_start && y < y_start + y_extent;
    }



    double h_value;
    bool expanded;
};

class WorkSpace{
private:
    struct Obstacle {
        double x;
        double y;
        double x_extent;
        double y_extent;

        bool isObstacle(double x_, double y_){
            return x_ >= x-x_extent && x_ <= x+x_extent && y_ >= y-y_extent && y_ <= y+y_extent; 
        }
        // bool isObstacle(Region r){
            
        //     isObstacle(r.x_start, r.y_start);
        //     isObstacle(r.x_start + r.x_extent, r.y_start);
        //     isObstacle(r.x_start, r.y_start+r.y_extent);
        //     isObstacle(r.x_start + r.x_extent, r.y_start + r.y_extent);


            
        // }

        Obstacle(double x, double y, double x_extent, double y_extent): x(x), y(y), x_extent(x_extent), y_extent(y_extent){};
    };
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    std::vector<Obstacle> obstacles;
    std::vector<Region> regions;
public:
    WorkSpace(){};
   
    WorkSpace(double x_min, double x_max, double y_min, double y_max, std::vector<Obstacle> obs)
        : x_min(x_min), y_min(y_min), x_max(x_max), y_max(y_max), obstacles(obs){};
    bool Check_collision(double x, double y);
    Region GetRegion(int id);

    // Helper functions
    bool containObstacle(Region r);
    void decomposeHelper(Region r);

    void Decompose();
    void CalculateHeuristic();
    int LocateRegion(double x, double y);
    std::vector<Region> SplitRegion(Region r);
};