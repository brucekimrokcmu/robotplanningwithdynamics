#include "WorkSpace.hpp"
#include <iostream> 

bool WorkSpace::Check_collision (double x, double y){
    for(auto r : obstacles){
        if (r.isObstacle(x,y)){
            return true;
        }
    }
    return false;
}

Region WorkSpace::GetRegion(int id_){
    return regions[id_];
}


std::vector<Region> WorkSpace::SplitRegion(Region r){
    Region up_left;
    Region up_right;
    Region bottom_left;
    Region bottom_right;

    // Up-left region
    up_left.x_start = r.x_start;
    up_left.y_start = r.y_start + r.y_extent/2.0;
    up_left.x_extent = r.x_extent/2.0;
    up_left.y_extent = r.y_extent/2.0;
    // Up-right region
    up_right.x_start = r.x_start + r.x_extent/2.0;
    up_right.y_start = r.y_start + r.y_extent/2.0;
    up_right.x_extent = r.x_extent/2.0;
    up_right.y_extent = r.y_extent/2.0;
    // bottom-left region
    bottom_left.x_start = r.x_start;
    bottom_left.y_start = r.y_start;
    bottom_left.x_extent = r.x_extent/2.0;
    bottom_left.y_extent = r.y_extent/2.0;
    // Up-right region
    bottom_right.x_start = r.x_start + r.x_extent/2.0;
    bottom_right.y_start = r.y_start;
    bottom_right.x_extent = r.x_extent/2.0;
    bottom_right.y_extent = r.y_extent/2.0;

    std::vector<Region> result;
    result.push_back(up_left);
    result.push_back(up_right);
    result.push_back(bottom_left);
    result.push_back(bottom_right);
   
    return result;

}

/**
 * @return 
 * -1: can't find region contain position (x,y)
 * i : the index number of region which contains (x,y)
 * 
*/
int WorkSpace::LocateRegion(double x, double y){
    for(int i = 0; i < regions.size(); i++ ){
        Region r = regions[i];
        if(!r.splitted && r.x_start + r.x_extent > x && r.x_start <= x && r.y_start + r.y_extent > y && r.y_start <= y ){
            return i;
        }
    }
    return -1;

}

// TODO: Need Implement -- Decomposition (split regions then append to `regions`)

bool WorkSpace::containObstacle(Region r){
    for(auto obs : obstacles){
        if(
        obs.isObstacle(r.x_start, r.y_start)||
        obs.isObstacle(r.x_start + r.x_extent, r.y_start)||
        obs.isObstacle(r.x_start, r.y_start + r.y_extent)||
        obs.isObstacle(r.x_start + r.x_extent, r.y_start + r.y_extent)||
        r.inRegion(obs.x-obs.x_extent, obs.y-obs.y_extent)||
        r.inRegion(obs.x+obs.x_extent, obs.y-obs.y_extent)||
        r.inRegion(obs.x-obs.x_extent, obs.y+obs.y_extent)||
        r.inRegion(obs.x+obs.x_extent, obs.y+obs.y_extent)){
            return true;
        }

    }
    return false;
}

void WorkSpace::decomposeHelper(Region r){
    if(r.x_extent < SMALLESTEXTENT || r.y_extent < SMALLESTEXTENT){
        return;
    }
    if(!containObstacle(r)){
        return;
    }
    
    std::vector<Region> new_regions = SplitRegion(r);
    
    r.splitted = true;
    for(auto nr : new_regions){
        regions.push_back(nr);
        decomposeHelper(nr);
    }
    return;

}

void WorkSpace::makeGraph(){
    for(size_t i = 0; i < regions.size(); i++){
        Region r = regions[i];
        if(!r.splitted){
            for(size_t j = 0; j < regions.size(); j++){
                if(i!=j && !regions[j].splitted){
                    Region r_n = regions[j];
                    //right neighbor
                    if(r_n.x_start == r.x_start+r.x_extent && r_n.y_start == r.y_start){
                        r.neighbors.push_back(j);
                    }

                    // right top
                    if(r_n.x_start == r.x_start+r.x_extent && r_n.y_start == r.y_start+r.y_extent){
                        r.neighbors.push_back(j);
                    }

                    // right bottom
                    if(r_n.x_start == r.x_start+r.x_extent && r_n.y_start + r_n.y_extent == r.y_start){
                        r.neighbors.push_back(j);
                    }

                    // bottom
                    if(r_n.x_start == r.x_start && r_n.y_start + r_n.y_extent == r.y_start){
                        r.neighbors.push_back(j);
                    }

                    // top
                    if(r_n.x_start == r.x_start && r_n.y_start == r.y_start + r.y_extent){
                        r.neighbors.push_back(j);
                    }

                    // top left
                    if(r_n.x_start + r_n.x_extent == r.x_start && r_n.y_start == r.y_start + r.y_extent){
                        r.neighbors.push_back(j);
                    }

                    // left
                    if(r_n.x_start + r_n.x_extent == r.x_start && r_n.y_start == r.y_start){
                        r.neighbors.push_back(j);
                    }

                    // bottom left
                    if(r_n.x_start + r_n.x_extent == r.x_start && r_n.y_start + r_n.y_extent == r.y_start){
                        r.neighbors.push_back(j);
                    }
                }
            }
        }
    }
}

void WorkSpace::Decompose(){
    // Start Region 
    Region start = Region(x_min,y_min,x_max-x_min, y_max-y_min);
    regions.push_back(start);
    decomposeHelper(start);
    makeGraph();
    return;
}

//TODO: Need Implement -- Calculate Heuristic value for each region
void WorkSpace::CalculateHeuristic(){
    return;
}