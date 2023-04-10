#include <iostream> 
#include <queue>

#include "WorkSpace.hpp"

/**
 *  Set `splitted` to true
 * @param id : region id
*/
void WorkSpace::setSplitted(int id){
    regions[id].splitted = true;
}

/**
 * Add a new region to `regions`, and set id as `regions.size()`.
 * @param r : region
*/
void WorkSpace::addRegion(Region r){
    r.id = (int)regions.size();
    regions.push_back(r);
}

/**
* @param x,y: position
* @return 
* true: collision
* false: no collision
*/
bool WorkSpace::Check_collision (double x, double y){
    for(auto r : obstacles){
        if (r.isObstacle(x,y)){
            return true;
        }
    }
    return false;
}

/**
* @param id_ : region id
* @return Region
*/
Region WorkSpace::GetRegion(int id_) const{
    return regions[id_];
}

/**
 * add 1 to the number of `nsel` in region id_
* @param id_ : region id
*/
void WorkSpace::addSel(int id_){
    return regions[id_].addSel();
}

/**
 *  Each region is split into 4 regions (subdivision)
* Each new region inharits the `h_value` and `nsel` from the parent region
* @param id_ : region id
* @return new regions after split
*/
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
    if(up_left.x_extent != 0.0 && up_left.y_extent != 0.0)
        up_left.h_value = r.h_value;
        up_left.nsel = r.nsel;
        up_left.splitted = false;
        result.push_back(up_left);
    if(up_right.x_extent != 0.0 && up_right.y_extent != 0.0)
        up_right.h_value = r.h_value;
        up_right.nsel = r.nsel;
        up_right.splitted = false;
        result.push_back(up_right);
    if(bottom_left.x_extent != 0.0 && bottom_left.y_extent != 0.0)
        bottom_left.h_value = r.h_value;
        bottom_left.nsel = r.nsel;
        bottom_left.splitted = false;
        result.push_back(bottom_left);
    if(bottom_right.x_extent != 0.0 && bottom_right.y_extent != 0.0)
        bottom_right.h_value = r.h_value;
        bottom_right.nsel = r.nsel;
        bottom_right.splitted = false;
        result.push_back(bottom_right);
   
    return result;

}

/**
 * @param x,y: position
 * @return 
 * -1: can't find region contain position (x,y)
 * i : the index number of region which contains (x,y)
 * 
*/
int WorkSpace::LocateRegion(double x, double y) const{
    for(int i = 0; i < regions.size(); i++ ){
        Region r = regions[i];
        if(!r.splitted && r.x_start + r.x_extent > x && r.x_start <= x && r.y_start + r.y_extent > y && r.y_start <= y ){
            return i;
        }
    }
    return -1;

}

/**
 * @param r : region
 * @return true: region r contains obstacle
 *        false: region r does not contain obstacle 
*/
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

/**
 * Helper function for `Decopose()`, doing recursive split.
 * @param r : region
*/
void WorkSpace::decomposeHelper(Region &r){
    if(r.x_extent < SMALLESTEXTENT || r.y_extent < SMALLESTEXTENT){
        return;
    }
    if(!containObstacle(r)){
        return;
    }
    
    std::vector<Region> new_regions = SplitRegion(r);
    
    regions[r.id].splitted = true;
    for(auto nr : new_regions){
        nr.id = (int)regions.size();
        regions.push_back(nr);
        decomposeHelper(nr);
    }
    return;

}

/**
 * adding neighbors to each region
*/
void WorkSpace::makeGraph(){
    for(size_t i = 0; i < regions.size(); i++){
        Region r = regions[i];
        if(!r.splitted){
            for(size_t j = 0; j < regions.size(); j++){
                if(i!=j && !regions[j].splitted){
                    Region r_n = regions[j];
                    //right neighbor
                    if(r_n.x_start == r.x_start+r.x_extent && r_n.y_start < r.y_start + r.y_extent && r_n.y_start + r_n.y_extent >= r.y_start){
                        regions[i].neighbors.push_back(j);
                        continue;
                    }

                    // bottom
                    if(r_n.x_start <= r.x_start+r.x_extent && r_n.y_start + r_n.y_extent == r.y_start && r_n.x_start + r_n.x_extent >= r.x_start){
                        regions[i].neighbors.push_back(j);
                        continue;
                    }

                    // top
                    if(r_n.x_start <= r.x_start+r.x_extent && r_n.y_start == r.y_start + r.y_extent && r_n.x_start + r_n.x_extent >= r.x_start){
                        regions[i].neighbors.push_back(j);
                        continue;
                    }

                    // left
                    if(r_n.x_start + r_n.x_extent == r.x_start && r_n.y_start +r_n.y_extent >= r.y_start && r_n.y_start <= r.y_start + r.y_extent){
                        regions[i].neighbors.push_back(j);
                        continue;
                    }
                }
            }
        }
    }
}

/**
 * Decomposition at the very beginning of GUST
 * ! Called in `GUST::RunGUST()`
*/
void WorkSpace::Decompose(){
    // Start Region 
    Region start = Region(x_min,y_min,x_max-x_min, y_max-y_min);
    start.id = 0;
    regions.push_back(start);
    decomposeHelper(start);
    makeGraph();
    return;
}


struct Comparator{
    bool operator()(Region& a, Region& b){
        return a.h_value > b.h_value;
    }
};

/**
 * Calculate heuristic value for each region
 * @param x,y : goal position
 * ! Called in `GUST::RunGUST()` 
*/
void WorkSpace::CalculateHeuristic(double x, double y){
    Region start;
    for(size_t i = 0; i < regions.size(); i++){
        if(regions[i].inRegion(x,y)){
            printf("Here");
        }
        if(!regions[i].splitted && regions[i].inRegion(x,y)){
            regions[i].h_value = 0.0;
            // printf("%f\n", regions[i].x_start);
            start = regions[i];
        }
    }
    // start.h_value = 0.0;

  
    std::priority_queue <Region, std::vector<Region>, Comparator> openPQ;
    openPQ.push(start);
    while(openPQ.size() != 0){
        
        Region current = openPQ.top();
        openPQ.pop();

        regions[current.id].expanded = true;

        for(size_t i = 0; i < current.neighbors.size(); i++){
            
            Region neighbor = regions[current.neighbors[i]];
            // printf("%f,%f, %f |",neighbor.x_start, neighbor.y_start, current.h_value);
            // printf("%i, %i, %i\n", neighbor.expanded, containObstacle(neighbor), neighbor.h_value > current.h_value + 1.0);
            if(!neighbor.expanded&&!containObstacle(neighbor) && neighbor.h_value > current.h_value + 1.0){
                
                regions[current.neighbors[i]].h_value = current.h_value + 1.0;
                if(h_max < regions[current.neighbors[i]].h_value)
                    h_max = regions[current.neighbors[i]].h_value;
                openPQ.push(regions[current.neighbors[i]]);
            }

        }

    }


    return;
}   
