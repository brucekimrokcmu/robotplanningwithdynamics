#include "workspace.h" 

bool WorkSpace::check_collision (double x, double y){
    if(map[y][x] == 0) return true;
    return false;
}

Region WorkSpace::getRegion(int id_){
    return regions[id_];
}

// TODO: Need Implement -- Decomposition (split regions then append to `regions`)
void WorkSpace::decompose(){
    return;
}

//TODO: Need Implement -- Calculate Heuristic value for each region
void WorkSpace::calculateHeuristic(){
    return;
}