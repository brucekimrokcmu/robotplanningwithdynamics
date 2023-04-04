#include "WorkSpace.hpp" 

bool WorkSpace::Check_collision (double x, double y){
    if(map[y][x] == 0) return true;
    return false;
}

Region WorkSpace::GetRegion(int id_){
    return regions[id_];
}

// TODO: Need Implement -- Decomposition (split regions then append to `regions`)
void WorkSpace::Decompose(){
    return;
}

//TODO: Need Implement -- Calculate Heuristic value for each region
void WorkSpace::CalculateHeuristic(){
    return;
}