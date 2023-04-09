#include "../src/GUST.hpp"
#include <iostream>

#define PI_D 3.14159265358979323846

int main(){
    Obstacle o1 = Obstacle(3,3,0.5,0.5);
    WorkSpace W2 = WorkSpace(0,20,0,20);
    Obstacle o2 = Obstacle(10,10,0.5,0.5);
    W2.addObstacle(o1);
    W2.addObstacle(o2);

    StateSpace S = StateSpace(20,20,2*PI_D, 1, PI_D/2);
    ControlSpace C = ControlSpace(1, 1);

     std::function<StateSpace::VehicleState(StateSpace::VehicleState, ControlSpace::VehicleControl, double)>
         motion = [&](StateSpace::VehicleState s, ControlSpace::VehicleControl c, double dt){
        StateSpace::VehicleState s_new;
        // TODO: Implement this function
        return s_new;
    };
    StateSpace::VehicleState s_init = StateSpace::VehicleState(0,0,0,0,0);

    Region goal_region = Region(18,18,0.5,0.5); 
    std::function<bool(StateSpace::VehicleState)> goal = [&](StateSpace::VehicleState s){
        if(goal_region.inRegion(s.x_, s.y_)){
            return true;
        }
        return false;
    };

    std::function<bool(StateSpace::VehicleState)> valid = [&](StateSpace::VehicleState s){
       if(W2.Check_collision(s.x_, s.y_)){
           return false;
       }
       return true;
    };

    GUST gust = GUST( S, W2, C, motion, valid, s_init, goal, goal_region);
    std::vector<MotionTree::Node> result = gust.RunGUST();

    for(auto n : result){
        std::cout << n.state.x_ << " " << n.state.y_  << std::endl;
    }

}