#include "RRT.hpp"
#include <limits>
#include <vector>
#include <queue>
#include <cmath>
#include <iostream>
#include <random>
#include <cassert>




//****************************************************
//***************** Helper Functions *****************
//****************************************************
void RRT::InitTree(){
    T = MotionTree();
    T.nodes[0].state = s_init;
}
/*
* Euclidean distance between two points
*/
inline double PointDistance(
    double x1, double y1, double x2, double y2) {
    double x_diff = pow(x1 - x2, 2);
    double y_diff = pow(y1 - y2, 2);
    return sqrt(x_diff + y_diff);
}

/*
* Samples a target uniformly at random from a region R
*
* TODO: actually sample the target. for now
* this just chooses the border of the region.
*/
StateSpace::VehicleState RRT::SampleTarget() {
    // randomly sample a state from S in region r
    if(rand() < 0.5){
        StateSpace::VehicleState s_target = S.getRandomState();
        s_target.x_ = goal_region.x_start;
        s_target.y_ = goal_region.y_start;
        return s_target;
    }
    StateSpace::VehicleState s_target = S.getRandomState();
    return s_target;
    
}


/*
* Get vertex in Lambda_r that is closest to s_target
*/
MotionTree::Node RRT::SelectVertex(StateSpace::VehicleState s_target) {
    
    double min_dist = std::numeric_limits<double>::max();
    int idx;
    for(auto node : T.nodes) {
        if(PointDistance(node.state.x_, node.state.y_, s_target.x_, s_target.y_) < min_dist) {
            min_dist = PointDistance(node.state.x_, node.state.y_, s_target.x_, s_target.y_);
            idx = node.id;
        }
    }

    return T.getNode(idx);

}

/*
* Use a controller to make a motion to move from v
* towards s_target by moving to a new intermediate 
* vertex v_new, and expand the MotionTree to
* include v_new.
*
* Returns the last node added to the tree as well as a list
* of all of the new vertices added.
*/
std::pair<MotionTree::Node, std::vector<MotionTree::Node>> RRT::ExpandTree(
    MotionTree::Node v, StateSpace::VehicleState s_target) {
    std::vector<MotionTree::Node> new_vertices;
    ControlSpace::VehicleControl u;


    bool usePID = ((double) rand() / (RAND_MAX)) < constants::usePID;
    // usePID = false; // ! Delete when integrating PID controller
    
    ;
    if (!usePID) {
        u = M.RandomController();
    }

    int numIter = rand()%(constants::maxNrSteps-constants::minNrSteps + 1) + constants::minNrSteps;
    MotionTree::Node v_parent = v;

    for (int k=1; k<numIter; k++) {
        if (usePID) {
            u = M.PIDController(v.state, s_target);
        }

        StateSpace::VehicleState s_new = motion(v.state, u, constants::dt);
        
        
        if (!valid(s_new)) {
            return std::make_pair(MotionTree::Node(), new_vertices);
        }
        
        MotionTree::Node v_new = T.newVertex(s_new);
        // std::cout << "s_new: " << s_new.x_ << ", " << s_new.y_ << std::endl;
        T.setParent(v_new.id, v_parent.id);
        T.addChild(v_parent.id, v_new.id);
        T.setControl(v_new.id, u);
        v_parent = T.getNode(v_new.id);
        new_vertices.push_back(T.getNode(v_new.id));

        if (goal(s_new)) {
            return std::make_pair(T.getNode(v_new.id), new_vertices);
        }
        if (S.nearTarget(s_new, s_target)) {
            
             return std::make_pair(MotionTree::Node(), new_vertices);
        }

        v = v_new;
    }

    return std::make_pair(MotionTree::Node(), new_vertices);

}


MotionTree::Node RRT::GroupPlanner() {

    StateSpace::VehicleState s_target = SampleTarget();
    MotionTree::Node v = SelectVertex( s_target);
    
    // s_target.v_ = 0.5;
    std::pair<MotionTree::Node, std::vector<MotionTree::Node>> new_v = ExpandTree(v, s_target);

    MotionTree::Node v_last = new_v.first;

    
    return v_last;
}



//******************************************************************
//************************* GUST Function **************************
//******************************************************************

std::vector<MotionTree::Node> RRT::RunRRT(std::vector<MotionTree::Node> &allNodes){

    InitTree();
    // printf("Finish Init\n");
    auto start = std::chrono::high_resolution_clock::now();  
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    while(duration.count() < time){ // TODO: change to max time
      
        MotionTree::Node v_last = GroupPlanner();

        if(v_last.id != -1){
            printf("finish\n");
            allNodes = T.nodes;
            return T.getPath(v_last);
        }

        // assert(Lambda.size() != 0);
        stop = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    } 
    allNodes = T.nodes;
    return std::vector<MotionTree::Node>{};  
}
