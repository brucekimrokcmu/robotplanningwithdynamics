#include "GUST.hpp"
#include <limits>
#include <vector>
#include <queue>
#include <cmath>
#include <iostream>
#include <random>
#include "WorkSpace.hpp"
#include "StateSpace.hpp"
#include "ControlSpace.hpp"
#include "MotionTree.hpp"

#define minNrSteps 500
#define maxNrSteps 1500

/**
* Initialize the tree and the groups
*/
void GUST::InitTreeAndGroups(){
    T = MotionTree();
    T.nodes[0].state = s_init;
    double x = Proj(s_init).first;
    double y = Proj(s_init).second;
    T.nodes[0].region = W.LocateRegion(x,y);
    Lambda[T.nodes[0].region].push_back(T.nodes[0]);
}
std::pair<std::vector<MotionTree::Node>, int> GUST::SelectGroup(){
    // Select a group Lambda_r
    int index = 0;
    double max_weight = 0.0;
    for(auto it = Lambda.begin(); it != Lambda.end(); it++){
        double h_norm = (W.h_max - W.GetRegion(it->first).h_value)/W.h_max;
        h_norm = delta + ( 1-delta)*h_norm;
        double weight = std::pow(h_norm, alpha) * std::pow(beta, W.GetRegion(it->first).nsel);
        if(max_weight < weight){
            max_weight = weight;
            index = it->first;
        }
    }
    W.addSel(index);
    return std::make_pair(Lambda[index], index);
};

/*
* Samples a target uniformly at random from a region R
*
* TODO: actually sample the target. for now
* this just chooses the border of the region.
*/
StateSpace::VehicleState GUST::SampleTarget(int r) {
    // randomly sample a state from S in region r

    // TODO: actually sample this, for now just return a fixed state in the region

    Region lambda_R = W.GetRegion(r);
    StateSpace::VehicleState s_target;
    double lower_bound = 0;
    double upper_bound = 1;
 
    std::uniform_real_distribution<double> unif(lower_bound,
                                           upper_bound);
    std::default_random_engine re;
 
    // Getting a random double value
    double random_double = unif(re);
    if(random_double < smallProb){
        s_target = S.getRandomState();
        // TODO: sample a state in the regions along the shortest path to goal
    }else{
        s_target = S.getRandomState();
    }
    
    // return fixed state in region
    

    return s_target;
}

/*
* Euclidean distance between two points
*/
double PointDistance(
    double x1, double y1, double x2, double y2) {
    
    double x_diff = pow(x1 - x2, 2);
    double y_diff = pow(y1 - y2, 2);

    return sqrt(x_diff + y_diff);

}

std::pair<double,double> GUST::Proj(StateSpace::VehicleState s) {
    return std::make_pair(s.x_, s.y_);
}

/*
* Get vertex in Lambda_r that is closest to s_target
*/
MotionTree::Node GUST::SelectVertex(
    std::vector<MotionTree::Node> Lambda_r, StateSpace::VehicleState s_target) {
    
    std::pair<double,double> p_target = GUST::Proj(s_target);

    MotionTree::Node v_closest;
    double minDist = std::numeric_limits<double>::max();

    // Find vertex closest to s_target
    for (auto v : Lambda_r) {

        std::pair<double,double> p_curr = GUST::Proj(v.state);

        double dist = PointDistance(
            p_target.first, p_target.second,
            p_curr.first, p_curr.second);
        
        if (dist < minDist) {
            v_closest = v;
            minDist = dist;
        }
    }

    return v_closest;

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
std::pair<MotionTree::Node, std::vector<MotionTree::Node>> GUST::ExpandTree(
    MotionTree::Node v, StateSpace::VehicleState s_target) {
        //*****************Original (should be changed back) ****************
    // std::vector<MotionTree::Node> new_vertices;
    // VehicleControl u;

    // // TODO: Implement random controller
    // bool usePID = ((double) rand() / (RAND_MAX)) < 0.5;
    // if (!usePID) {
    //     u = RandomController();
    // }

    // int numIter = rand()%(maxNrSteps-minNrSteps + 1) + minNrSteps;

    // // TODO: Implement PID controller, Runge-Katta integration for motion,
    // for (int k=1; k<numIter; k++) {
    //     if (usePID) {
    //         u = PIDController(v.state, s_target);
    //     }

    //     // TODO: implement motion function --> this should be passed in as an arg to the GUST constructor.
    //     VehicleState s_new = motion(v.state, u, dt);

    //     if (!StateSpace::validState(s_new)) {
    //         return NULL;
    //     }
        
    //     MotionTree::Node v_new = T.newVertex(s_new);
    //     v_new.parent = v;
    //     v_new.state = s_new;
    //     v_new.control = u;

    //     new_vertices.push_back(v_new);

    //     if (StateSpace::goalState(s_new)) {
    //         return std::pair(v_new, new_vertices);
    //     }
    //     if (StateSpace::nearTarget(s_new, s_target)) {
    //         return NULL;
    //     }

    //     v = v_new;
    // }
    // return NULL;
    // ******************* FOR TEST (without PIDController) ****************
    std::vector<MotionTree::Node> new_vertices;
    double delta_x = s_target.x_ - v.state.x_;
    double delta_y = s_target.y_ - v.state.y_;
    double dist = PointDistance(s_target.x_, s_target.y_, v.state.x_, v.state.y_);
    if(dist < epsilon){
        if(!valid(s_target)){
            return std::make_pair(MotionTree::Node(), new_vertices);
        }
        MotionTree::Node v_new = T.newVertex(s_target);
        v_new.parent = v.id;
        new_vertices.push_back(v_new);
        if(goal(s_target)){
            return std::make_pair(v_new, new_vertices);
        }
        return std::make_pair(MotionTree::Node(), new_vertices);
    }
    double delta_x_norm = delta_x * epsilon / dist;
    double delta_y_norm = delta_y * epsilon / dist;
    MotionTree::Node v_parent = v;
    while (delta_x_norm <= delta_x && delta_y_norm <= delta_y)
    {   
        StateSpace::VehicleState s_new ;
        s_new.x_ = v.state.x_ + delta_x_norm;
        s_new.y_ = v.state.y_ + delta_y_norm;
        s_new.theta_ = v.state.theta_;
        s_new.v_ = v.state.v_;
        s_new.phi_= v.state.phi_;
        
        if(!valid(s_new)){
            return std::make_pair(MotionTree::Node(), new_vertices);
        }
        MotionTree::Node v_new = T.newVertex(s_new);
        v_new.parent = v_parent.id;
        v_new.region = W.LocateRegion(s_new.x_, s_new.y_);
        v_new.state = s_new;
        v_new.id = T.nodes.size();
        v.children.push_back(v_new.id);
        // Need control but now just leave it blank for test
        new_vertices.push_back(v_new);
        v_parent = v_new;
        delta_x_norm += epsilon * delta_x / dist;
        delta_y_norm += epsilon * delta_y / dist;
        if(goal(s_new)){
            return std::make_pair(v_new, new_vertices);
        }
    }
    
    return make_pair(MotionTree::Node(), new_vertices);



}

std::pair<int,std::vector<MotionTree::Node>> NewGroup(
    int r, MotionTree::Node v_new) {
    
    return make_pair(r, std::vector<MotionTree::Node>{v_new});
}

MotionTree::Node GUST::GroupPlanner(
    std::vector<MotionTree::Node> Lambda_r, int r) {

    StateSpace::VehicleState s_target = SampleTarget(r);
    MotionTree::Node v = SelectVertex(Lambda_r, s_target);
    std::pair<MotionTree::Node, std::vector<MotionTree::Node>> new_v = ExpandTree(v, s_target);

    MotionTree::Node v_last = new_v.first;
    std::vector<MotionTree::Node> new_vertices = new_v.second;

    for (MotionTree::Node v_new : new_vertices) {
        std::pair<double,double> v_new_point = Proj(v_new.state);
        int r_new = W.LocateRegion(v_new_point.first, v_new_point.second);

        std::unordered_map<int,std::vector<MotionTree::Node>>::const_iterator iter = EmptyLambda.find(r_new);
        // If region of the v_new is in the set of empty regions
        if (iter != EmptyLambda.end()) {
            // Remove r_new from empty splits
            std::vector<MotionTree::Node> vertex_list = iter->second;
            EmptyLambda.erase(r_new);

            // Add r_new back to Lambda
            Lambda.insert(
                make_pair(
                    r_new, vertex_list
                )
            );
        }
        else if(Lambda.find(r_new) == Lambda.end()){
            // If region of the v_new is in the set of non-empty regions
            std::pair<int,std::vector<MotionTree::Node>> new_group = NewGroup(r_new, v_new);
            int r_new_new = new_group.first;
            std::vector<MotionTree::Node> vertex_list = new_group.second;

            // Add r_new_new to Lambda
            Lambda.insert(
                make_pair(
                    r_new_new, vertex_list
                )
            );
        }
        else{
            // If region of the v_new is not in the set of regions
            Lambda.find(r_new)->second.push_back(v_new);
        }
    }
    return v_last;

}

/*
* SplitGroup
*/
void GUST::SplitGroup(int r){
    std::vector<Region> new_regions = W.SplitRegion(W.GetRegion(r));
    if(new_regions.size() == 0){
        return;
    }
    else{
        W.setSplitted(r);
        for (auto &region : new_regions)
        {
            W.addRegion(region);
            std::vector<MotionTree::Node> vertex_list;
            for(auto &v : Lambda.find(r)->second){
                if(W.LocateRegion(v.state.x_, v.state.y_) == region.id){
                    v.region = region.id;
                    vertex_list.push_back(v);
                }
            }
            if(vertex_list.size() == 0){
                EmptyLambda.insert(
                    make_pair(
                        region.id, vertex_list
                    )
                );
                
            }else{
                Lambda.insert(
                    make_pair(
                        region.id, vertex_list
                    )
                );
            }
            
        }
        
    }
}

//******************************************************************
//************************* GUST Function **************************
//******************************************************************

std::vector<MotionTree::Node> GUST::RunGUST(std::vector<MotionTree::Node> &allNodes){
    W.Decompose();
    W.CalculateHeuristic(goal_region.x_start + goal_region.x_extent/2, goal_region.y_start + goal_region.y_extent/2);

    InitTreeAndGroups();
    printf("Finish Init\n");
    auto start = std::chrono::high_resolution_clock::now();  
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    while(true){ // TODO: change to max time
        printf("Enter Loop\n");
        int groupId = SelectGroup().second;
        std::vector<MotionTree::Node> Lambda_r = Lambda.find(groupId)->second;
        
        MotionTree::Node v_last = GroupPlanner(Lambda_r, groupId);
        printf("Out GroupPlanner\n");
        if(v_last.id != -1){
            printf("finish\n");
            allNodes = T.nodes;
            return T.getPath(v_last);
        }
        if(W.GetRegion(groupId).whetherCanSplit()){
            printf("Split\n");
            SplitGroup(groupId);
        }
        stop = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        printf("Finish one loop: %f\n", (double)duration.count());
    } 
    return std::vector<MotionTree::Node>{};  
}