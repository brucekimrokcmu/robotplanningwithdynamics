#include "GUST.hpp"
#include <limits>
#include <vector>
#include <queue>
#include <cmath>
#include <iostream>
#include <random>
#include <cassert>
#include "WorkSpace.hpp"
#include "StateSpace.hpp"
#include "ControlSpace.hpp"
#include "MotionTree.hpp"

#define minNrSteps 1
#define maxNrSteps 2
#define dt 1.0

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
    // printf("Selecting group");
    int index = -1;
    double max_weight = 0.0;
    for(auto it = Lambda.begin(); it != Lambda.end(); it++){
        double h_norm = (W.h_max - W.GetRegion(it->first).h_value)/W.h_max;
        h_norm = delta + ( 1-delta)*h_norm;
        double weight = std::pow(h_norm, alpha) * std::pow(beta, W.GetRegion(it->first).nsel);
        // std::cout <<  std::pow(h_norm, alpha) << std::endl;
        // std::cout << std::pow(beta, W.GetRegion(it->first).nsel) << std::endl;
        // std::cout << "Weight: " << weight << std::endl;
        if(max_weight < weight){
            // printf("Smallers");
            max_weight = weight;
            index = it->first;
        }
    }
    if(index == -1){
        // printf("Error: No group selected");
        // std::cout << "Lambda size: " << Lambda.size() << std::endl;
        // std::cout << "Max weight: " << max_weight << std::endl;
        // std::cout << "Index: " << Lambda.begin()->second.size() << std::endl;
        // std::cout << max_weight << std::endl;   
        return std::make_pair(Lambda.begin()->second, Lambda.begin()->first);
    }else{
        W.addSel(index);
    // std::cout << "Selected region: " << index << std::endl;
        return std::make_pair(Lambda[index], index);
    }

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
        Region targetRegion = W.GetRegion(r);
        std::vector<int> neighbors = targetRegion.neighbors;
        int minH =  W.GetRegion(neighbors[0]).h_value;
        int idx = 0;
        for(int i = 0; i < neighbors.size(); i++){
            Region neighbor = W.GetRegion(neighbors[i]);
            if(neighbor.h_value < minH){
                idx = i;
                minH = neighbor.h_value;
            }
        }
        Region minHRegion = W.GetRegion(neighbors[idx]);
        s_target.x_ = minHRegion.x_start + rand()/RAND_MAX * minHRegion.x_extent;
        s_target.y_ = minHRegion.y_start + rand()/RAND_MAX * minHRegion.y_extent;

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
    // printf("newone\n");
    // Find vertex closest to s_target
    for (auto v : Lambda_r) {
        // printf("enter\n");
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
    std::vector<MotionTree::Node> new_vertices;
    ControlSpace::VehicleControl u;

    // // TODO: Implement random controller
    bool usePID = ((double) rand() / (RAND_MAX)) < smallProb;
    usePID = false; // ! Delete when integrating PID controller
    
    ;
    if (!usePID) {
        u = M.RandomController();
    }

    int numIter = rand()%(maxNrSteps-minNrSteps + 1) + minNrSteps;
    MotionTree::Node v_parent = v;
    // // TODO: Implement PID controller, Runge-Katta integration for motion,
    for (int k=1; k<numIter; k++) {
        if (usePID) {
            u = M.PIDController(v.state, s_target);
        }

    //     // TODO: implement motion function --> this should be passed in as an arg to the GUST constructor.
        StateSpace::VehicleState s_new = motion(v.state, u, dt);
        
        
        if (!valid(s_new)) {
            return std::make_pair(MotionTree::Node(), new_vertices);
        }
        
        MotionTree::Node v_new = T.newVertex(s_new);
        // std::cout << "s_new: " << s_new.x_ << ", " << s_new.y_ << std::endl;
        T.setParent(v_new.id, v_parent.id);
        T.setRegion(v_new.id, W.LocateRegion(s_new.x_, s_new.y_));
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
    // printf("return size: %d\n", (int)new_vertices.size());
    return std::make_pair(MotionTree::Node(), new_vertices);

    // ******************* FOR TEST (without PIDController) ****************
    // std::vector<MotionTree::Node> new_vertices;
    // double delta_x = s_target.x_ - v.state.x_;
    // double delta_y = s_target.y_ - v.state.y_;
    // double dist = PointDistance(s_target.x_, s_target.y_, v.state.x_, v.state.y_);
    // if(dist < epsilon){
    //     if(!valid(s_target)){
    //         return std::make_pair(MotionTree::Node(), new_vertices);
    //     }
    //     MotionTree::Node v_new = T.newVertex(s_target);
    //     // if(v_new.id == 1){
    //     //     std::cout << "child ID " << v_new.parent << std::endl;
    //     //     std::cout << "child selfID " << v_new.id << std::endl;
    //     // }
    //     T.setParent(v_new.id, v.id);
       
    //     T.addChild(v.id, v_new.id);
    //     new_vertices.push_back(T.getNode(v_new.id));
        
    //     if(goal(s_target)){
    //         return std::make_pair(T.getNode(v_new.id), new_vertices);
    //     }
    //     return std::make_pair(MotionTree::Node(), new_vertices);
    // }
    // double delta_x_norm = delta_x * epsilon / dist;
    // double delta_y_norm = delta_y * epsilon / dist;
    // MotionTree::Node v_parent = v;
    // while (delta_x_norm <= delta_x && delta_y_norm <= delta_y)
    // {   
    //     StateSpace::VehicleState s_new ;
    //     s_new.x_ = v.state.x_ + delta_x_norm;
    //     s_new.y_ = v.state.y_ + delta_y_norm;
    //     s_new.theta_ = v.state.theta_;
    //     s_new.v_ = v.state.v_;
    //     s_new.phi_= v.state.phi_;
        
    //     if(!valid(s_new)){
    //         return std::make_pair(MotionTree::Node(), new_vertices);
    //     }
    //     MotionTree::Node v_new = T.newVertex(s_new);
    //     // printf("ID: %d\n", v_new.id);
    //     // assert(v_parent.id != 0);
        
    //     T.setParent(v_new.id, v_parent.id);
    //     T.setRegion(v_new.id, W.LocateRegion(s_new.x_, s_new.y_));
        
    //     T.addChild(v_parent.id, v_new.id);
    //     // Need control but now just leave it blank for test
    //     new_vertices.push_back(T.getNode(v_new.id));
    //     v_parent = T.getNode(v_new.id);
    //     delta_x_norm += epsilon * delta_x / dist;
    //     delta_y_norm += epsilon * delta_y / dist;
    //     if(goal(s_new)){
    //         return std::make_pair(T.getNode(v_new.id), new_vertices);
    //     }

    // }
    
    // return make_pair(MotionTree::Node(), new_vertices);



}

std::pair<int,std::vector<MotionTree::Node>> GUST::NewGroup(
    int r, MotionTree::Node v_new) {
    
    return make_pair(r, std::vector<MotionTree::Node>{T.getNode(v_new.id)});
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
                    r_new, std::vector<MotionTree::Node>{v_new}
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
                    r_new_new, std::vector<MotionTree::Node>{v_new}
                )
            );
        }
        else{
            // If region of the v_new is not in the set of regions
            Lambda.find(r_new)->second.push_back(v_new);
        }
    }
    // std::cout << "GPLambda Size: " << Lambda.size() << std::endl;
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
            int id = W.addRegion(region);
            
            std::vector<MotionTree::Node> vertex_list;
            for(auto &v : Lambda.find(r)->second){
                // printf("Have something\n");
                if(W.LocateRegion(v.state.x_, v.state.y_) == id){
                    // printf("Something in \n");
                    v.region = id;
                    vertex_list.push_back(v);
                }
            }
            
            if(vertex_list.size() == 0){
                // printf("Nothing \n");
                EmptyLambda.insert(
                    make_pair(
                        id, vertex_list
                    )
                );
                
            }else{
                // printf("Something should be inserted \n");
                Lambda.insert(
                    make_pair(
                        id, vertex_list
                    )
                );
            }
            
        }
        Lambda.erase(r);
        
    }
    // std::cout << "Lambda Size: " << Lambda.size() << std::endl;
}

//******************************************************************
//************************* GUST Function **************************
//******************************************************************

std::vector<MotionTree::Node> GUST::RunGUST(std::vector<MotionTree::Node> &allNodes){
    W.Decompose();
    W.CalculateHeuristic(goal_region.x_start + goal_region.x_extent/2, goal_region.y_start + goal_region.y_extent/2);

    InitTreeAndGroups();
    // printf("Finish Init\n");
    auto start = std::chrono::high_resolution_clock::now();  
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    while(duration.count() < time){ // TODO: change to max time
        // printf("Enter Loop\n");
        int groupId = SelectGroup().second;
        // std::cout << "groupId: " << groupId << std::endl;
        std::vector<MotionTree::Node> Lambda_r = Lambda.find(groupId)->second;
        if(Lambda_r.size() == 0){
            printf("wrong\n");
        }
        MotionTree::Node v_last = GroupPlanner(Lambda_r, groupId);
        // std::cout << Lambda.size() << std::endl;
        // std::cout << Lambda.begin()->second.size() << std::endl;
        // printf("Out GroupPlanner\n");
        if(v_last.id != -1){
            printf("finish\n");
            allNodes = T.nodes;
            return T.getPath(v_last);
        }
        if(W.GetRegion(groupId).whetherCanSplit()){
            // printf("Split\n");
            SplitGroup(groupId);
        }
        // assert(Lambda.size() != 0);
        stop = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        // printf("Finish one loop: %f\n", (double)duration.count());
    } 
    allNodes = T.nodes;
    return std::vector<MotionTree::Node>{};  
}
