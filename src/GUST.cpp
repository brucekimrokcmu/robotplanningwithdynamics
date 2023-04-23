#include "GUST.hpp"
#include <limits>
#include <vector>
#include <queue>
#include <cmath>
#include <iostream>
#include <random>
#include <cassert>


#define minNrSteps 1
#define maxNrSteps 4
#define dt 1.0

//****************************************************
//***************** Helper Functions *****************
//****************************************************

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
        if(max_weight < weight){
            max_weight = weight;
            index = it->first;
        }
    }
    if(index == -1){
 
        return std::make_pair(Lambda.begin()->second, Lambda.begin()->first);
    }else{
        W.addSel(index);
        return std::make_pair(Lambda[index], index);
    }

};

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
StateSpace::VehicleState GUST::SampleTarget(int r) {
    // randomly sample a state from S in region r

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
        Region targetRegion = W.GetRegion(r);
        std::vector<int> neighbors = targetRegion.neighbors;
        int minH =  W.GetRegion(r).h_value;
        int idx = 0;
        std::vector<int> minHNeighbors;
        for(int i = 0; i < W.countRegionSize(); i++){
            if(!W.GetRegion(i).splitted && W.GetRegion(i).h_value < minH){
                minHNeighbors.push_back(i);
            }
        }
        if(minHNeighbors.size() > 0){
            idx = minHNeighbors[rand() % minHNeighbors.size()];
        }
        Region minHRegion = W.GetRegion(idx);
        if(rand() < 0.5){
            s_target.x_ = minHRegion.x_start + rand()/RAND_MAX * minHRegion.x_extent;
            s_target.y_ = minHRegion.y_start + rand()/RAND_MAX * minHRegion.y_extent;
        }else{
            double dist = PointDistance(W.GetRegion(r).x_start, W.GetRegion(r).y_start, targetRegion.x_start, targetRegion.y_start);
            double frac = rand()/RAND_MAX * dist;
            s_target.x_ = W.GetRegion(r).x_start + frac * (targetRegion.x_start - W.GetRegion(r).x_start);
            s_target.y_ = W.GetRegion(r).y_start + frac * (targetRegion.y_start - W.GetRegion(r).y_start);
        }
    }else{
        s_target = S.getRandomState();
    }

    return s_target;
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
    std::vector<MotionTree::Node> new_vertices;
    ControlSpace::VehicleControl u;


    bool usePID = ((double) rand() / (RAND_MAX)) < 0.8;
    // usePID = false; // ! Delete when integrating PID controller
    
    ;
    if (!usePID) {
        u = M.RandomController();
    }

    int numIter = rand()%(maxNrSteps-minNrSteps + 1) + minNrSteps;
    MotionTree::Node v_parent = v;

    for (int k=1; k<numIter; k++) {
        if (usePID) {
            u = M.PIDController(v.state, s_target);
        }

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

    return std::make_pair(MotionTree::Node(), new_vertices);

}

std::pair<int,std::vector<MotionTree::Node>> GUST::NewGroup(
    int r, MotionTree::Node v_new) {
    
    return make_pair(r, std::vector<MotionTree::Node>{T.getNode(v_new.id)});
}

MotionTree::Node GUST::GroupPlanner(
    std::vector<MotionTree::Node> Lambda_r, int r) {

    StateSpace::VehicleState s_target = SampleTarget(r);
    MotionTree::Node v = SelectVertex(Lambda_r, s_target);
    // s_target.v_ = 0.5;
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
                if(W.LocateRegion(v.state.x_, v.state.y_) == id){
                    v.region = id;
                    vertex_list.push_back(v);
                }
            }
            
            if(vertex_list.size() == 0){
                EmptyLambda.insert(
                    make_pair(
                        id, vertex_list
                    )
                );
                
            }else{
                Lambda.insert(
                    make_pair(
                        id, vertex_list
                    )
                );
            }
            
        }
        Lambda.erase(r);
        
    }
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

        if(v_last.id != -1){
            printf("finish\n");
            allNodes = T.nodes;
            return T.getPath(v_last);
        }
        if(W.GetRegion(groupId).whetherCanSplit()){

            SplitGroup(groupId);
        }
        // assert(Lambda.size() != 0);
        stop = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    } 
    allNodes = T.nodes;
    return std::vector<MotionTree::Node>{};  
}
