#pragma once
#include <unordered_map>
#include <functional>
#include <vector>
#include <chrono> 

#include "WorkSpace.hpp"
#include "StateSpace.hpp"
#include "ControlSpace.hpp"
#include "MotionTree.hpp"

class GUST{
public: 
    GUST(StateSpace& S, WorkSpace& W, const ControlSpace& M,  
        std::function<StateSpace::VehicleState(StateSpace::VehicleState, ControlSpace::VehicleControl, double)> motion, 
        std::function<bool(StateSpace::VehicleState)> valid, const StateSpace::VehicleState& s_init, 
        std::function<bool(StateSpace::VehicleState)> goal, Region goal_region)
            : S(S), W(W), M(M), motion(motion), valid(valid), s_init(s_init),goal(goal), goal_region(goal_region){}
    
    std::vector<MotionTree::Node> RunGUST();  // Main function to Run GUST
private:
    // Input Parameters
    StateSpace& S; 
    WorkSpace& W; 
    const ControlSpace& M;  
    std::function<StateSpace::VehicleState(StateSpace::VehicleState, ControlSpace::VehicleControl, double)> motion; 
    std::function<bool(StateSpace::VehicleState)> valid;
    const StateSpace::VehicleState& s_init;
    std::function<bool(StateSpace::VehicleState)> goal;
    Region goal_region;

    // Constants
    double time = 10000; // TODO: max time to sampling
    double delta = 0.5; 
    double alpha = 8;
    double beta = 0.85;
    double smallProb = 0.15;
    double epsilon = 0.8;

    // Motion Tree
    MotionTree T;
    // Set of set of vertices in small regions
    std::unordered_map<int,std::vector<MotionTree::Node>> Lambda;
    std::unordered_map<int,std::vector<MotionTree::Node>> EmptyLambda;

    // Function Interfaces
    void InitTreeAndGroups();
    std::pair<std::vector<MotionTree::Node>, int> SelectGroup();
    MotionTree::Node GroupPlanner(std::vector<MotionTree::Node> Lambda_r, int r);
    void SplitGroup(int r);

    // Help Functions for `GroupPlanner`
    StateSpace::VehicleState SampleTarget(int r);
    MotionTree::Node SelectVertex(std::vector<MotionTree::Node> Lambda_r, StateSpace::VehicleState s_target);
    std::pair<MotionTree::Node, std::vector<MotionTree::Node>> ExpandTree(MotionTree::Node v, StateSpace::VehicleState s_target);
    
    std::pair<double, double> Proj(StateSpace::VehicleState s);
    // int LocateRegion(double x, double y);


};