#pragma once
#include <unordered_map>
#include <functional>
#include <vector>
#include <chrono> 

#include "WorkSpace.hpp"
#include "StateSpace.hpp"
#include "ControlSpace.hpp"
#include "MotionTree.hpp"
#include "Constants.hpp"

class RRT{
public: 
    RRT(StateSpace& S, WorkSpace& W, ControlSpace& M,  
        std::function<StateSpace::VehicleState(StateSpace::VehicleState, ControlSpace::VehicleControl, double)> motion, 
        std::function<bool(StateSpace::VehicleState)> valid, const StateSpace::VehicleState& s_init, 
        std::function<bool(StateSpace::VehicleState)> goal, Region goal_region)
            : S(S), W(W), M(M), motion(motion), valid(valid), s_init(s_init),goal(goal), goal_region(goal_region){}
    
    std::vector<MotionTree::Node> RunRRT(std::vector<MotionTree::Node> &allNodes);  // Main function to Run GUST
private:
    // Input Parameters
    StateSpace& S; 
    WorkSpace& W; 
    ControlSpace& M;  
    std::function<StateSpace::VehicleState(StateSpace::VehicleState, ControlSpace::VehicleControl, double)> motion; 
    std::function<bool(StateSpace::VehicleState)> valid;
    const StateSpace::VehicleState& s_init;
    std::function<bool(StateSpace::VehicleState)> goal;
    Region goal_region;

    // Constants
    double time = constants::MAXTIME; 
    double delta = constants::DELTA; 
    double alpha = constants::ALPHA;
    double beta = constants::BETA;
    double smallProb = constants::targetNearGoal;
    double epsilon = constants::EPSILON;

    // Motion Tree
    MotionTree T;

    void InitTree();
    MotionTree::Node GroupPlanner();


    // Help Functions for `GroupPlanner`
    /**
     * Sample a target state in a region
     * @param r : region ID
    */
    StateSpace::VehicleState SampleTarget();
    /**
     * Select a vertex from Lambda_r
     * @param Lambda_r : vector of Nodes
     * @param s_target : target state
     * @return Node to expand
    */
    MotionTree::Node SelectVertex( StateSpace::VehicleState s_target);
    /**
     * Expand the tree from a vertex
     * @param v : Node to expand
     * @param s_target : target state
     * @return Node : if it is a empty node (id = -1), then do not find a path
     *         vector of Nodes : all the new nodes created during expansion
    */
    std::pair<MotionTree::Node, std::vector<MotionTree::Node>> ExpandTree(MotionTree::Node v, StateSpace::VehicleState s_target);
    

};