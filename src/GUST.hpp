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
    
    std::vector<MotionTree::Node> RunGUST(std::vector<MotionTree::Node> &allNodes);  // Main function to Run GUST
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
    double epsilon = 0.5;

    // Motion Tree
    MotionTree T;
    // Set of set of vertices in small regions
    std::unordered_map<int,std::vector<MotionTree::Node>> Lambda;
    std::unordered_map<int,std::vector<MotionTree::Node>> EmptyLambda;

    // Function Interfaces
    /**
     * Initialize the motion tree (T) and the set of all set of vertices in small regions (Lambda)
    */
    void InitTreeAndGroups();
    /**
     * Select a group of vertices from Lambda which has largest weight
     * @return pair of vector of Nodes and the region ID
    */
    std::pair<std::vector<MotionTree::Node>, int> SelectGroup();
    /**
     * Expand the tree from a group of vertices
     * @param Lambda_r : vector of Nodes
     * @param r : region ID
     * @return Node : if it is a empty node (id = -1), then do not find a path
     *              else return the path
    */
    MotionTree::Node GroupPlanner(std::vector<MotionTree::Node> Lambda_r, int r);
    /**
     * Refine the decomposition of the workspace
     * @param r : region ID
    */
    void SplitGroup(int r);

    // Help Functions for `GroupPlanner`
    /**
     * TODO: need implement with small probability choose a random state in the shortes path to the goal
     * Sample a target state in a region
     * @param r : region ID
    */
    StateSpace::VehicleState SampleTarget(int r);
    /**
     * Select a vertex from Lambda_r
     * @param Lambda_r : vector of Nodes
     * @param s_target : target state
     * @return Node to expand
    */
    MotionTree::Node SelectVertex(std::vector<MotionTree::Node> Lambda_r, StateSpace::VehicleState s_target);
    /**
     * Expand the tree from a vertex
     * @param v : Node to expand
     * @param s_target : target state
     * @return Node : if it is a empty node (id = -1), then do not find a path
     *         vector of Nodes : all the new nodes created during expansion
    */
    std::pair<MotionTree::Node, std::vector<MotionTree::Node>> ExpandTree(MotionTree::Node v, StateSpace::VehicleState s_target);
    
    /**
     * Project a state to the workspace
     * @param s : VehicleState
     * @return pair of x and y in the workspace
    */
    std::pair<double, double> Proj(StateSpace::VehicleState s);
    // int LocateRegion(double x, double y);


};