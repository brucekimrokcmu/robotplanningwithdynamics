#pragma once
#include <unordered_map>
#include <functional>
#include <vector>
#include <chrono> 

#include "WorkSpace.hpp"
#include "./StateSpaces/StateSpace.hpp"
#include "./ControlSpaces/ControlSpace.hpp"
#include "MotionTree.hpp"
#include "Constants.hpp"

template<typename State, typename Control>
class GUST{
public: 
    GUST(StateSpace<State>& S, WorkSpace& W, ControlSpace<State,Control>& M,  
        std::function<State(State, Control, double)> motion, 
        std::function<bool(State)> valid, const State& s_init, 
        std::function<bool(State)> goal, Region goal_region)
            : S(S), W(W), M(M), motion(motion), valid(valid), s_init(s_init),goal(goal), goal_region(goal_region){}
    
    std::vector<typename MotionTree<State,Control>::Node> RunGUST(std::vector<typename MotionTree<State,Control>::Node> &allNodes);  // Main function to Run GUST
private:
    // Input Parameters
    StateSpace<State>& S; 
    WorkSpace& W; 
    ControlSpace<State,Control>& M;  
    std::function<State(State, Control, double)> motion; 
    std::function<bool(State)> valid;
    const State& s_init;
    std::function<bool(State)> goal;
    Region goal_region;

    // Constants
    double time = constants::MAXTIME; 
    double delta = constants::DELTA; 
    double alpha = constants::ALPHA;
    double beta = constants::BETA;
    double smallProb = constants::targetNearGoal;
    double epsilon = constants::EPSILON;

    // Motion Tree
    MotionTree<State,Control> T;
    // Set of set of vertices in small regions
    std::unordered_map<int,std::vector<typename MotionTree<State,Control>::Node>> Lambda;
    std::unordered_map<int,std::vector<typename MotionTree<State,Control>::Node>> EmptyLambda;

    // Function Interfaces
    /**
     * Initialize the motion tree (T) and the set of all set of vertices in small regions (Lambda)
    */
    void InitTreeAndGroups();
    /**
     * Select a group of vertices from Lambda which has largest weight
     * @return pair of vector of Nodes and the region ID
    */
    std::pair<std::vector<typename MotionTree<State,Control>::Node>, int> SelectGroup();
    /**
     * Expand the tree from a group of vertices
     * @param Lambda_r : vector of Nodes
     * @param r : region ID
     * @return Node : if it is a empty node (id = -1), then do not find a path
     *              else return the path
    */
    typename MotionTree<State,Control>::Node GroupPlanner(std::vector<typename MotionTree<State,Control>::Node> Lambda_r, int r);
    /**
     * Refine the decomposition of the workspace
     * @param r : region ID
    */
    void SplitGroup(int r);

    // Help Functions for `GroupPlanner`
    /**
     * Sample a target state in a region
     * @param r : region ID
    */
    State SampleTarget(int r);
    /**
     * Select a vertex from Lambda_r
     * @param Lambda_r : vector of Nodes
     * @param s_target : target state
     * @return Node to expand
    */
    typename MotionTree<State,Control>::Node SelectVertex(std::vector<typename MotionTree<State,Control>::Node> Lambda_r, State s_target);
    /**
     * Expand the tree from a vertex
     * @param v : Node to expand
     * @param s_target : target state
     * @return Node : if it is a empty node (id = -1), then do not find a path
     *         vector of Nodes : all the new nodes created during expansion
    */
    std::pair<typename MotionTree<State,Control>::Node,  std::vector<typename MotionTree<State,Control>::Node>> ExpandTree(typename MotionTree<State,Control>::Node v, State s_target);
    
    /**
     * Project a state to the workspace
     * @param s : VehicleState
     * @return pair of x and y in the workspace
    */
    std::pair<double, double> Proj(State s);
    // int LocateRegion(double x, double y);

    std::pair<int,std::vector<typename MotionTree<State,Control>::Node>> NewGroup(int r, typename MotionTree<State,Control>::Node v_new);


};

#include "GUST.cpp"