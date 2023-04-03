#include <unordered_map>
#include <functional>

#include "WorkSpace.h"
#include "StateSpace.h"
#include "ControlSpace.h"
#include "VehicleState.h"
#include "VehicleMotion.h"
#include "MotionTree.h"

class GUST{
public: 
    GUST(const StateSpace& S, const WorkSpace& W, const ControlSpace& M,  
        std::function<VehicleState(VehicleState, VehicleMotion, double)> motion, 
        std::function<bool(VehicleState)> valid, const VehicleState& s_init, 
        std::function<bool(VehicleState)> goal)
            : S(S), W(W), M(M), motion(motion), valid(valid), s_init(s_init),goal(goal){}
    
    std::vector<MotionTree::Node> RunGUST();  // Main function to Run GUST
private:
    // Input Parameters
    const StateSpace& S; 
    const WorkSpace& W; 
    const ControlSpace& M;  
    std::function<VehicleState(VehicleState, VehicleMotion, double)> motion; 
    std::function<bool(VehicleState)> valid;
    const VehicleState& s_init;
    std::function<bool(VehicleState)> goal;

    // Constants
    double time; // TODO: max time to sampling

    // Motion Tree
    MotionTree T;
    // Set of set of vertices in small regions
    std::unordered_map<int,std::vector<MotionTree::Node>> Lambda;
    std::unordered_map<int,std::vector<MotionTree::Node>> EmptyLambda;

    // Function Interfaces
    void initTreeAndGroups();
    std::pair<std::vector<MotionTree::Node>, int> SelectGroup();
    MotionTree::Node GroupPlanner(std::vector<MotionTree::Node> Labda_r, int r);
    void SplitGroup(int r);

    // Help Functions for `GroupPlanner`
    VehicleState SampleTarget(int r);
    MotionTree::Node SelectVertex(std::vector<MotionTree::Node> Labda_r, VehicleState s_target);
    MotionTree::Node ExpandTree(MotionTree::Node v, VehicleState s_target);
    
    std::pair<double, double> proj(VehicleState s);
    int LocateRegion(double x, double y);


};