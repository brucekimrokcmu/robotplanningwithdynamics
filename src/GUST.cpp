#include "GUST.hpp"
#include <limits>
#include <vector>
#include "WorkSpace.hpp"
#include "StateSpace.hpp"
#include "ControlSpace.hpp"
#include "MotionTree.hpp"

#define minNrSteps 500
#define maxNrSteps 1500

/*
 * Calculate heuristics of all regions
*/
void GUST::CalculateHeuristics(){
    
}

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

    // return fixed state in region
    StateSpace::VehicleState s_target = VehicleState(
        lambda_R.x_start, lambda_R.y_start, 0,
        0, 0);

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
    return pair(s.x_, s.y_);
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
    for (auto v : Lambda_r.vertices) {

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
std::pair(MotionTree::Node, std::vector<MotionTree::Node>) GUST::ExpandTree(
    MotionTree::Node v, StateSpace::VehicleState s_target) {

    std::vector<MotionTree::Node> new_vertices;
    VehicleControl u;

    // TODO: Implement random controller
    bool usePID = ((double) rand() / (RAND_MAX)) < 0.5;
    if (!usePID) {
        u = RandomController();
    }

    int numIter = rand()%(maxNrSteps-minNrSteps + 1) + minNrSteps;

    // TODO: Implement PID controller, Runge-Katta integration for motion,
    for (int k=1; k<numIter; k++) {
        if (usePID) {
            u = PIDController(v.state, s_target);
        }

        // TODO: implement motion function --> this should be passed in as an arg to the GUST constructor.
        VehicleState s_new = motion(v.state, u, dt);

        if (!StateSpace::validState(s_new)) {
            return NULL;
        }
        
        MotionTree::Node v_new = T.newVertex(s_new);
        v_new.parent = v;
        v_new.state = s_new;
        v_new.control = u;

        new_vertices.push_back(v_new);

        if (StateSpace::goalState(s_new)) {
            return std::pair(v_new, new_vertices);
        }
        if (StateSpace::nearTarget(s_new, s_target)) {
            return NULL;
        }

        v = v_new;
    }
    return NULL;
}

std::pair<int,std::vector<MotionTree::Node>> NewGroup(
    int r, MotionTree::Node v_new) {

    return NULL;
}

MotionTree::Node GUST::GroupPlanner(
    std::vector<MotionTree::Node> Lambda_r, int r) {

    StateSpace::VehicleState s_target = SampleTarget(r);
    MotionTree::Node v = SelectVertex(Lambda_r, s_target);
    std::pair<MotionTree::Node, std::vector<MotionTree::Node>> new_v = ExpandTree(v, s_target);

    MotionTree::Node v_last = new_v.first;
    MotionTree::Node new_vertices = new_v.second;

    for (MotionTree::Node v_new : new_vertices) {
        std::pair<double,double> v_new_point = Proj(v_new.state);
        int r_new = W.LocateRegion(v_new_point.first, v_new_point.second);

        std::unordered_map<int,std::vector<MotionTree::Node>>::const_iterator iter = EmptyLambda.find(r_new);
        // If region of the v_new is in the set of empty regions
        if (iter == EmptyLambda.end()) {
            // Remove r_new from empty splits
            std::vector<MotionTree::Node> vertex_list = iter.second;
            EmptyLambda.erase(r_new);

            // Add r_new back to Lambda
            Lambda.insert(
                std::make_pair<std::int,std::vector<MotionTree::Node>>(
                    r_new, vertex_list
                )
            );
        }
    }

}