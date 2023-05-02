#pragma once
#include <unordered_map>
#include <functional>
#include <vector>
#include <chrono> 

#include "../WorkSpace.hpp"
#include "../StateSpaces/StateSpace.hpp"
#include "../ControlSpaces/ControlSpace.hpp"
#include "../MotionTree.hpp"
#include "../Constants.hpp"

template<typename State, typename Control>
class RRT{
public: 
    RRT(StateSpace<State>& S, WorkSpace& W, ControlSpace<State,Control>& M,  
        std::function<State(State, Control, double)> motion, 
        std::function<bool(State)> valid, const State& s_init, 
        std::function<bool(State)> goal, Region goal_region)
            : S(S), W(W), M(M), motion(motion), valid(valid), s_init(s_init),goal(goal), goal_region(goal_region){}
    
    std::vector<typename MotionTree<State,Control>::Node> RunRRT(std::vector<typename MotionTree<State,Control>::Node> &allNodes){
       

        InitTree();
        // printf("Finish Init\n");
        auto start = std::chrono::high_resolution_clock::now();  
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        while(duration.count() < time){ // TODO: change to max time
         
            typename MotionTree<State, Control>::Node v_last = GroupPlanner();

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
        return std::vector<typename MotionTree<State, Control>::Node>{}; 
    };  // Main function to Run GUST
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

    // Helper Functions
    /*
    * Euclidean distance between two points
    */
    inline double PointDistance(
        double x1, double y1, double x2, double y2) {
        double x_diff = pow(x1 - x2, 2);
        double y_diff = pow(y1 - y2, 2);
        return sqrt(x_diff + y_diff);
    }
    // Function Interfaces
    /**
     * Initialize the motion tree (T) and the set of all set of vertices in small regions (Lambda)
    */
    void InitTree(){
        T = MotionTree<State, Control>();
        T.nodes[0].state = s_init;
    };

    /**
     * Expand the tree from a group of vertices
     * @param Lambda_r : vector of Nodes
     * @param r : region ID
     * @return Node : if it is a empty node (id = -1), then do not find a path
     *              else return the path
    */
    typename MotionTree<State,Control>::Node GroupPlanner()
    {
        State s_target = SampleTarget();
        typename MotionTree<State, Control>::Node v = SelectVertex( s_target);
        // s_target.v_ = 0.5;
        std::pair<typename MotionTree<State, Control>::Node, std::vector<typename MotionTree<State, Control>::Node>> new_v = ExpandTree(v, s_target);
        typename MotionTree<State, Control>::Node v_last = new_v.first;
       
        return v_last;
    };
    /**
     * Refine the decomposition of the workspace
     * @param r : region ID
    */
    void SplitGroup(int r){
        std::vector<Region> new_regions = W.SplitRegion(W.GetRegion(r));
        if(new_regions.size() == 0){
            return;
        }
        else{
            W.setSplitted(r);
            for (auto &region : new_regions)
            {
                int id = W.addRegion(region);
                
                std::vector<typename MotionTree<State, Control>::Node> vertex_list;
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
    };

    // Help Functions for `GroupPlanner`
    /**
     * Sample a target state in a region
     * @param r : region ID
    */
    State SampleTarget(){
   // randomly sample a state from S in region r
        if(rand() < 0.5){
            State s_target = S.getRandomState();
            s_target.x_ = goal_region.x_start;
            s_target.y_ = goal_region.y_start;
            return s_target;
        }
        State s_target = S.getRandomState();
        return s_target;
    };
    /**
     * Select a vertex from Lambda_r
     * @param s_target : target state
     * @return Node to expand
    */
    typename MotionTree<State,Control>::Node SelectVertex(State s_target){
        double min_dist = std::numeric_limits<double>::max();
        int idx;
        for(auto node : T.nodes) {
            if(PointDistance(node.state.x_, node.state.y_, s_target.x_, s_target.y_) < min_dist) {
                min_dist = PointDistance(node.state.x_, node.state.y_, s_target.x_, s_target.y_);
                idx = node.id;
            }
        }

        return T.getNode(idx);
    };
    /**
     * Expand the tree from a vertex
     * @param v : Node to expand
     * @param s_target : target state
     * @return Node : if it is a empty node (id = -1), then do not find a path
     *         vector of Nodes : all the new nodes created during expansion
    */
    std::pair<typename MotionTree<State,Control>::Node,  std::vector<typename MotionTree<State,Control>::Node>> ExpandTree(typename MotionTree<State,Control>::Node v, State s_target){
        std::vector<typename MotionTree<State, Control>::Node> new_vertices;
        Control u;


        bool usePID = ((double) rand() / (RAND_MAX)) < constants::usePID;
        // usePID = false; // ! Delete when integrating PID controller
    
        if (!usePID) {
            u = M.RandomController();
        }

        int numIter = rand()%(constants::maxNrSteps-constants::minNrSteps + 1) + constants::minNrSteps;
        typename MotionTree<State, Control>::Node v_parent = v;

        for (int k=1; k<numIter; k++) {
            if (usePID) {
                u = M.PIDController(v.state, s_target);
            }

            State s_new = motion(v.state, u, constants::dt);
            
            
            if (!valid(s_new)) {
                return std::make_pair(typename MotionTree<State, Control>::Node(), new_vertices);
            }
            
            typename MotionTree<State, Control>::Node v_new = T.newVertex(s_new);
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
                
                return std::make_pair(typename MotionTree<State, Control>::Node(), new_vertices);
            }

            v = v_new;
        }

        return std::make_pair(typename MotionTree<State, Control>::Node(), new_vertices);

    };
    
    /**
     * Project a state to the workspace
     * @param s : VehicleState
     * @return pair of x and y in the workspace
    */
    std::pair<double, double> Proj(State s){
        return std::make_pair(s.x_, s.y_);
    };


};
