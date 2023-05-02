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
class GUST{
public: 
    GUST(StateSpace<State>& S, WorkSpace& W, ControlSpace<State,Control>& M,  
        std::function<State(State, Control, double)> motion, 
        std::function<bool(State)> valid, const State& s_init, 
        std::function<bool(State)> goal, Region goal_region)
            : S(S), W(W), M(M), motion(motion), valid(valid), s_init(s_init),goal(goal), goal_region(goal_region){}
    
    std::vector<typename MotionTree<State,Control>::Node> RunGUST(std::vector<typename MotionTree<State,Control>::Node> &allNodes){
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
            std::vector<typename MotionTree<State, Control>::Node> Lambda_r = Lambda.find(groupId)->second;
            if(Lambda_r.size() == 0){
                printf("wrong\n");
            }
            typename MotionTree<State, Control>::Node v_last = GroupPlanner(Lambda_r, groupId);

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
    void InitTreeAndGroups(){
        T = MotionTree<State, Control>();
        T.nodes[0].state = s_init;
        double x = Proj(s_init).first;
        double y = Proj(s_init).second;
        T.nodes[0].region = W.LocateRegion(x,y);
        Lambda[T.nodes[0].region].push_back(T.nodes[0]);
    };
    /**
     * Select a group of vertices from Lambda which has largest weight
     * @return pair of vector of Nodes and the region ID
    */
    std::pair<std::vector<typename MotionTree<State,Control>::Node>, int> SelectGroup(){
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
    /**
     * Expand the tree from a group of vertices
     * @param Lambda_r : vector of Nodes
     * @param r : region ID
     * @return Node : if it is a empty node (id = -1), then do not find a path
     *              else return the path
    */
    typename MotionTree<State,Control>::Node GroupPlanner(std::vector<typename MotionTree<State,Control>::Node> Lambda_r, int r)
    {
        State s_target = SampleTarget(r);
        typename MotionTree<State, Control>::Node v = SelectVertex(Lambda_r, s_target);
        // s_target.v_ = 0.5;
        std::pair<typename MotionTree<State, Control>::Node, std::vector<typename MotionTree<State, Control>::Node>> new_v = ExpandTree(v, s_target);

        typename MotionTree<State, Control>::Node v_last = new_v.first;
        std::vector<typename MotionTree<State, Control>::Node> new_vertices = new_v.second;

        for (typename MotionTree<State, Control>::Node v_new : new_vertices) {
            std::pair<double,double> v_new_point = Proj(v_new.state);
            int r_new = W.LocateRegion(v_new_point.first, v_new_point.second);

            typename std::unordered_map<int,std::vector< typename MotionTree<State, Control>::Node>>::const_iterator iter = EmptyLambda.find(r_new);
            // If region of the v_new is in the set of empty regions
            if (iter != EmptyLambda.end()) {
                // Remove r_new from empty splits
                std::vector<typename MotionTree<State, Control>::Node> vertex_list = iter->second;
                EmptyLambda.erase(r_new);

                // Add r_new back to Lambda
                Lambda.insert(
                    make_pair(
                        r_new, std::vector<typename MotionTree<State, Control>::Node>{v_new}
                    )
                );
            }
            else if(Lambda.find(r_new) == Lambda.end()){
                // If region of the v_new is in the set of non-empty regions
                std::pair<int,std::vector<typename MotionTree<State, Control>::Node>> new_group = NewGroup(r_new, v_new);
                int r_new_new = new_group.first;
                std::vector<typename MotionTree<State, Control>::Node> vertex_list = new_group.second;
    
                // Add r_new_new to Lambda
                Lambda.insert(
                    make_pair(
                        r_new_new, std::vector<typename MotionTree<State, Control>::Node>{v_new}
                    )
                );
            }
            else{
                // If region of the v_new is not in the set of regions
                Lambda.find(r_new)->second.push_back(v_new);
            }
        }
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
    State SampleTarget(int r){
        // randomly sample a state from S in region r

        Region lambda_R = W.GetRegion(r);
        State s_target;
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
                double dist = PointDistance(W.GetRegion(r).x_start, W.GetRegion(r).y_start, goal_region.x_start, goal_region.y_start);
                double frac = rand()/RAND_MAX * dist;
                s_target.x_ = W.GetRegion(r).x_start + frac * (goal_region.x_start - W.GetRegion(r).x_start);
                s_target.y_ = W.GetRegion(r).y_start + frac * (goal_region.y_start - W.GetRegion(r).y_start);
            }
        }else{
            s_target = S.getRandomState();
        }

        return s_target;
    };
    /**
     * Select a vertex from Lambda_r
     * @param Lambda_r : vector of Nodes
     * @param s_target : target state
     * @return Node to expand
    */
    typename MotionTree<State,Control>::Node SelectVertex(std::vector<typename MotionTree<State,Control>::Node> Lambda_r, State s_target){
        std::pair<double,double> p_target = GUST::Proj(s_target);

        typename MotionTree<State, Control>::Node v_closest;
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
    // int LocateRegion(double x, double y);

    std::pair<int,std::vector<typename MotionTree<State,Control>::Node>> NewGroup(int r, typename MotionTree<State,Control>::Node v_new){
         return make_pair(r, std::vector<typename MotionTree<State, Control>::Node>{T.getNode(v_new.id)});
    };

};
