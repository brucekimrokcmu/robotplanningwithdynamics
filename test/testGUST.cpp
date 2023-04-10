#include "../src/GUST.hpp"
#include <iostream>
#include <cassert>
#include <fstream> // For reading/writing files
#include <string>

#define PI_D 3.14159265358979323846

int main(int argc, char** argv){
    srand(time(NULL));
    // Making a workspace
    Obstacle o1 = Obstacle(3,3,0.5,0.5);
    WorkSpace W2 = WorkSpace(0,20,0,20);
    Obstacle o2 = Obstacle(10,13,0.5,0.5);
    Obstacle o3 = Obstacle(5,5,1,1);
    Obstacle o4 = Obstacle(12,5,1,1);
    W2.addObstacle(o1);
    W2.addObstacle(o2);
     W2.addObstacle(o3);
    W2.addObstacle(o4);

    // Making a state space
    StateSpace S = StateSpace(20,20,2*PI_D, 1, PI_D/2);
    // Making a control space (random we don't need this for now)
    ControlSpace C = ControlSpace(1, 1);
    // motion function (we don't need this for now)
     std::function<StateSpace::VehicleState(StateSpace::VehicleState, ControlSpace::VehicleControl, double)>
         motion = [&](StateSpace::VehicleState s, ControlSpace::VehicleControl c, double dt){
        StateSpace::VehicleState s_new;
        // TODO: Implement this function
        return s_new;
    };
    // Initial state
    StateSpace::VehicleState s_init = StateSpace::VehicleState(0,0,0,0,0);

    // Goal region
    Region goal_region = Region(18,18,0.5,0.5); 

    // goal function
    std::function<bool(StateSpace::VehicleState)> goal = [&](StateSpace::VehicleState s){
        if(goal_region.inRegion(s.x_, s.y_)){
            
            return true;
        }
        return false;
    };

    // valid function
    std::function<bool(StateSpace::VehicleState)> valid = [&](StateSpace::VehicleState s){
       if(W2.Check_collision(s.x_, s.y_)){
           return false;
       }
       return true;
    };

    // ! Running GUST
    GUST gust = GUST( S, W2, C, motion, valid, s_init, goal, goal_region);
    std::vector<MotionTree::Node> allNodes;
    std::vector<MotionTree::Node> result = gust.RunGUST(allNodes);
    assert(allNodes.size() > 0);

    // Printing the solution
    for(auto n : result){
        std::cout << n.state.x_ << " " << n.state.y_  << std::endl;
    }

    

    /** 
     * Saves the solution to output file
	 */
    std::string outputFile = argv[2];
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << std::endl; // Write out map name first

	/// Then write out all the joint angles in the plan sequentially
    for(size_t i = 0; i < W2.countObstacleSize(); i++){
        double x =  W2.GetObstacle(i).x - W2.GetObstacle(i).x_extent;
        double y =  W2.GetObstacle(i).y - W2.GetObstacle(i).y_extent;
        double x_extent = W2.GetObstacle(i).x_extent * 2;
        double y_extent = W2.GetObstacle(i).y_extent * 2;
            m_log_fstream << x << ",";
            m_log_fstream << y << ",";
            m_log_fstream << x_extent << ",";
            m_log_fstream << y_extent << ",";
            
            m_log_fstream << std::endl;
    }
    m_log_fstream << std::endl;
	for (size_t i = 0; i < W2.countRegionSize() ; i++) {
		
        if(!W2.GetRegion(i).splitted){
            m_log_fstream << W2.GetRegion(i).x_start << ",";
            m_log_fstream << W2.GetRegion(i).x_extent << ",";
            m_log_fstream << W2.GetRegion(i).y_start << ",";
            m_log_fstream << W2.GetRegion(i).y_extent << ",";
            m_log_fstream << W2.GetRegion(i).h_value << ",";
            
            m_log_fstream << std::endl;

        }
		
	}
    m_log_fstream << std::endl;
    for (size_t i = 0; i < result.size() ; i++) {
            m_log_fstream << result[i].state.x_ << ",";
            m_log_fstream << result[i].state.y_ << ","; 
            m_log_fstream << std::endl;
	}
    m_log_fstream << std::endl;
    for (size_t i = 0; i < allNodes.size() ; i++) {
            m_log_fstream << allNodes[i].state.x_ << ",";
            m_log_fstream << allNodes[i].state.y_ << ","; 
            m_log_fstream << std::endl;
	}

}