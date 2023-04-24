#include "../src/GUST.hpp"
#include "../src/Update.hpp"
#include <iostream>
#include <cassert>
#include <fstream> // For reading/writing files
#include <sstream>
#include <string>

#include "../src/Constants.hpp"



void addMapConfig(
    string fpath, WorkSpace &W,
    double &start_x, double &start_y,
    double &goal_x, double &goal_y) {
    fstream fin;
    fin.open(fpath, ios::in);

    // First line is start state coordinates
    string line;
    getline(fin, line);
    stringstream starts(line);
    string startstate;
    getline(starts, startstate, ','); // split on comma
    start_x = stof(startstate);
    getline(starts, startstate, ',');
    start_y = stof(startstate);
    getline(fin, line); // Blank line to separate

    // Second line is start state coordinates
    getline(fin, line);
    stringstream goals(line);
    string goalstate;
    getline(goals, goalstate, ','); // split on comma
    goal_x = stof(goalstate);
    getline(goals, goalstate, ',');
    goal_y = stof(goalstate);
    getline(fin, line);

    while(getline(fin, line)) {
        
        // Split line on comma
        stringstream ss(line);
        string val;

        int index = 0;
        double coordinates[4];
        while (!ss.eof()) {
            getline(ss, val, ',');
            coordinates[index] = stof(val);
            index += 1;
        }
        Obstacle o = Obstacle(
            coordinates[0], coordinates[1],
            coordinates[2], coordinates[3]);
        W.addObstacle(o);
    }
}

int main(int argc, char** argv){
    srand(time(NULL));
    // Making a workspace
    WorkSpace W2 = WorkSpace(constants::workSpaceMinX,constants::workSpaceMaxX,
        constants::workSpaceMinY,constants::workSpaceMaxY);
    double start_x, start_y, goal_x, goal_y;
    addMapConfig("obstacles.txt", W2, start_x, start_y, goal_x, goal_y);

    // Making a state space
    StateSpace S = StateSpace(constants::workSpaceMaxX,constants::workSpaceMaxY,
        constants::stateSpaceMaxHeading, constants::stateSpaceMaxSpeed, constants::stateSpaceMaxSteering);
    // Initial state
    StateSpace::VehicleState s_init = StateSpace::VehicleState(start_x, start_y, 
        constants::initHeading, constants::stateSpaceMaxSpeed, constants::stateSpaceMaxSteering);
    // Making a control space (random we don't need this for now)
    Update U = Update();
    ControlSpace C = ControlSpace();
    // motion function (we don't need this for now)
     std::function<StateSpace::VehicleState(StateSpace::VehicleState, ControlSpace::VehicleControl, double)>
         motion = [&](StateSpace::VehicleState v, ControlSpace::VehicleControl c, double t){return U.Motion(v,c,t);};
    

    // Goal region
    Region goal_region = Region(goal_x,goal_y,constants::goalRegionWidth,constants::goalRegionWidth); 

    // goal function
    std::function<bool(StateSpace::VehicleState)> goal = [&](StateSpace::VehicleState s){
        if(goal_region.inRegion(s.x_, s.y_)){
            
            return true;
        }
        return false;
    };

    // valid function
    std::function<bool(StateSpace::VehicleState)> valid = [&](StateSpace::VehicleState s){

        std::vector<std::pair<double,double>> car;
        car.push_back(std::make_pair(s.x_, s.y_));
        car.push_back(std::make_pair(s.x_ + cos(s.theta_) * constants::carLength, s.y_ + sin(s.theta_) * constants::carLength));
        car.push_back(std::make_pair(s.x_ + cos(s.theta_) * constants::carLength - sin(s.theta_) * constants::carWidth, s.y_ + sin(s.theta_) * constants::carLength + cos(s.theta_) * constants::carWidth));
        car.push_back(std::make_pair(s.x_ + sin(s.theta_) * constants::carWidth, s.y_ + cos(s.theta_) * constants::carWidth));
                            
                           
       if(W2.Check_collision(car)){
           return false;
       }
       if(s.x_ < constants::workSpaceMinX || s.x_ > constants::workSpaceMaxX || s.y_ < constants::workSpaceMinY || s.y_ > constants::workSpaceMaxY){
              return false;
       }
       return true;
    };

    // ! Running GUST
    GUST gust = GUST( S, W2, C, motion, valid, s_init, goal, goal_region);
    std::vector<MotionTree::Node> allNodes;
    std::vector<MotionTree::Node> result = gust.RunGUST(allNodes);

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
    m_log_fstream << std::endl;

	// Then write out obstacle boundaries
    for(size_t i = 0; i < W2.countObstacleSize(); i++){
        double x =  W2.GetObstacle(i).x - W2.GetObstacle(i).x_extent;
        double y =  W2.GetObstacle(i).y - W2.GetObstacle(i).y_extent;
        double x_extent = W2.GetObstacle(i).x_extent * 2;
        double y_extent = W2.GetObstacle(i).y_extent * 2;
            m_log_fstream << x << ",";
            m_log_fstream << y << ",";
            m_log_fstream << x_extent << ",";
            m_log_fstream << y_extent;
            
            m_log_fstream << std::endl;
    }
    m_log_fstream << std::endl;

    // Region decomposition from GUST algorithm
	for (size_t i = 0; i < W2.countRegionSize() ; i++) {
		
        if(!W2.GetRegion(i).splitted){
            m_log_fstream << W2.GetRegion(i).x_start << ",";
            m_log_fstream << W2.GetRegion(i).x_extent << ",";
            m_log_fstream << W2.GetRegion(i).y_start << ",";
            m_log_fstream << W2.GetRegion(i).y_extent << ",";
            m_log_fstream << W2.GetRegion(i).h_value;
            
            m_log_fstream << std::endl;
        }
		
	}
    m_log_fstream << std::endl;

    // States for each step of the plan
    for (int i = result.size()-1; i >= 0; i--) {
            m_log_fstream << result[i].state.x_ << ",";
            m_log_fstream << result[i].state.y_ << ","; 
            m_log_fstream << result[i].state.v_ << ","; 
            m_log_fstream << result[i].state.theta_ << ","; 
            m_log_fstream << result[i].state.phi_; 
            m_log_fstream << std::endl;
	}
    m_log_fstream << std::endl;

    // (x,y) points sampled in motion tree
    for (size_t i = 0; i < allNodes.size() ; i++) {
            m_log_fstream << allNodes[i].state.x_ << ",";
            m_log_fstream << allNodes[i].state.y_; 
            m_log_fstream << std::endl;
	}
}