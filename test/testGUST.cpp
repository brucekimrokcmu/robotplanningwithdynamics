#include "../src/GUST.hpp"
#include "../src/Update.hpp"
#include <iostream>
#include <cassert>
#include <fstream> // For reading/writing files
#include <sstream>
#include <string>


#define PI_D 3.14159265358979323846

void addAllObstacles(string fpath, WorkSpace &W) {
    fstream fin;
    fin.open(fpath, ios::in);

    string line;
    while(getline(fin, line)) {
        
        // Split line on comma
        stringstream ss(line);
        string val;

        int index = 0;
        int coordinates[4];
        while (!ss.eof()) {
            getline(ss, val, ',');
            coordinates[index] = stoi(val);
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
    //Obstacle o1 = Obstacle(3,3,0.5,0.5);
    WorkSpace W2 = WorkSpace(0,30,0,30);
    // Obstacle o2 = Obstacle(10,13,0.5,0.5);
    // Obstacle o3 = Obstacle(5,5,1,1);
    // Obstacle o4 = Obstacle(12,5,1,1);
    // Obstacle o5 = Obstacle(17.5,10,1,1);
    // Obstacle o6 = Obstacle(2.5,17.5,1,1);
    // Obstacle o7 = Obstacle(10,17.5,1,1);
    // W2.addObstacle(o1);
    // W2.addObstacle(o2);
    // W2.addObstacle(o3);
    // W2.addObstacle(o4);
    // W2.addObstacle(o5);
    // W2.addObstacle(o6);
    // W2.addObstacle(o7);
    addAllObstacles("obstacles.txt", W2);

    // Making a state space
    StateSpace S = StateSpace(20,20,2*PI_D, 1, PI_D/6);
    // Initial state
    StateSpace::VehicleState s_init = StateSpace::VehicleState(0,0,0,0,0);
    // Making a control space (random we don't need this for now)
    Update U = Update(1.5,0.5,1,1,0.7);
    ControlSpace C = ControlSpace();
    // motion function (we don't need this for now)
     std::function<StateSpace::VehicleState(StateSpace::VehicleState, ControlSpace::VehicleControl, double)>
         motion = [&](StateSpace::VehicleState v, ControlSpace::VehicleControl c, double t){return U.Motion(v,c,t);};
    

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

        std::vector<std::pair<double,double>> car;
        car.push_back(std::make_pair(s.x_, s.y_));
        car.push_back(std::make_pair(s.x_ + cos(s.theta_) * 0.5, s.y_ + sin(s.theta_) * 0.5));
        car.push_back(std::make_pair(s.x_ + cos(s.theta_) * 0.5 - sin(s.theta_) * 0.25, s.y_ + sin(s.theta_) * 0.5 + cos(s.theta_) * 0.25));
        car.push_back(std::make_pair(s.x_ + sin(s.theta_) * 0.25, s.y_ + cos(s.theta_) * 0.25));
                            
                           
       if(W2.Check_collision(car)){
           return false;
       }
       if(s.x_ < 0 || s.x_ > 20 || s.y_ < 0 || s.y_ > 20){
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