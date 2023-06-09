#include <iostream>
#include <cassert>
#include <fstream> // For reading/writing files
#include <string>
#include "../src/ControlSpace.hpp"
#include "../src/Update.hpp"

int main(){
    ControlSpace cs = ControlSpace();
    // // Test random controller
    // ControlSpace::VehicleControl randomControl = cs.RandomController(); 
    // std::cout << "Random control: " << randomControl.acc << ", " << randomControl.steering_rate << std::endl;
    // std::vector<StateSpace::VehicleState > states;
    // Update up = Update(1.0, 0.5, 1.0, 1.0, 0.785);
    // StateSpace::VehicleState s_curr = StateSpace::VehicleState(0.0, 0.0, 1.57, 0.0, 0.0);
    // states.push_back(s_curr);
    // int count = 0;
    // while(count < 10){
    //     StateSpace::VehicleState s_new = up.Motion(s_curr, randomControl, 1);
    //     states.push_back(s_new);
    //     s_curr = s_new;
    //     count++;
    // }
   

    // Test PID controller
    StateSpace::VehicleState s_curr = StateSpace::VehicleState(5.0, 5.0, 1.57, 0.5, 0.0);
    StateSpace::VehicleState s_target = StateSpace::VehicleState(75.5, 3.5, 0, 0.5, -0.1);
    ControlSpace::VehicleControl pidControl;
    Update up;
    StateSpace::VehicleState s_new;
    std::vector<StateSpace::VehicleState > pidstates;
    pidstates.push_back(s_curr);
    const int numIter = 100;
    int count = 0;
    while(count < numIter){
        pidControl = cs.PIDController(s_curr,s_target);
        std::cout << "PID control: " << pidControl.acc << ", " << pidControl.steering_rate << std::endl;
        s_new = up.Motion(s_curr, pidControl, 1);
        pidstates.push_back(s_new);
        
        if (std::sqrt(std::pow(s_target.x_-s_curr.x_, 2) + std::pow(s_target.y_-s_curr.y_, 2)) < 0.5) {
            break;
        }
        s_curr = s_new;
        pidControl = cs.PIDController(s_curr,s_target);
        std::cout << "PID control: " << pidControl.acc << ", " << pidControl.steering_rate << std::endl;
        count++;
    }
    if (count == numIter) {
        std::cout<< "No solution!" <<std::endl;
    }


    // Output file
    std::string outputFile = "testPIDControl.txt";
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	/// Then write out all the joint angles in the plan sequentially
    for(size_t i = 0; i < pidstates.size(); i++){
        double x =  pidstates[i].x_;
        double y =  pidstates[i].y_;
        double theta = pidstates[i].theta_;
        double speed = pidstates[i].v_;
        double phi = pidstates[i].phi_;
        m_log_fstream << x << ",";
        m_log_fstream << y << ",";
        m_log_fstream << theta << ",";
        m_log_fstream << speed << ",";
        m_log_fstream << phi;
        
        if (i < pidstates.size()-1)
            m_log_fstream << std::endl;
    }

	


}