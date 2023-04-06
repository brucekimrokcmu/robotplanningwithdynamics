#include <cassert>
#include <iostream>
#include <fstream> // For reading/writing files
#include <string>

#include "../src/WorkSpace.hpp"

int main(int argc, char** argv){
    Obstacle o1 = Obstacle(3,3,1,1);
    WorkSpace W2 = WorkSpace(0,10,0,10);
    W2.addObstacle(o1);
    W2.Decompose();
/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
    std::string outputFile = argv[2];
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << std::endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (size_t i = 0; i < W2.countRegionSize() ; i++) {
		
        if(!W2.GetRegion(i).splitted){
            m_log_fstream << W2.GetRegion(i).x_start << ",";
            m_log_fstream << W2.GetRegion(i).x_extent << ",";
            m_log_fstream << W2.GetRegion(i).y_start << ",";
            m_log_fstream << W2.GetRegion(i).y_extent << ",";
            
            m_log_fstream << std::endl;

        }
		
	}
}