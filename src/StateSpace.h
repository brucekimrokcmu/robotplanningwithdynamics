#include "VehicleState.h"

class StateSpace{
private:
    double maxX; 
    double maxY; 
    double maxHeadingAngle; 
    double maxSpeed; 
    double maxSteering;
public:
    StateSpace(double maxX, double maxY, double maxHeadingAngle, double maxSpeed, double maxSteering)
        : maxX(maxX), maxY(maxY),maxHeadingAngle(maxHeadingAngle), maxSpeed(maxSpeed), maxSteering(maxSteering){};
    VehicleState getRandomState(int r);
    bool nearTarget(VehicleState s_new, VehicleState s_target);
    bool validState();
};