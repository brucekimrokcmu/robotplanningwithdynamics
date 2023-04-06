#pragma once
class StateSpace{

public:
    struct VehicleState{
        double x_;     // x-position
        double y_;     // y-position
        double theta_; // orientation angle
        double v_;     // velocity
        double phi_;   // steering angle

        VehicleState(){}
        VehicleState(double x, double y, double theta, double v, double phi) :
        x_(x), y_(y), theta_(theta), v_(v), phi_(phi){}
    };
    StateSpace(double maxX, double maxY, double maxHeadingAngle, double maxSpeed, double maxSteering)
        : maxX(maxX), maxY(maxY),maxHeadingAngle(maxHeadingAngle), maxSpeed(maxSpeed), maxSteering(maxSteering){};
    VehicleState getRandomState(int r);
    bool nearTarget(VehicleState s_new, VehicleState s_target);
    bool validState();
    

private:
    double maxX; 
    double maxY; 
    double maxHeadingAngle; 
    double maxSpeed; 
    double maxSteering;
};