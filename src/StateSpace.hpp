#pragma once
#include <vector>
#include <limits>
#include <iostream>
#include <cmath>
#include <random>
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
    
    /**
     * TODO: need change when implementing controller
     * @return random VehicleState
    */
    VehicleState getRandomState(){
 
        double x = (double)rand() / RAND_MAX * maxX;
        double y = (double)rand() / RAND_MAX * maxY;
        printf("%f, %f\n", x, y);
        double theta = (double)rand() / RAND_MAX * maxHeadingAngle;
        double v = (double)rand() / RAND_MAX * maxSpeed;
        double phi = (double)rand() / RAND_MAX * maxSteering;
        return VehicleState(x, y, theta, v, phi);
    };
    // bool nearTarget(VehicleState s_new, VehicleState s_target);
    // bool goalState(VehicleState s_new);
    

private:
    double maxX; 
    double maxY; 
    double maxHeadingAngle; 
    double maxSpeed; 
    double maxSteering;
};