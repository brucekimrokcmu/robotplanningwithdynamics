#pragma once
#include <vector>
#include <limits>
#include <iostream>
#include <cmath>
#include <random>

#include "Constants.hpp"
class StateSpace{

public:
    struct VehicleState{
        double x_;     // x-position
        double y_;     // y-position
        double z_;     // z-positon
        double theta_; // orientation angle
        double vx_;    // linear x-velocity
        double vy_;    // linear y-velocity
        double vz_;    // linear z-velocity
        double wx_;    // angular x-velocity 
        double wy_;    // angular y-velocity
        double wz_;    // angular z-velocity
        double phi_;   // steering angle

        VehicleState(){};
        VehicleState(double x, double y, double theta, double vx, double vy, double vz, double wx, double wy, double wz, double phi)
            : x_(x),
              y_(y),
              theta_(theta),
              vx_(vx),
              vy_(vy),
              vz_(vz),
              wx_(wx),
              wy_(wy),
              wz_(wz),
              phi_(phi)
              {
              };
    };

    StateSpace(double maxX, double maxY, double maxHeadingAngle, double maxSpeed, double maxSteering)
        :   maxX(maxX), 
            maxY(maxY),
            maxHeadingAngle(maxHeadingAngle), 
            maxSpeed(maxSpeed), 
            maxSteering(maxSteering)   
        {
        };
    
    /**
     * 
     * @return random VehicleState
    */
    VehicleState getRandomState();
    bool nearTarget(VehicleState s_new, VehicleState s_target);

private:
    double maxX; 
    double maxY; 
    double maxHeadingAngle; 
    double maxSpeed; 
    double maxSteering;
};