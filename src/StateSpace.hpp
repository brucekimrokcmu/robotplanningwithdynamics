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
        double theta_; // orientation angle
        double v_;     // velocity
        double phi_;   // steering angle

        VehicleState(){};
        VehicleState(double x, double y, double theta, double v, double phi)
            : x_(x),
              y_(y),
              theta_(theta),
              v_(v),
              phi_(phi)
              {
              };
    };

    struct VehicleHighDOFState{
        double x_;
        double y_;
        double z_;
        double roll_;   // roll
        double pitch_;   // pitch
        double yaw_;   // yaw
        double phi_; // steering angle 
        double vx_;  // linear velocithy
        double vy_;
        double vz_;
        double wx_;  // angular velocity
        double wy_;
        double wz_;

        VehicleHighDOFState(){};
        VehicleHighDOFState(double x, double y, double z, 
                            double roll, double pitch, double yaw, double phi,
                            double vx, double vy, double vz,
                            double wx, double wy, double wz)
            : x_(x),
              y_(y),
              z_(z),
              roll_(roll),
              pitch_(pitch),
              yaw_(yaw),
              phi_(phi),
              vx_(vx),
              vy_(vy),
              vz_(vz),
              wx_(wx),
              wy_(wy),
              wz_(wz)

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