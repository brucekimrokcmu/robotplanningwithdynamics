#pragma once
#include <vector>
#include <limits>
#include <iostream>
#include <cmath>
#include <random>

#include "StateSpace.hpp"
#include "../Constants.hpp"
#include "../Subjects/Vehicle2D.hpp"



class Vehicle2DStateSpace : public StateSpace<Vehicle2D::Vehicle2DState> {

public:
    
    Vehicle2DStateSpace(double maxX, double maxY, double maxHeadingAngle, double maxSpeed, double maxSteering)
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
    Vehicle2D::Vehicle2DState getRandomState() override;
    bool nearTarget(Vehicle2D::Vehicle2DState s_new, Vehicle2D::Vehicle2DState s_target) override;

private:
    double maxX; 
    double maxY; 
    double maxHeadingAngle; 
    double maxSpeed; 
    double maxSteering;
};