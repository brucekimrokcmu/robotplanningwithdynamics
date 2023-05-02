#pragma once
#include <vector>
#include <limits>
#include <iostream>
#include <cmath>
#include <random>

#include "StateSpace.hpp"
#include "../Constants.hpp"
#include "../Subjects/Vehicle3D.hpp"


class Vehicle3DStateSpace : public StateSpace<Vehicle3D::Vehicle3DState> {

public:
    
    Vehicle3DStateSpace(double maxX, double maxY, double maxZ, 
                        double maxRoll, double maxPitch, double maxYaw, 
                        double maxPhi, 
                        double maxVx, double maxVy, double maxVz,
                        double maxWx, double maxWy, double maxWz)
        : maxX(maxX),
          maxY(maxY),
          maxZ(maxZ),
          maxRoll(maxRoll),
          maxPitch(maxPitch),
          maxYaw(maxYaw),
          maxPhi(maxPhi),
          maxVx(maxVx),
          maxVy(maxVy),
          maxVz(maxVz),
          maxWx(maxWx),
          maxWy(maxWy),
          maxWz(maxWz)
          {
          };

    /**
     * 
     * @return random VehicleState
    */
    Vehicle3D::Vehicle3DState getRandomState() override;
    bool nearTarget(Vehicle3D::Vehicle3DState s_new, Vehicle3D::Vehicle3DState s_target) override;

private:
    double maxX; 
    double maxY; 
    double maxZ;
    double maxRoll;
    double maxPitch;
    double maxYaw;
    double maxPhi;
    double maxVx;
    double maxVy;
    double maxVz;
    double maxWx;
    double maxWy;
    double maxWz;

};