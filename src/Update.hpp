#pragma once
#include <cmath>
#include <vector>

#include "StateSpace.hpp"
#include "ControlSpace.hpp"

using namespace std;

class Update
{
public:
        
    struct VehicleGeometry 
    {
        const double length = 0.5; // check URDF
    } VehicleGeometry;
    Update(){};
    Update(double length, double width, double maxSpeed, double maxAcceleration, double maxSteeringAngle) {};
    StateSpace::VehicleState Dynamics(StateSpace::VehicleState s_curr, ControlSpace::VehicleControl u, double dt);
    StateSpace::VehicleState Motion(const StateSpace::VehicleState s_curr, const ControlSpace::VehicleControl u, double dt);

private:
    double mlength_;         // Length of the vehicle
};




