#pragma once
#include <cmath>
#include <vector>

#include "StateSpace.hpp"
#include "ControlSpace.hpp"
#include "Constants.hpp"

using namespace std;

class Update
{
public:
        
    struct VehicleGeometry 
    {
        const double length = constants::carLength; // check URDF
    } VehicleGeometry;
    Update(){};
    // Update(double length, double width, double maxSpeed, double maxAcceleration, double maxSteeringAngle) {};
    StateSpace::VehicleState Dynamics(StateSpace::VehicleState s_curr, ControlSpace::VehicleControl u, double dt);
    StateSpace::VehicleState Motion(const StateSpace::VehicleState s_curr, const ControlSpace::VehicleControl u, double dt);

private:
    double mlength_;         // Length of the vehicle
};



// x_dot = vx_ * cos(theta_) * cos(phi_) - vy_ * sin(theta_) * cos(phi_)
// y_dot = vx_ * sin(theta_) * cos(phi_) + vy_ * cos(theta_) * cos(phi_)
// theta_dot = (wx_ * sin(theta_) + wy_ * cos(theta_)) / cos(phi_)
// vx_dot = (u_f / m - (vz_ * wy_ - vy_ * wz_) / m) - g * sin(theta_)
// vy_dot = ((u_f * phi_dot) / m + (vz_ * wx_ - vx_ * wz_) / m) * cos(phi_) - g * cos(theta_) * sin(phi_)
// vz_dot = ((u_f * phi_dot) / m + (vy_ * wx_ - vx_ * wy_) / m) * sin(phi_) + g * cos(theta_) * cos(phi_)
// wx_dot = ((Iyy_ - Izz_) * wy_ * wz_ + T_phi_) / Ixx_
// wy_dot = ((Izz_ - Ixx_) * wz_ * wx_ + T_theta_) / Iyy_
// wz_dot = ((Ixx_ - Iyy_) * wx_ * wy_ + T_psi_) / Izz_
// phi_dot = phi_dot_ # phi_dot is a control input
