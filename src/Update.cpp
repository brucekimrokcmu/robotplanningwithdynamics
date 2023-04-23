#include <cmath>
#include <vector>
#include "Update.hpp"


using namespace std;

StateSpace::VehicleState Update::Dynamics(StateSpace::VehicleState s_curr, ControlSpace::VehicleControl u, double dt)
{
    
    double x = s_curr.x_;
    double y = s_curr.y_;
    double theta = s_curr.theta_;
    double v = s_curr.v_;
    double psi = s_curr.phi_;
    
    double acc = u.acc;
    double steering_rate = u.steering_rate;

    double x_dot = v*cos(theta)*cos(psi);
    double y_dot = v*sin(theta)*cos(psi);
    double theta_dot = v*sin(psi)/VehicleGeometry.length;
    double v_dot = acc;
    double psi_dot = steering_rate;

    StateSpace::VehicleState s_dot = {x_dot, y_dot, theta_dot, v_dot, psi_dot}; // Dynamics         

    return s_dot;
}

StateSpace::VehicleState Update::Motion(const StateSpace::VehicleState s_curr, const ControlSpace::VehicleControl u, double dt) 
{
    //Using Explicit Midpoint
    StateSpace::VehicleState s_temp, s_new;
    StateSpace::VehicleState f1 = Dynamics(s_curr, u, dt);

    s_temp.x_ = s_curr.x_ + 0.5 * dt * f1.x_;
    s_temp.y_ = s_curr.y_ + 0.5 * dt * f1.y_;
    s_temp.theta_ = s_curr.theta_ + 0.5 * dt * f1.theta_;
    s_temp.v_ = s_curr.v_ + 0.5 * dt * f1.v_;
    s_temp.phi_ = s_curr.phi_ + 0.5 * dt * f1.phi_;

    StateSpace::VehicleState f2 = Dynamics(s_temp, u, dt);

    s_new.x_ = s_curr.x_ + dt * f2.x_;
    s_new.y_ = s_curr.y_ + dt * f2.y_;
    s_new.theta_ = s_curr.theta_ + dt * f2.theta_;
    s_new.v_ = s_curr.v_ + dt * f2.v_;
    s_new.phi_ = s_curr.phi_ + dt * f2.phi_;

    return s_new;

}



