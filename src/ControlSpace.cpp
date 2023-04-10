#pragma once
#include "StateSpace.hpp"
#include "ControlSpace.hpp"



ControlSpace::VehicleControl ControlSpace::PIDController(StateSpace::VehicleState s_current, StateSpace::VehicleState s_target, double dt)
{
    const double Kp_acc = 10.0;
    const double Ki_acc = 0.1;
    const double Kd_acc = 0.1;
    const double Kp_steering = 10.0;
    const double Ki_steering = 0.1;
    const double Kd_steering = 0.1;

    // Define PID errors
    double error_acc = s_target.v_ - s_current.v_;
    double error_steering = s_target.phi_ - s_current.phi_;

    // Calculate PID components
    double P_acc = Kp_acc * error_acc;
    double I_acc = Ki_acc * (error_acc + prev_error_acc) * dt;
    double D_acc = Kd_acc * (error_acc - prev_error_acc) / dt;
    double u_acc = P_acc + I_acc + D_acc;

    double P_steering = Kp_steering * error_steering;
    double I_steering = Ki_steering * (error_steering + prev_error_steering) * dt;
    double D_steering = Kd_steering * (error_steering - prev_error_steering) / dt;
    double u_steering_rate = P_steering + I_steering + D_steering;

    // Update previous errors
    prev_error_acc = error_acc;
    prev_error_steering = error_steering;

    // Return control input
    return {u_acc, u_steering_rate};

}
        




