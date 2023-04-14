#pragma once
#include "StateSpace.hpp"
// #include "ControlSpace.hpp"



class ControlSpace{

    public: 

        struct VehicleControl 
        {
            double acc;
            double steering_rate;
        };

        double prev_error_acc;
        double prev_error_steering;
        double total_error_acc;
        double total_error_steering;

        ControlSpace(StateSpace::VehicleState s_current, StateSpace::VehicleState s_target) : total_error_acc(0), total_error_steering(0)
        {
            
        }
      
        VehicleControl PIDController(StateSpace::VehicleState s_current, StateSpace::VehicleState s_target, double dt);
        // ControlSpace::VehicleControl Control();

    private:
        StateSpace::VehicleState s_current;
        StateSpace::VehicleState s_target;

    



};