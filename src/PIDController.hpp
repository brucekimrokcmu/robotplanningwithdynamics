#pragma once
#include "StateSpace.hpp"
#include "ControlSpace.hpp"

class PIDController{

    public: 
        PIDController(StateSpace::VehicleState s_current, StateSpace::VehicleState s_target);
        ControlSpace::VehicleControl Control();

    private:
        StateSpace::VehicleState s_current;
        StateSpace::VehicleState s_target;

    



};