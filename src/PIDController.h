#pragma once
#include "StateSpace.h"
#include "ControlSpace.h"

class PIDController{

public: 
    PIDController(StateSpace::VehicleState s_current, StateSpace::VehicleState s_target);
    ControlSpace::VehicleControl control();
private:
    StateSpace::VehicleState s_current;
    StateSpace::VehicleState s_target;


};