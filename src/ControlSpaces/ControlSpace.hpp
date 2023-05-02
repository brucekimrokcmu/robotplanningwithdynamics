#pragma once
#include "../StateSpaces/StateSpace.hpp"
#include "../Constants.hpp"

template<typename State, typename Control>
class ControlSpace{
    public: 
        virtual Control PIDController(State s_current, State s_target) = 0;
        virtual Control RandomController() = 0;

    private:
        State s_current;
        State s_target;

};