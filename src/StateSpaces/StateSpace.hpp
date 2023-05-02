#pragma once
#include <vector>
#include <limits>
#include <iostream>
#include <cmath>
#include <random>

template<typename T>
class StateSpace{
public:
    /**
     * @return random VehicleState
    */
    virtual T getRandomState() = 0;
    virtual bool nearTarget(T s_new, T s_target) = 0;
};