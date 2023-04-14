#pragma once
#include "StateSpace.hpp"

#define PI 3.14159265358979323846  /* pi */


class ControlSpace{

    public: 

        struct VehicleControl 
        {
            double acc;
            double steering_rate;
        };

        ControlSpace() 
        :   mtotal_error_acc(0), 
            mtotal_error_steering(0),
            mprev_error_acc(0),
            mprev_error_steering(0),
            max_u_acc(7),
            min_u_acc(0.1),
            max_u_steering(PI/6),
            min_u_steering(-PI/6) 
        {
        }

        double GetTotalErrorAcc() const { return mtotal_error_acc; };
        double GetTotalErrorSteering() const { return mtotal_error_steering; };
        double GetPrevErrorAcc() const { return mprev_error_acc; };
        double GetPrevErrorSteering() const { return mprev_error_steering; };

        void SetTotalErrorAcc(const double total_error_acc) {
            mtotal_error_acc = total_error_acc;
        };
        void SetTotalErrorSteering(const double total_error_steering) {
            mtotal_error_steering = total_error_steering;
        }
        void SetPrevErrorAcc(const double prev_error_acc) {
            mprev_error_acc = prev_error_acc;
        };
        void SetPrevErrorSteering(const double prev_error_steering) {
            mprev_error_steering = prev_error_steering;
        };

        VehicleControl PIDController(StateSpace::VehicleState s_current, StateSpace::VehicleState s_target);
        VehicleControl RandomController();

    private:
        StateSpace::VehicleState s_current;
        StateSpace::VehicleState s_target;

        double mprev_error_acc;
        double mprev_error_steering;
        double mtotal_error_acc;
        double mtotal_error_steering;

        double max_u_acc;
        double min_u_acc;
        double max_u_steering;
        double min_u_steering;
};