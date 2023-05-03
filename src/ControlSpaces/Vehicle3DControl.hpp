#pragma once
#include "ControlSpace.hpp"
#include "../Subjects/Vehicle3D.hpp"



class Vehicle3DControlSpace : public ControlSpace<Vehicle3D::Vehicle3DState, Vehicle3D::Vehicle3DControl>{

    public: 

        Vehicle3DControlSpace() 
        :   mtotal_error_acc(0), 
            mtotal_error_steering(0),
            mprev_error_acc(0),
            mprev_error_steering(0),
            max_u_acc(constants::max_u_acc),
            min_u_acc(constants::min_u_acc),
            max_u_steering(constants::max_u_steering),
            min_u_steering(constants::min_u_steering) 
        {
        }

        double GetTotalErrorAcc() const { return mtotal_error_acc; };
        double GetTotalErrorSteering() const { return mtotal_error_steering; };
        double GetPrevErrorAcc() const { return mprev_error_acc; };
        double GetPrevErrorSteering() const { return mprev_error_steering; };
        double GetMaxAcc() const {return max_u_acc;};
        double GetMaxSteering() const {return max_u_steering;};

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

        Vehicle3D::Vehicle3DControl PIDController(Vehicle3D::Vehicle3DState s_current, Vehicle3D::Vehicle3DState s_target) override;
        Vehicle3D::Vehicle3DControl RandomController() override;

    private:

        double mprev_error_acc;
        double mprev_error_steering;
        double mtotal_error_acc;
        double mtotal_error_steering;

        double max_u_acc;
        double min_u_acc;
        double max_u_steering;
        double min_u_steering;
};