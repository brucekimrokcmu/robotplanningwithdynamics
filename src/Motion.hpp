#pragma once
#include <cmath>
#include <vector>


#include "StateSpace.hpp"
#include "ControlSpace.hpp"

using namespace std;

class MotionSpace : StateSpace, ControlSpace
{
    public:
        MotionSpace(double length, double width, double maxSpeed, double maxAcceleration, double maxSteeringAngle);
        // Constructor that initializes the motion space with the given parameters
          
        StateSpace::VehicleState Dynamics(StateSpace::VehicleState& s_curr, ControlSpace& u)
        {
            double x = s_curr.x_;
            double y = s_curr.y_;
            double theta = s_curr.theta_;
            double v = s_curr.v_;
            double psi = s_curr.psi_;
            
            double acc = u[0];
            double steering_rate = u[1];

            double x_dot = v*cos(theta)*cos(psi);
            double y_dot = v*sin(theta)*sin(psi);
            double theta_dot = v*sin(psi)/length_;
            double v_dot = acc;
            double psi_dot = steering_rate;

            StateSpace::VehicleState s_dot = {x_dot, y_dot, theta_dot, v_dot, psi_dot};

            return s_dot;
        }


        StateSpace motion(const StateSpace& s, const ControlSpace& u, double dt) 
        {
            double L = length_;  // Length of the robot

            // Compute k1
            StateSpace k1;
            k1[0] = s[3]*cos(s[2])*cos(s[4]);
            k1[1] = s[3]*sin(s[2])*sin(s[4]);
            k1[2] = s[3]*sin(s[4])/L;
            k1[3] = u[0];
            k1[4] = u[1];

            // Compute k2
            StateSpace k2;
            k2[0] = (s[3] + 0.5*k1[3])*cos(s[2] + 0.5*k1[2])*cos(s[4] + 0.5*k1[4]);
            k2[1] = (s[3] + 0.5*k1[3])*sin(s[2] + 0.5*k1[2])*sin(s[4] + 0.5*k1[4]);
            k2[2] = (s[3] + 0.5*k1[3])*sin(s[4] + 0.5*k1[4])/L;
            k2[3] = u[0];
            k2[4] = u[1];

            // Compute k3
            StateSpace k3;
            k3[0] = (s[3] + 0.5*k2[3])*cos(s[2] + 0.5*k2[2])*cos(s[4] + 0.5*k2[4]);
            k3[1] = (s[3] + 0.5*k2[3])*sin(s[2] + 0.5*k2[2])*sin(s[4] + 0.5*k2[4]);
            k3[2] = (s[3] + 0.5*k2[3])*sin(s[4] + 0.5*k2[4])/L;
            k3[3] = u[0];
            k3[4] = u[1];

            // Compute k4
            StateSpace k4;
            k4[0] = (s[3] + k3[3])*cos(s[2] + k3[2])*cos(s[4] + k3[4]);
            k4[1] = (s[3] + k3[3])*sin(s[2] + k3[2])*sin(s[4] + k3[4]);
            k4[2] = (s[3] + k3[3])*sin(s[4] + k3[4])/L;
            k4[3] = u[0];
            k4[4] = u[1];

            StateSpace s_new;
            for (int i = 0; i < 5; i++) {
                s_new[i] = s[i] + (1.0/6.0)*(k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i])*dt;
            }

            return s_new;

        }



    private:
        const double maxTime = 100;
        double length_;         // Length of the vehicle
        double width_;          // Width of the vehicle
        double maxSpeed_;       // Maximum speed of the vehicle
        double maxAcceleration_;// Maximum acceleration of the vehicle
        double maxSteeringAngle;// Maximum turning radius of the vehicle


};



/*

state-space s = {x, y, theta, v, psi}

1st-order ODE
s' = f(s, u)
    = [x', y', theta', v', psi']
    = [v*cos(theta)*cos(psi), v*sin(theta)*sin(psi), v*sin(psi)/length, acc, steering_rate]


To predict the system's motion, q and q' are needed. 
Combining this to state vector

s = [q; q'] = [x, y, theta, v, psi].T

s' = [q';q''] = [x', y', theta', acc, steering_rate].T
   

q'' = f(q, q')
[acc;steering_rate] = f(x, y, theta, v, steering_rate)


It's a non linear function
Therefore, we would have to use numerical integrator 
such as Runge-Kutta to linearize about a point and integrate it from initial point (s_init).




*/

/*
Required Library:

Eigen 

*/