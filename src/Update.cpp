#include <cmath>
#include <vector>

#include "StateSpace.hpp"
#include "ControlSpace.hpp"

using namespace std;

class Update
{
    public:
        Update(double length, double width, double maxSpeed, double maxAcceleration, double maxSteeringAngle): mlength_(length){};
        // Constructor that initializes the motion space with the given parameters


        double GetLength() const
        {
            return mlength_;
        }

        StateSpace::VehicleState Motion(const StateSpace::VehicleState& s_curr, const ControlSpace::VehicleControl& u, double dt) 
        {
            double length = GetLength();  // Length of the robot
            vector<double> k1, k2, k3, k4, k5;
            // Compute k1
            k1.push_back(s_curr.v_ * cos(s_curr.theta_) * cos(s_curr.phi_));
            k1.push_back(s_curr.v_ * sin(s_curr.theta_) * cos(s_curr.phi_));
            k1.push_back(s_curr.v_ * sin(s_curr.phi_) / length);
            k1.push_back(u.acc);
            k1.push_back(u.steering_rate);

            // Compute k2
            k2.push_back((s_curr.v_ + 0.5 * k1[3]) * cos(s_curr.theta_ + 0.5 * k1[2]) * cos(s_curr.phi_ + 0.5 * k1[4]));
            k2.push_back((s_curr.v_ + 0.5 * k1[3]) * sin(s_curr.theta_ + 0.5 * k1[2]) * cos(s_curr.phi_ + 0.5 * k1[4]));
            k2.push_back((s_curr.v_ + 0.5*k1[3])*sin(s_curr.phi_ + 0.5*k1[4])/length);
            k2.push_back(u.acc);
            k2.push_back(u.steering_rate);

            // Compute k3
            k3.push_back((s_curr.v_ + 0.5 * k2[3]) * cos(s_curr.theta_ + 0.5 * k2[2]) * cos(s_curr.phi_ + 0.5 * k2[4]));
            k3.push_back((s_curr.v_ + 0.5 * k2[3]) * sin(s_curr.theta_ + 0.5 * k2[2]) * cos(s_curr.phi_ + 0.5 * k2[4]));
            k3.push_back((s_curr.v_ + 0.5 * k2[3]) * sin(s_curr.phi_ + 0.5 * k2[4]) / length);
            k3.push_back(u.acc);
            k3.push_back(u.steering_rate);

            // Compute k4
            k4.push_back((s_curr.v_ + k3[3]) * cos(s_curr.theta_ + k3[2]) * cos(s_curr.phi_ + k3[4]));
            k4.push_back((s_curr.v_ + k3[3]) * sin(s_curr.theta_ + k3[2]) * cos(s_curr.phi_ + k3[4]));
            k4.push_back((s_curr.v_ + k3[3]) * sin(s_curr.phi_ + k3[4]) / length);
            k4.push_back(u.acc);
            k4.push_back(u.steering_rate);
            
            StateSpace::VehicleState s_new;
            
            s_new.x_ = s_curr.x_ + (1.0/6.0)*(k1[0] + 2.0*k2[0] + 2.0*k3[0] + k4[0])*dt;
            s_new.y_ = s_curr.y_ + (1.0/6.0)*(k1[1] + 2.0*k2[1] + 2.0*k3[1] + k4[1])*dt; 
            s_new.theta_ = s_curr.theta_ + (1.0/6.0)*(k1[2] + 2.0*k2[2] + 2.0*k3[2] + k4[2])*dt;
            s_new.v_ = s_curr.v_ + (1.0/6.0)*(k1[3] + 2.0*k2[3] + 2.0*k3[3] + k4[3])*dt;  
            s_new.phi_ = s_curr.phi_ + (1.0/6.0)*(k1[4] + 2.0*k2[4] + 2.0*k3[4] + k4[4])*dt; 
            
            return s_new;

        }



    private:
        // const double maxTime = 100;
        double mlength_;         // Length of the vehicle
        // double width_;          // Width of the vehicle
        // double maxSpeed_;       // Maximum speed of the vehicle
        // double maxAcceleration_;// Maximum acceleration of the vehicle
        // double maxSteeringAngle;// Maximum turning radius of the vehicle


};


    // StateSpace::VehicleState Dynamics(StateSpace::VehicleState& s_curr, ControlSpace::VehicleControl& u, double dt)
    // {
    //     double x = s_curr.x_;
    //     double y = s_curr.y_;
    //     double theta = s_curr.theta_;
    //     double v = s_curr.v_;
    //     double psi = s_curr.phi_;
        
    //     double acc = u.acc;
    //     double steering_rate = u.steering_rate;

    //     double x_dot = v*cos(theta)*cos(psi);
    //     double y_dot = v*sin(theta)*sin(psi);
    //     double theta_dot = v*sin(psi)/length_;
    //     double v_dot = acc;
    //     double psi_dot = steering_rate;

    //     StateSpace::VehicleState s_dot = {x_dot, y_dot, theta_dot, v_dot, psi_dot}; // Dynamics         

    //     return s_dot;
    // }


