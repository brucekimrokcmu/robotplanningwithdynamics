// #include "StateSpace.hpp"
#include "ControlSpace.hpp"


ControlSpace::VehicleControl ControlSpace::PIDController(StateSpace::VehicleState s_current, StateSpace::VehicleState s_target)
{
    double u_acc, u_steering_rate;
    // P, I, D controls
    const double Kp_acc = 10.0;
    const double Ki_acc = 0.1;
    const double Kd_acc = 0.1;
    const double Kp_steering = 10.0;
    const double Ki_steering = 0.1;
    const double Kd_steering = 0.1;

    // Define PID errors
    double error_acc = s_target.v_ - s_current.v_;
    double error_steering = s_target.phi_ - s_current.phi_;

    //
    double total_error_ecc = GetTotalErrorAcc();
    double prev_error_acc = GetPrevErrorAcc();
    double total_error_steering = GetTotalErrorSteering();
    double prev_error_steering = GetPrevErrorSteering();

    // Calculate PID components
    double P_acc = Kp_acc * error_acc;
    double I_acc = Ki_acc * (total_error_ecc);
    double D_acc = Kd_acc * (error_acc - prev_error_acc);

    double P_steering = Kp_steering * error_steering;
    double I_steering = Ki_steering * (total_error_steering);
    double D_steering = Kd_steering * (error_steering - prev_error_steering);
    

    u_acc = P_acc + I_acc + D_acc;
    
    
    u_steering_rate = P_steering + I_steering + D_steering;

    // Update previous errors
    SetPrevErrorAcc(error_acc);
    SetPrevErrorSteering(error_steering);
    SetTotalErrorAcc(total_error_ecc + error_acc);
    SetTotalErrorSteering(total_error_steering + error_steering);

    // Return control input
    return {u_acc, u_steering_rate};

}
        
ControlSpace::VehicleControl ControlSpace::RandomController()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> rand_acc(0.1, 7);                                  // m/s^2
    std::uniform_real_distribution<> rand_steering_rate(-PI/6, PI/6);                   // -30deg ~ 30 deg

    double rand_u_acc = rand_acc(gen);
    double rand_u_steering_rate = rand_steering_rate(gen);

    return {rand_u_acc, rand_u_steering_rate};
}
