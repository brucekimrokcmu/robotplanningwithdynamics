// #include "StateSpace.hpp"
#include "ControlSpace.hpp"


ControlSpace::VehicleControl ControlSpace::PIDController(StateSpace::VehicleState s_current, StateSpace::VehicleState s_target)
{
    double u_acc, u_steering_rate;
    // P, I, D controls
    const double Kp_acc = 0.8;
    const double Ki_acc = 0.005;
    const double Kd_acc = 0.18;
    const double Kp_steering = 0.8;
    const double Ki_steering = 0.005;
    const double Kd_steering = 0.18;

    // Define PID errors
    double error_acc = s_target.v_ - s_current.v_;
    double error_steering = s_target.phi_ - s_current.phi_;

    // Get Previous/Total errors in acceleration and steering rate 
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

    double max_u_acc = GetMaxAcc();
    double max_u_steering = GetMaxSteering();
    // if (u_acc > max_u_acc){
    //     u_acc = max_u_acc;
    // } else if (u_acc < -max_u_acc) {
    //     u_acc = -max_u_acc;
    // }
    // if (u_steering_rate > max_u_steering) {
    //     u_steering_rate = max_u_steering;
    // } else if (u_steering_rate < -max_u_steering) {
    //     u_steering_rate = -max_u_steering;
    // }

    // Return control input
    return {u_acc, u_steering_rate};

}
        
ControlSpace::VehicleControl ControlSpace::RandomController()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> rand_acc(-0.1, 0.15);                                  // m/s^2
    std::uniform_real_distribution<> rand_steering_rate(-0.02, 0.02);                   // -30deg ~ 30 deg

    double rand_u_acc = rand_acc(gen);
    double rand_u_steering_rate = rand_steering_rate(gen);

    return {rand_u_acc, rand_u_steering_rate};
}
