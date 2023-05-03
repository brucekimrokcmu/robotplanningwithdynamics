#include "Vehicle3DControl.hpp"



Vehicle3D::Vehicle3DControl Vehicle3DControlSpace::PIDController(Vehicle3D::Vehicle3DState s_current, Vehicle3D::Vehicle3DState s_target)
{
    double u_acc, u_steering_rate;
    // P, I, D controls
    const double Kp_acc = constants::Kd_acc;
    const double Ki_acc = constants::Ki_acc;
    const double Kd_acc = constants::Kd_acc;
    const double Kp_steering = constants::Kp_steering;
    const double Ki_steering = constants::Ki_steering;
    const double Kd_steering = constants::Kd_steering;


    // Define PID errors
    double error_acc = s_target.vx_ - s_current.vx_;
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
        
Vehicle3D::Vehicle3DControl Vehicle3DControlSpace::RandomController()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> rand_acc(constants::rand_acc_min, constants::rand_acc_max);                                  // m/s^2
    std::uniform_real_distribution<> rand_steering_rate(constants::rand_steering_min, constants::rand_steering_max);                   // -30deg ~ 30 deg

    double rand_u_acc = rand_acc(gen);
    double rand_u_steering_rate = rand_steering_rate(gen);

    return {rand_u_acc, rand_u_steering_rate};
}
