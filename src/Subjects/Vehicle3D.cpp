#include "Vehicle3D.hpp"
#include <cmath>

Vehicle3D::Vehicle3DState Vehicle3D::Dynamics(const Vehicle3DState s_curr, const Vehicle3DControl u, double dt){
    double x = s_curr.x_;
    double y = s_curr.y_;
    double z = s_curr.z_;

    double roll = s_curr.roll_;
    double pitch = s_curr.pitch_;
    double yaw = s_curr.yaw_;
    
    double vx = s_curr.vx_;
    double vy = s_curr.vy_;
    double vz = s_curr.vz_;
    double wx = s_curr.wx_;
    double wy = s_curr.wy_;
    double wz = s_curr.wz_;

    double phi = s_curr.phi_;
    
    double engine_force = u.acc;
    double steering_rate = u.steering_rate; 

    const double g = constants::GRAVITY;


    double x_dot = vx*cos(yaw) + vy*sin(yaw); 
    double y_dot = vx*sin(yaw) - vy*cos(yaw);
    double z_dot = vz;
    double roll_dot = wx + wy*sin(roll)*tan(pitch) + wz*cos(roll)*tan(pitch);
    double pitch_dot = wy*cos(roll) - wz*sin(roll);
    double yaw_dot = (wy*sin(roll) + wz*cos(roll))/cos(pitch);

    double fx = engine_force*cos(phi)*cos(pitch);
    double fy = engine_force*sin(phi)*cos(pitch);
    double fz = engine_force*sin(pitch);

    double vx_dot = fx/mass - g*sin(pitch) - vy*wz + vz*wy;
    double vy_dot = fy/mass + g*sin(roll)*cos(pitch) -vx*wz + vz*wx;
    double vz_dot = fz/mass - g*cos(pitch)*cos(roll);
    double wx_dot = (Iyy - Izz)*wy*wz/Ixx;
    double wy_dot = (Izz - Ixx)*wx*wz/Iyy;
    double wz_dot = (Ixx - Iyy)*wx*wy/Izz;
    double phi_dot = steering_rate;


    Vehicle3D::Vehicle3DState s_dot = {x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot, vx_dot, vy_dot, vz_dot, wx_dot, wy_dot, wz_dot, phi_dot}; // Dynamics         

    return s_dot;
}
Vehicle3D::Vehicle3DState Vehicle3D::Motion(const Vehicle3DState s_curr, const Vehicle3DControl u, double dt){
    //Using Explicit Midpoint
    Vehicle3DState s_temp, s_new;
    Vehicle3DState f1 = Dynamics(s_curr, u, dt);

    s_temp.x_ = s_curr.x_ + 0.5 * dt * f1.x_;
    s_temp.y_ = s_curr.y_ + 0.5 * dt * f1.y_;
    s_temp.z_ = s_curr.z_ + 0.5 * dt * f1.z_;
    s_temp.roll_ = s_curr.roll_ + 0.5 * dt * f1.roll_;
    s_temp.pitch_ = s_curr.pitch_ + 0.5 * dt * f1.pitch_;
    s_temp.yaw_ = s_curr.yaw_ + 0.5 * dt * f1.yaw_;
    s_temp.vx_ = s_curr.vx_ + 0.5 * dt * f1.vx_;
    s_temp.vy_ = s_curr.vy_ + 0.5 * dt * f1.vy_;
    s_temp.vz_ = s_curr.vz_ + 0.5 * dt * f1.vz_;
    s_temp.wx_ = s_curr.wx_ + 0.5 * dt * f1.wx_;
    s_temp.wy_ = s_curr.wy_ + 0.5 * dt * f1.wy_;
    s_temp.wz_ = s_curr.wz_ + 0.5 * dt * f1.wz_;
    s_temp.phi_ = s_curr.phi_ + 0.5 * dt * f1.phi_;

    Vehicle3DState f2 = Dynamics(s_temp, u, dt);

    s_new.x_ = s_curr.x_ + dt * f2.x_;
    s_new.y_ = s_curr.y_ + dt * f2.y_;
    s_new.z_ = s_curr.z_ + dt * f2.z_;
    s_new.roll_ = s_curr.roll_ + dt * f2.roll_;
    s_new.pitch_ = s_curr.pitch_ + dt * f2.pitch_;
    s_new.yaw_ = s_curr.yaw_ +  dt * f2.yaw_;
    s_new.vx_ = s_curr.vx_ + dt * f2.vx_;
    s_new.vy_ = s_curr.vy_ + dt * f2.vy_;
    s_new.vz_ = s_curr.vz_ + dt * f2.vz_;
    s_new.wx_ = s_curr.wx_ + dt * f2.wx_;
    s_new.wy_ = s_curr.wy_ + dt * f2.wy_;
    s_new.wz_ = s_curr.wz_ + dt * f2.wz_;
    s_new.phi_ = s_curr.phi_ +  dt * f2.phi_;

    return s_new;
}