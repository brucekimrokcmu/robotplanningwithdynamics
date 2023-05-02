#include "Vehicle3DStateSpace.hpp"

Vehicle3D::Vehicle3DState Vehicle3DStateSpace::getRandomState(){
        double x = (double)rand() / RAND_MAX * maxX;
        double y = (double)rand() / RAND_MAX * maxY;
        double z = (double)rand() / RAND_MAX * maxZ;
        double roll = (double)rand() / RAND_MAX * maxRoll;
        double pitch = (double)rand() / RAND_MAX * maxPitch;
        double yaw = (double)rand() / RAND_MAX * maxYaw;
        double phi = (double)rand() / RAND_MAX * maxPhi;
        double vx = (double)rand() / RAND_MAX * maxVx;
        double vy = (double)rand() / RAND_MAX * maxVy;
        double vz = (double)rand() / RAND_MAX * maxVz;
        double wx = (double)rand() / RAND_MAX * maxWx;
        double wy = (double)rand() / RAND_MAX * maxWy;
        double wz = (double)rand() / RAND_MAX * maxWz;
        return Vehicle3D::Vehicle3DState(x, y, z, roll, pitch, yaw, phi, vx, vy, vz, wx, wy, wz);
}

bool Vehicle3DStateSpace::nearTarget(Vehicle3D::Vehicle3DState s_new, Vehicle3D::Vehicle3DState s_target)
{
    double x_diff = s_new.x_ - s_target.x_;
    double y_diff = s_new.y_ - s_target.y_;
    double z_diff = s_new.z_ - s_target.z_;
    return (x_diff*x_diff + y_diff*y_diff + z_diff*z_diff) < constants::nearTargetThre;
}
