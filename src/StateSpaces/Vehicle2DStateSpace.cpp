#include "Vehicle2DStateSpace.hpp"

Vehicle2D::Vehicle2DState Vehicle2DStateSpace::getRandomState() 
{
        double x = (double)rand() / RAND_MAX * maxX;
        double y = (double)rand() / RAND_MAX * maxY;
        // printf("%f, %f\n", x, y);
        double theta = (double)rand() / RAND_MAX * maxHeadingAngle;
        double v = (double)rand() / RAND_MAX * maxSpeed;
        bool isNegative = rand() % 2;
        double phi = isNegative ? -(double)rand() / RAND_MAX * maxSteering : (double)rand() / RAND_MAX * maxSteering;
        return Vehicle2D::Vehicle2DState(x, y, theta, v, phi);
}

bool Vehicle2DStateSpace::nearTarget(Vehicle2D::Vehicle2DState s_new, Vehicle2D::Vehicle2DState s_target) 
{
    double x_diff = s_new.x_ - s_target.x_;
    double y_diff = s_new.y_ - s_target.y_;
    double theta_diff = s_new.theta_ - s_target.theta_;
    double v_diff = s_new.v_ - s_target.v_;
    double phi_diff = s_new.phi_ - s_target.phi_;
    return (x_diff*x_diff + y_diff*y_diff ) < constants::nearTargetThre;
}
