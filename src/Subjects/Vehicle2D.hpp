#pragma once
class Vehicle2D{
  public: 
    struct Vehicle2DState{
      double x_;     // x-position
      double y_;     // y-position
      double theta_; // orientation angle
      double v_;     // velocity
      double phi_;   // steering angle

      Vehicle2DState(){};
      Vehicle2DState(double x, double y, double theta, double v, double phi)
          : x_(x),
              y_(y),
              theta_(theta),
              v_(v),
              phi_(phi)
              {
              };
   };
    struct Vehicle2DControl 
    {
        double acc;
        double steering_rate;
    };
    Vehicle2D(double length): length(length){};
    Vehicle2DState Dynamics(const Vehicle2DState s_curr, const Vehicle2DControl u, double dt);
    Vehicle2DState Motion(const Vehicle2DState s_curr, const Vehicle2DControl u, double dt);
  private:
    double length;
};
