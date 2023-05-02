class Vehicle3D{
public:
    struct Vehicle3DState{
        double x_;
        double y_;
        double z_;
        double roll_;   // roll
        double pitch_;   // pitch
        double yaw_;   // yaw
        double phi_; // steering angle 
        double vx_;  // linear velocithy
        double vy_;
        double vz_;
        double wx_;  // angular velocity
        double wy_;
        double wz_;

        Vehicle3DState(){};
        Vehicle3DState(double x, double y, double z, 
                            double roll, double pitch, double yaw, double phi,
                            double vx, double vy, double vz,
                            double wx, double wy, double wz)
            : x_(x),
              y_(y),
              z_(z),
              roll_(roll),
              pitch_(pitch),
              yaw_(yaw),
              phi_(phi),
              vx_(vx),
              vy_(vy),
              vz_(vz),
              wx_(wx),
              wy_(wy),
              wz_(wz)

            {
            };

    };
    struct Vehicle3DControl 
    {
        //TODO: Implement control for 3D vehicle
    };
    Vehicle3D(double length): length(length){}; //TODO: add other nessaary parameters
    Vehicle3DState Dynamics(const Vehicle3DState s_curr, const Vehicle3DControl u, double dt);
    Vehicle3DState Motion(const Vehicle3DState s_curr, const Vehicle3DControl u, double dt);
  private:
    double length;
    //TODO: add other nessaary parameters
};