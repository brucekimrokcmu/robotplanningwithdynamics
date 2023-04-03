#include <vector>

class VehicleState {
public:
    VehicleState(){}
    VehicleState(double x, double y, double theta, double v, double phi) :
        x_(x), y_(y), theta_(theta), v_(v), phi_(phi){}

    // Getters and setters for the state variables
    double getX() const { return x_; }
    double getY() const { return y_; }
    double getTheta() const { return theta_; }
    double getV() const { return v_; }
    double getPhi() const {return phi_;}
    void setX(double x) { x_ = x; }
    void setY(double y) { y_ = y; }
    void setTheta(double theta) { theta_ = theta; }
    void setV(double v) { v_ = v; }
    void setPhi(double phi) { phi_ = phi; }

private:
    // State variables
    double x_;     // x-position
    double y_;     // y-position
    double theta_; // orientation angle
    double v_;     // velocity
    double phi_;   // steering angle
};


