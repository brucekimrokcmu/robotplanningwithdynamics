#include <vector>

class ControlSpace {
public:
    ControlSpace( double maxAcceleration, double maxSteeringrate)
        : maxAcceleration(maxAcceleration), maxSteeringRate(maxSteeringRate){};
    // Constructor that initializes the motion space with the given parameters
    
    bool isValidControl(double acc, double ste) const;
    // Returns true if the given state (x, y, theta) is valid within the motion space, false otherwise
    
    // std::vector<double> getNextValidState(double x, double y, double theta, double velocity, double steeringAngle, double timeStep) const;
    // Returns the next valid state of the vehicle given the current state (x, y, theta), velocity, steering angle, and time step,
    // within the constraints of the motion space
    
private:
    // double length_;         // Length of the vehicle
    // double width_;          // Width of the vehicle
    // double maxSpeed_;       // Maximum speed of the vehicle
    double maxAcceleration;// Maximum acceleration of the vehicle
    double maxSteeringRate;// Maximum turning radius of the vehicle
    
    // Helper functions for checking validity of states and calculating next states within the motion space
    // bool isValidPosition(double x, double y) const;
    // bool isValidHeading(double theta) const;
    bool isValidAcc(double velocity) const;
    bool isValidSteeringRate(double steeringRate) const;
    // bool isValidTimeStep(double timeStep) const;
    // double calculateTurningRadius(double steeringAngle) const;
    // double calculateMaxTurningSpeed(double turningRadius) const;
    // double calculateNextX(double x, double velocity, double theta, double timeStep) const;
    // double calculateNextY(double y, double velocity, double theta, double timeStep) const;
    // double calculateNextTheta(double theta, double velocity, double steeringAngle, double timeStep) const;
};