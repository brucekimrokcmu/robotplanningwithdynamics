#pragma once
#include <vector>

class MotionSpace {
public:
    MotionSpace(double length, double width, double maxSpeed, double maxAcceleration, double maxTurningRadius);
    // Constructor that initializes the motion space with the given parameters
    
    bool IsValidState(double x, double y, double theta) const;
    // Returns true if the given state (x, y, theta) is valid within the motion space, false otherwise
    
    std::vector<double> GetNextValidState(double x, double y, double theta, double velocity, double steeringAngle, double timeStep) const;
    // Returns the next valid state of the vehicle given the current state (x, y, theta), velocity, steering angle, and time step,
    // within the constraints of the motion space
    
private:
    double length_;         // Length of the vehicle
    double width_;          // Width of the vehicle
    double maxSpeed_;       // Maximum speed of the vehicle
    double maxAcceleration_;// Maximum acceleration of the vehicle
    double maxTurningRadius_;// Maximum turning radius of the vehicle
    
    // Helper functions for checking validity of states and calculating next states within the motion space
    bool IsValidPosition(double x, double y) const;
    bool IsValidHeading(double theta) const;
    bool IsValidVelocity(double velocity) const;
    bool IsValidSteeringAngle(double steeringAngle) const;
    bool IsValidTimeStep(double timeStep) const;
    double CalculateTurningRadius(double steeringAngle) const;
    double CalculateMaxTurningSpeed(double turningRadius) const;
    double CalculateNextX(double x, double velocity, double theta, double timeStep) const;
    double CalculateNextY(double y, double velocity, double theta, double timeStep) const;
    double CalculateNextTheta(double theta, double velocity, double steeringAngle, double timeStep) const;
};