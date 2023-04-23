#ifndef CONSTANTS_H
#define CONSTANTS_H

// define your own namespace to hold constants
namespace constants
{   
    // Basic Constants
    constexpr double PI_D { 3.14159265358979323846 };

    //  ControlSpace
    constexpr double max_u_acc { 0.15 };
    constexpr double min_u_acc { -0.1 };
    constexpr double max_u_steering { 0.02 };
    constexpr double min_u_steering { -0.02 };

    // PIDController
    constexpr double Kp_acc { 0.8 };
    constexpr double Ki_acc { 0.005 };
    constexpr double Kd_acc { 0.18 };
    constexpr double Kp_steering { 0.8 };
    constexpr double Ki_steering { 0.005 };
    constexpr double Kd_steering { 0.18 };

    // Random Controller
    constexpr double rand_acc_min { -0.1 };
    constexpr double rand_acc_max { 0.15 };
    constexpr double rand_steering_min { -0.02 };
    constexpr double rand_steering_max { 0.02 };

    // WorkSpace
    constexpr double smallestExtent { 1 };
    constexpr double collisionThreshold { 0.5 };

    // StateSpace
    constexpr double nearTargetThre { 0.1 };

    // Update
    constexpr double carLength { 0.5 };
    constexpr double carWidth { 0.25 };

    // GUST
    constexpr double MAXTIME { 60000 }; // 60 seconds
    constexpr double DELTA { 0.5 }; 
    constexpr double EPSILON { 0.5 };
    constexpr double ALPHA { 8 };
    constexpr double BETA { 0.85 };
    constexpr double targetNearGoal { 0.15 };
    constexpr int    minNrSteps { 1 };
    constexpr int    maxNrSteps { 4 };
    constexpr double dt { 1.0 };
    constexpr double usePID { 0.75 };
    
    // TEST GUST
    constexpr double workSpaceMinX { 0 };
    constexpr double workSpaceMaxX { 20 };
    constexpr double workSpaceMinY { 0 };
    constexpr double workSpaceMaxY { 20 };
    constexpr double stateSpaceMaxHeading { 2*PI_D };
    constexpr double stateSpaceMaxSpeed { 1 };
    constexpr double stateSpaceMaxSteering { PI_D/6 };

    constexpr double initX { 0.0 };
    constexpr double initY { 0.0 };
    constexpr double initHeading { 0.0 };
    constexpr double initSpeed { 0.0 };
    constexpr double initSteering { 0.0 };
    constexpr double goalRegionX { 18 };
    constexpr double goalRegionY { 18 };
    constexpr double goalRegionWidth { 0.5 };

}
#endif