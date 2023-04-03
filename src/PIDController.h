#include "VehicleState.h"
#include "VehicleMotion.h"

class PIDController{

public: 
    PIDController(VehicleState s_current, VehicleState s_target);
    VehicleMotion control();
private:
    VehicleState s_current;
    VehicleState s_target;


};