#pragma once

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"

#include "libs/Config.h"

class DarmSolution : public BaseSolution
{
public:
    DarmSolution(){};
    DarmSolution(Config *);
    void cartesian_to_actuator(const float millimeters[], ActuatorCoordinates &steps) const override;
    void actuator_to_cartesian(const ActuatorCoordinates &steps, float millimeters[]) const override;

private:
    float degree(float rad) const;
    
    float arm1_length;
    float arm2_length;
    float base_offset;
    float base_height;

};
