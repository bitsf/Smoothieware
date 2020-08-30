#pragma once

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"

#include "libs/Config.h"

class PolargraphSolution : public BaseSolution
{
public:
    PolargraphSolution(){};
    PolargraphSolution(Config *);
    void cartesian_to_actuator(const float millimeters[], ActuatorCoordinates &steps) const override;
    void actuator_to_cartesian(const ActuatorCoordinates &steps, float millimeters[]) const override;

private:
    float degree(float rad) const;
    
    float polar_left;
    float polar_right;
    float polar_top;

};
