#include "PolargraphSolution.h"
#include "ActuatorCoordinates.h"
#include <math.h>
#include "libs/Config.h"
#include "libs/Kernel.h"
#include "StreamOutputPool.h"
#include "checksumm.h"
#include "ConfigValue.h"

#define polar_left_checksum CHECKSUM("polar_left")
#define polar_right_checksum CHECKSUM("polar_right")
#define polar_top_checksum CHECKSUM("polar_top")

#define DEBUG_PRINTF THEKERNEL->streams->printf

PolargraphSolution::PolargraphSolution(Config *config)
{
    // arm1_length is the length of the inner main arm from hinge to hinge
    polar_left = config->value(polar_left_checksum)->by_default(10.0f)->as_number();
    // arm2_length is the length of the inner main arm from hinge to hinge
    polar_right = config->value(polar_right_checksum)->by_default(210.0f)->as_number();

    polar_top = config->value(polar_top_checksum)->by_default(10.0f)->as_number();

    DEBUG_PRINTF("polar_left:%0.2f polar_right:%0.2f polar_top:%0.2f width:%0.2f", polar_left, polar_right, polar_top, (polar_right-polar_left));
}

void PolargraphSolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm) const
{
    float xmm = cartesian_mm[X_AXIS];
    float ymm = cartesian_mm[Y_AXIS];
    float zmm = cartesian_mm[Z_AXIS];
    // xmm+=221;
    // zmm+=177;

    float dx = xmm - polar_left;
    float dy = ymm - polar_top;

    actuator_mm[BETA_STEPPER] = sqrt(dx*dx+dy*dy);

    dx = polar_right - xmm;
    actuator_mm[ALPHA_STEPPER] = sqrt(dx*dx+dy*dy);

    actuator_mm[GAMMA_STEPPER] = zmm;

    DEBUG_PRINTF("c2a:%0.2f %0.2f %0.2f, %0.2f %0.2f %0.2f\n", xmm, ymm, zmm, actuator_mm[BETA_STEPPER], actuator_mm[ALPHA_STEPPER], actuator_mm[GAMMA_STEPPER]);
}

void PolargraphSolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[]) const
{
    float a = actuator_mm[BETA_STEPPER];
    float c = actuator_mm[ALPHA_STEPPER];
    float z = actuator_mm[GAMMA_STEPPER];

    float b = polar_right - polar_left;
    
    float costheta = (a * a + b * b - c * c)/2.0/a/b;

    cartesian_mm[X_AXIS] = costheta * a + polar_left;
    cartesian_mm[Y_AXIS] = sqrt( 1.0 - costheta * costheta ) * a + polar_top;
    cartesian_mm[Z_AXIS] = z;

    DEBUG_PRINTF("a2c:%0.2f %0.2f %0.2f, %0.2f %0.2f %0.2f\n", a, c, z, cartesian_mm[X_AXIS], cartesian_mm[Y_AXIS], cartesian_mm[Z_AXIS]);
}
