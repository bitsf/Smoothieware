#include "DarmSolution.h"
#include "ActuatorCoordinates.h"
#include <math.h>
#include "libs/Config.h"
#include "libs/Kernel.h"
#include "StreamOutputPool.h"
#include "checksumm.h"
#include "ConfigValue.h"

#define arm1_length_checksum CHECKSUM("arm1_length")
#define arm2_length_checksum CHECKSUM("arm2_length")
#define base_offset_checksum CHECKSUM("base_offset")
#define base_height_checksum CHECKSUM("base_height")

#define DEBUG_PRINTF THEKERNEL->streams->printf

DarmSolution::DarmSolution(Config *config)
{
    // arm1_length is the length of the inner main arm from hinge to hinge
    arm1_length = config->value(arm1_length_checksum)->by_default(181.0f)->as_number();
    // arm2_length is the length of the inner main arm from hinge to hinge
    arm2_length = config->value(arm2_length_checksum)->by_default(191.0f)->as_number();

    base_offset = config->value(base_offset_checksum)->by_default(41.0f)->as_number();
    base_height = config->value(base_height_checksum)->by_default(0.0f)->as_number();
}

void DarmSolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm) const
{
    float xmm = cartesian_mm[X_AXIS];
    float ymm = cartesian_mm[Y_AXIS];
    float zmm = cartesian_mm[Z_AXIS];
    // xmm+=221;
    // zmm+=177;

    float Al = arm1_length;
    float Au = arm2_length;
    float L3 = zmm - base_height;

    float L1 = sqrt((xmm * xmm) + (ymm * ymm)) - base_offset;
    float L7 = sqrt(L1 * L1 + L3 * L3);
    float a = L3 / L7;
    float b = (L7 * L7 + Al * Al - Au * Au) / (2 * L7 * Al);
    float c = (Al * Al + Au * Au - L7 * L7) / (2 * Al * Au);

    float low = atan2(a, sqrt(1 - a * a)) + atan2(sqrt(1 - b * b), b);
    float high = atan2(sqrt(1 - c * c), c);
    high = high + low;
    float rot = atan2(ymm, xmm);

    DEBUG_PRINTF("c2a:%0.2f %0.2f %0.2f, %0.2f %0.2f %0.2f, %0.2f %0.2f %0.2f\n", xmm, ymm, zmm, high, low, rot, degree(high), degree(low), degree(rot));
    actuator_mm[ALPHA_STEPPER] = high;
    actuator_mm[BETA_STEPPER] = low;
    actuator_mm[GAMMA_STEPPER] = rot;
}

void DarmSolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[]) const
{
    float high = actuator_mm[ALPHA_STEPPER];
    float low = actuator_mm[BETA_STEPPER];
    float rot = actuator_mm[GAMMA_STEPPER];

    float zmm = sin(low) * arm1_length - sin(high) * arm2_length + base_height;

    float s = cos(low) * arm1_length - cos(high) * arm2_length + base_offset;
    float xmm = cos(rot) * s;
    float ymm = sin(rot) * s;

    DEBUG_PRINTF("a2c:%0.2f %0.2f %0.2f, %0.2f %0.2f %0.2f\n", high, low, rot, xmm, ymm, zmm);
    cartesian_mm[X_AXIS] = xmm;
    cartesian_mm[Y_AXIS] = ymm;
    cartesian_mm[Z_AXIS] = zmm;
}

float DarmSolution::degree(float rad) const
{
    return rad * 57.2957795131f;
}