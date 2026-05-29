
#include <math.h>

//#include "printf.h"
//#include "util.h"
//#include "rlib.h"
#include <stdbool.h>
#include "ahrs.h"
#include "ahrsdt0058.h"

const AHRS_Interface_t AHRS_DT0058_Interface =
{
    .AHRS_init = AHRS_init,
    .AHRS_get_roll            = AHRS_get_roll,
    .AHRS_get_pitch           = AHRS_get_pitch,
    .AHRS_get_heading         = AHRS_get_heading,
    .AHRS_get_yawRate         = AHRS_get_yawRate,
    .AHRS_set_mag_vs_gyr_prop = AHRS_set_mag_vs_gyr_prop,
    .AHRS_update              = AHRS_DT0058_update
};

int AHRS_DT0058_update(AHRS_Status_t *mstatus, Vector3f *acc, Vector3f *gyr,
                       Vector3f *mag, float deltat)
{
    /*-----------------------------------------------------------*/
    /* From Application Note DT0058 from ST Microelectronics     */
    /* Computing tilt measurement and tilt-compensated e-compass */
    /*-----------------------------------------------------------*/
    float phi = atan2f(acc->y, acc->z);
    float Gz2 = acc->y * sinf(phi) + acc->z * cosf(phi);
    float theta = atanf(-acc->x / Gz2);
    float By2 = mag->z * sinf(phi) - mag->y * cosf(phi);
    float Bz2 = mag->y * sinf(phi) + mag->z * cosf(phi);
    float Bx3 = mag->x * cosf(theta) + Bz2 * sinf(theta);
    float psi = atan2f(By2, Bx3);
    /*-----------------------------------------------------------*/

    mstatus->heading = psi;
    mstatus->roll = phi;
    mstatus->pitch = theta;

    /*-----------------------------------------------------------*\
    * Code to set other values of mstatus, not in DT0058          *
    * application note but useful for our application.            *
    \*-----------------------------------------------------------*/
    mstatus->acc = *acc;
    mstatus->gyr = *gyr;
    mstatus->mag = *mag;
    mstatus->rollRate = 0.F;
    mstatus->pitchRate = 0.F;
    mstatus->yawRate = 0.F;

    return 0;
}
