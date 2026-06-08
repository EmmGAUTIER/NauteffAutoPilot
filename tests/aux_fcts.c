#include <math.h>

int cmpFloats(float a, float b)
{
    float mag;
    float diff;

    mag = fabs(a) +fabs(b);
    diff = fabs(a - b);

    if (mag == 0.0)
        return 1; /* both are zero */
    else
        if (diff/mag < 2.0e-5)
            return 1; /* they are nearly equal */
        else
            return 0; /* they are significantly different */
}
