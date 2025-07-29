
void MadgwickAHRSupdate(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz,
                        float deltaT);

extern float q0, q1, q2, q3;


int AHRS_compute(Vector3f *accel, Vector3f *mag, Vector3f *gyro, float deltat, 
    float *heading, float *roll, float *pitch);