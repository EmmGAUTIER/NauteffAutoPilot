
void madgwick_init(float ax, float ay, float az, float mx, float my, float mz);

void madgwick_update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz,
                     float dt);

Quaternionf madgwick_get_quaternion();
void madgwick_get_Euler_angles(float *roll, float *pitch, float *yaw);
float madgwick_get_roll();
float madgwick_get_pitch();
float madgwick_get_yaw();


void madgwick_set_beta(float new_beta);
void madgwick_set_zeta(float new_zeta);