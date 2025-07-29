
// System constants
#define gyroMeasError 3.14159265358979 * (5.0f / 180.0f)// gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 3.14159265358979 * (0.2f / 180.0f)// gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError// compute beta
#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift// compute zeta

void filter_Update(float a_x, float a_y, float a_z, float w_x, float w_y, float w_z, float m_x, float m_y, float m_z, float deltat);
Quaternionf filter_Get_Quaternion();

