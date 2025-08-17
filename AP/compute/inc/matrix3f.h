
#ifndef MATRIX3F
#define MATRIX3F

#include "geom.h"

typedef struct
{
    float a[3][3];
} Matrix3f;

extern Matrix3f Matrix3f_unit;
extern Matrix3f Matrix3f_null;

Matrix3f Matrix3f_init(float *);
Matrix3f Matrix3f_add(Matrix3f m1, Matrix3f m2);
Matrix3f Matrix3f_sub(Matrix3f m1, Matrix3f m2);
Matrix3f Matrix_mul(Matrix3f m1, Matrix3f m2);
int Matrix3f_transpose(Matrix3f mtp, Matrix3f m);
int Matrix3f_inverse(Matrix3f minv, Matrix3f m);
Vector3f Matrix_vector_mul(Matrix3f m, Vector3f v);

#endif /* MATRIX3F */