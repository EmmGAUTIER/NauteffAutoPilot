/*
MIT License

Copyright (c) 2025 Emmanuel Gautier / Nauteff

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef GEOM_H
#define GEOM_H

#ifndef M_PI
#define M_PI 3.14159265358979323846F
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif
#ifndef M_PI_4
#define M_PI_4 0.78539816339744830962
#endif

#ifndef M_PIF
#define M_PIF 3.1415927f
#endif
#ifndef M_PI_2F
#define M_PI_2F 1.5707963f
#endif
#ifndef M_PI_4F
#define M_PI_4F 0.7853982f
#endif

/**
 * @brief Structure representing a 3D vector with float components.
 *  componants are named x, y and z
 */
typedef struct
{
    float x; /**< X component */
    float y; /**< Y component */
    float z; /**< Z component */
} Vector3f;
/*
 * @brief Unity 3D vectors
 * those vectors are used to determine directions of vectors
 */

extern Vector3f unitxf;
extern Vector3f unityf;
extern Vector3f unitzf;

extern Vector3f unitmxf;
extern Vector3f unitmyf;
extern Vector3f unitmzf;

extern Vector3f Vector3f_null;

/*
 * @brief Initializes a 3D vector with given components.
 * @param x, y, z The components of the vector.
 * @return The Vector3f structure initialized with the given components.
 */
Vector3f vector3f_init(float x, float y, float z);

/**
 * @brief Adds two 3D vectors component-wise.
 *
 * @param v1 The first vector.
 * @param v2 The second vector.
 * @return The component-wise sum of v1 and v2.
 */
Vector3f vector3f_add(Vector3f v1, Vector3f v2);

/**
 * @brief Subtracts the second 3D vector from the first, component-wise.
 *
 * @param v1 The vector to subtract from.
 * @param v2 The vector to subtract.
 * @return The component-wise difference v1 - v2.
 */
Vector3f vector3f_sub(Vector3f v1, Vector3f v2);

/**
 * @brief Computes the cross product of two 3D vectors.
 *
 * The cross product results in a vector perpendicular to both input vectors.
 *
 * @param v1 The first vector.
 * @param v2 The second vector.
 * @return The cross product vector.
 */
Vector3f vector3f_getCrossProduct(Vector3f v1, Vector3f v2);

/**
 * @brief Computes the dot product (scalar product) of two 3D vectors.
 *
 * The dot product is a scalar value equal to the sum of the products of the corresponding components.
 *
 * @param v1 The first vector.
 * @param v2 The second vector.
 * @return The scalar dot product.
 */
float vector3f_getDotProduct(Vector3f v1, Vector3f v2);

/**
 * @brief Multiplies a 3D vector by a scalar.
 *
 * Each component of the vector is multiplied by the scalar value.
 *
 * @param v The input vector.
 * @param a The scalar value.
 * @return The scaled vector.
 */
Vector3f vector3f_getScaled(Vector3f v, float a);

/**
 * @brief Computes the Euclidean norm (length) of a 3D vector.
 *
 * The norm is calculated as sqrt(x^2 + y^2 + z^2).
 *
 * @param v The input vector.
 * @return The Euclidean norm of the vector.
 */
float vector3f_getNorm(Vector3f v);

/**
 * @brief Normalizes a 3D vector to have a length of 1.
 *
 * If the vector has zero length, the zero vector is returned.
 *
 * @param v The input vector.
 * @return The normalized vector (unit vector), or the zero vector if the input is zero.
 */
Vector3f vector3f_getNormalized(Vector3f v);

/*
 @ brief Computes the difference between two 3D vectors and checks if it is within a threshold.
 @ param v1 The first vector.
 @ param v2 The second vector.
 @ param threshold The threshold value.
 @ return true if the norm of the difference is less than or equal to the threshold, false otherwise.
 */

// bool Vector3f_normDiff(Vector3f v1, Vector3f v2, float threshold);

/*
 * @brief Computes the cosine of the angle between two 3D vectors.
 *  The cosine is calculated as the dot product of the vectors divided by the product of their norms.
 *  If either vector has zero length, the function returns 0.0.
 *  The cosine is usually used to know how two vectors are aligned.
 * @param v1 The first vector.
 * @param v2 The second vector.
 * @return The cosine of the angle between the two vectors.
 *         Returns 0.0 if either vector has zero length.
 *
 */
float vector3f_getCosine(Vector3f v1, Vector3f v2);

float cvt_dir_deg_rad(float angle);
float cvt_dir_rad_deg(float angle);

float cvt_dir_norm_rad(float angle);
float cvt_dir_norm_deg(float angle);

/**
 * @brief Structure representing a Quaternion with float components.
 *  componants are named x, y z and w
 */
typedef struct
{
    float x;
    float y;
    float z;
    float w;
} Quaternionf;

#endif /* GEOM_H */
