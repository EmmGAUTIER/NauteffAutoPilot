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

#include "math.h"
#include <geom.h>

int __errno = 0; /* needed by math functions */

Vector3f Vector3f_null = {.0F, .0F, .0F};

Vector3f unitxf = { 1.0F,  0.0F,  0.0F};
Vector3f unityf = { 0.0F,  1.0F,  0.0F};
Vector3f unitzf = { 0.0F,  0.0F,  1.0F};

Vector3f unitmxf = {-1.0F,  0.0F,  0.0F};
Vector3f unitmyf = { 0.0F, -1.0F,  0.0F};
Vector3f unitmzf = { 0.0F,  0.0F, -1.0F};

Vector3f vector3f_init(float x, float y, float z) {
    Vector3f v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

Vector3f vector3f_add(Vector3f v1, Vector3f v2) {
    Vector3f result = { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
    return result;
}

Vector3f vector3f_sub(Vector3f v1, Vector3f v2) {
    Vector3f result = { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
    return result;
}

Vector3f vector3f_getCrossProduct(Vector3f v1, Vector3f v2) {
    Vector3f result = {
        v1.y * v2.z - v1.z * v2.y,
        v1.z * v2.x - v1.x * v2.z,
        v1.x * v2.y - v1.y * v2.x
    };
    return result;
}

float vector3f_getDotProduct(Vector3f v1, Vector3f v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

Vector3f vector3f_getScaled(Vector3f v, float a) {
    Vector3f result = { v.x * a, v.y * a, v.z * a };
    return result;
}

float vector3f_getNorm(Vector3f v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vector3f vector3f_getNormalized(Vector3f v) {
    float norm = vector3f_getNorm(v);
    if (norm == 0.0f) {
        Vector3f zero = {0.0f, 0.0f, 0.0f};
        return zero;
    }
    return vector3f_getScaled(v, 1.0f / norm);
}

float vector3f_getCosine(Vector3f v1, Vector3f v2) {
    float dot = vector3f_getDotProduct(v1, v2);
    float norm1 = vector3f_getNorm(v1);
    float norm2 = vector3f_getNorm(v2);

    if (norm1 == 0.0f || norm2 == 0.0f) {
        return 0.0f; // Handle zero-length vectors
    }
    return dot / (norm1 * norm2);
}


float cvt_dir_norm_rad (float angle)
{

    float anglen;

    anglen = fmodf(angle, 2.0F * M_PIF);

    if (anglen < 0.0F) {
        anglen += 2.0F *  M_PIF;
    }   

    return anglen;
}

float cvt_dir_norm_deg (float angle)
{

    float anglen;

    anglen = fmodf(angle, 360.0F);

    if (anglen < 0.0F) {
        anglen += 360.0F;
    }
    
    return anglen;
}

/* Conversion : degrés horaires depuis le nord -> radians antihoraires depuis l'Est */
float cvt_dir_deg_rad (float angle)
{
    float anglerad = M_PI/2.0 - (M_PI/180.0) * angle;

    if (anglerad >= 0.0) {
        anglerad = fmodf(anglerad, 2.0 * M_PI);
    } else {
        anglerad = 2.0 * M_PI + fmodf(anglerad, 2.0 * M_PI);
    }

    return anglerad;
}

// Conversion : radians antihoraires depuis l'Est -> degrés horaires depuis le nord
float cvt_dir_rad_deg (float angle)
{
    float angledeg = 90.0 - (180.0/M_PI) * angle;

    if (angledeg >= 0.0) {
        angledeg = fmodf(angledeg, 360.0);
    } else {
        angledeg = 360.0 + fmodf(angledeg, 360.0);
    }

    return angledeg;
}

Vector3f get_unit_vector_x()
{
    Vector3f v;
    v.x = 1.0F;
    v.y = 0.0F;
    v.z = 0.0F;
    return v;
}