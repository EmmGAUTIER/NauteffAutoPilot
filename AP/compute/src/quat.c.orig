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
#if 1
#include <math.h>
#include <stdint.h>
#include "quat.h"

Quaternionf Quaternionf_unit = {1.0F, .0F, .0F, .0F};

Quaternionf Quaternionf_init(float q, float x, float y, float z)
{
    Quaternionf v;
    v.w = q;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

Quaternionf Quaternionf_add(Quaternionf v1, Quaternionf v2)
{
    Quaternionf result = {v1.w + v2.w, v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
    return result;
}

Quaternionf Quaternionf_sub(Quaternionf v1, Quaternionf v2)
{
    Quaternionf result = {v1.w - v2.w, v1.x - v2.x, v1.y - v2.y, v1.z - v2.z};
    return result;
}

Quaternionf Quaternionf_mul(Quaternionf v1, Quaternionf v2)
{
    Quaternionf result = {
        v1.w * v2.w - v1.x * v2.x - v1.y * v2.y - v1.z * v2.z,
        v1.w * v2.x + v1.x * v2.w + v1.y * v2.z - v1.z * v2.y,
        v1.w * v2.y - v1.x * v2.z + v1.y * v2.w + v1.z * v2.x,
        v1.w * v2.z + v1.x * v2.y - v1.y * v2.x + v1.z * v2.w};
    return result;
}

float Quaternionf_getDotProduct(Quaternionf v1, Quaternionf v2)
{
    return v1.w * v2.w + v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

Quaternionf Quaternionf_getScaled(Quaternionf v, float a)
{
    Quaternionf result = {v.w * a, v.x * a, v.y * a, v.z * a};
    return result;
}

float Quaternionf_getNorm(Quaternionf v)
{
    return sqrtf(v.w * v.w + v.x * v.x + v.y * v.y + v.z * v.z);
}

Quaternionf Quaternionf_getNormalized(Quaternionf v)
{
    float norm = Quaternionf_getNorm(v);
    if (norm == 0.0F)
    {
        Quaternionf zero = {0.0F, 0.0F, 0.0F, 0.0F};
        return zero;
    }
    return Quaternionf_getScaled(v, 1.0f / norm);
}

#endif