

#include "matrix3f.h"

Matrix3f Matrix3f_unit = {
    .a = {
        {1.0F, 0.0F, 0.0F},
        {0.0F, 1.0F, 0.0F},
        {0.0F, 0.0F, 1.0F}}};

Matrix3f Matrix3f_null = {
    .a = {
        {0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F}}};

Matrix3f Matrix3f_init(float *)
{
    Matrix3f m;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            m.a[i][j] = 0.0F;
        }
    }
    return m;
}

/*
 *  @brief Adds two 3x3 matrices.
 *
 *  The result is a new matrix where each element is the sum
 *  of the elements of the input matrices.
 *
 * @param m1 The first matrix.
 * @param m2 The second matrix.
 * @return The resulting matrix m1 + m2.
 */
Matrix3f Matrix3f_add(Matrix3f m1, Matrix3f m2)
{
    Matrix3f result;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result.a[i][j] = m1.a[i][j] + m2.a[i][j];
        }
    }
    return result;
}

/*
 * @brief Subtracts the second 3x3 matrix from the first.
 *  The result is a new matrix where each element is the difference of the corresponding elements of the input matrices.
 *
 * @param m1 The first matrix.
 * @param m2 The second matrix.
 * @return The resulting matrix  m1 - m2.
 */
Matrix3f Matrix3f_sub(Matrix3f m1, Matrix3f m2)
{
    Matrix3f result;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result.a[i][j] = m1.a[i][j] - m2.a[i][j];
        }
    }
    return result;
}

/*
 * @brief Multiplies two 3x3 matrices.
 *  The result is a new matrix where each element is computed as
 *  the dot product of the corresponding row of the first matrix
 *  and the corresponding column of the second matrix.
 * @param m1 The first matrix.
 * @param m2 The second matrix.
 * @return The resulting matrix m1 * m2.
 */
Matrix3f Matrix_mul(Matrix3f m1, Matrix3f m2)
{
    Matrix3f result = Matrix3f_null;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result.a[i][j] = m1.a[i][0] * m2.a[0][j] +
                             m1.a[i][1] * m2.a[1][j] +
                             m1.a[i][2] * m2.a[2][j];
        }
    }
    return result;
}

/*
 * @brief Transposes a 3x3 matrix.
 * The transpose of a matrix is obtained by swapping its rows and columns.
 * @param mtp The matrix to store the transposed result.
 * @param m The matrix to be transposed.
 * @return 0 on success, or an error code if the operation fails.
 */
int Matrix3f_transpose(Matrix3f mtp, Matrix3f m)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            mtp.a[i][j] = m.a[j][i];
        }
    }
    return 0; // Success
}

/*
 * @brief Computes the inverse of a 3x3 matrix.
 * The inverse is calculated using the determinant and the adjugate matrix.
 * @param minv The matrix to store the inverse result.
 * @param m The matrix to be inverted.
 * @return 0 on success, or an error code if the operation fails.
 */
int Matrix3f_inverse(Matrix3f minv, Matrix3f m)
{
    float det = m.a[0][0] * (m.a[1][1] * m.a[2][2] - m.a[1][2] * m.a[2][1]) -
                m.a[0][1] * (m.a[1][0] * m.a[2][2] - m.a[1][2] * m.a[2][0]) +
                m.a[0][2] * (m.a[1][0] * m.a[2][1] - m.a[1][1] * m.a[2][0]);

    if (fabs(det) < 1e-12)
    {
        return -1; // Matrix is singular, cannot be inverted
    }

    float invDet = 1.0F / det;

    minv.a[0][0] = invDet * (m.a[1][1] * m.a[2][2] - m.a[1][2] * m.a[2][1]);
    minv.a[0][1] = invDet * (m.a[0][2] * m.a[2][1] - m.a[0][1] * m.a[2][2]);
    minv.a[0][2] = invDet * (m.a[0][1] * m.a[1][2] - m.a[0][2] * m.a[1][1]);
    minv.a[1][0] = invDet * (m.a[1][2] * m.a[2][0] - m.a[1][0] * m.a[2][2]);
    minv.a[1][1] = invDet * (m.a[0][0] * m.a[2][2] - m.a[0][2] * m.a[2][0]);
    minv.a[1][2] = invDet * (m.a[0][2] * m.a[1][0] - m.a[0][0] * m.a[1][2]);
    minv.a[2][0] = invDet * (m.a[1][0] * m.a[2][1] - m.a[1][1] * m.a[2][0]);
    minv.a[2][1] = invDet * (m.a[0][1] * m.a[2][0] - m.a[0][0] * m.a[2][1]);
    minv.a[2][2] = invDet * (m.a[0][0] * m.a[1][1] - m.a[0][1] * m.a[1][0]);
}

/*
 * @brief Multiplies a 3x3 matrix by a 3D vector.
 * The result is a new vector where each component is computed as the dot product
 * of the corresponding row of the matrix and the vector.
 *
 * @param m The matrix to multiply.
 * @param v The vector to multiply.
 * @return The resulting vector m * v.
 */
Vector3f Matrix_vector_mul(Matrix3f m, Vector3f v)
{
    Vector3f result = {0.0F, 0.0F, 0.0F};
    for (int i = 0; i < 3; i++)
    {
        result.x += m.a[i][0] * v.x;
        result.y += m.a[i][1] * v.y;
        result.z += m.a[i][2] * v.z;
    }
    return result;
}
