/* calibrate_ellipsoid_test.c
 *
 * Compilation (linux/mac):
 *   gcc -O2 -std=c99 calibrate_ellipsoid_test.c -lm -o calib_test
 *
 * Exécution:
 *   ./calib_test
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

/* ---------------- types fournis / demandés ---------------- */
typedef struct
{
    float x, y, z;
} Vector3f;

typedef struct
{
    float time;
    Vector3f acc;
    Vector3f gyr;
    Vector3f mag;
} MEMsSample_t;

/* Résultat de calibration */
typedef struct
{
    Vector3f offset; // centre c
    float M[3][3];   // matrice correctrice : v_corr = M * (v - c)
    float quality;   // RMS error
} CalibrationEllipsoidResult_t;


/* ---------------- Jacobi pour matrice symétrique 3x3 ----------------
   Retourne valeurs propres lambda[3] et vecteurs propres colonnes de V.
*/
void jacobi_eigen_decomposition(const Matrix3f *B, float lambda[3], Matrix3f *V)
{
    Matrix3f A = *B;
    *V = (Matrix3f){{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

    for (int iter = 0; iter < 100; iter++)
    {
        // find largest off-diag
        int p = 0, q = 1;
        float max = fabsf(A.m[0][1]);
        if (fabsf(A.m[0][2]) > max)
        {
            max = fabsf(A.m[0][2]);
            p = 0;
            q = 2;
        }
        if (fabsf(A.m[1][2]) > max)
        {
            max = fabsf(A.m[1][2]);
            p = 1;
            q = 2;
        }
        if (max < 1e-12f)
            break;

        float app = A.m[p][p], aqq = A.m[q][q], apq = A.m[p][q];
        float phi = 0.5f * atan2f(2.0f * apq, aqq - app);
        float c = cosf(phi), s = sinf(phi);

        // rotate A
        for (int i = 0; i < 3; i++)
        {
            float aip = A.m[i][p], aiq = A.m[i][q];
            A.m[i][p] = c * aip - s * aiq;
            A.m[i][q] = s * aip + c * aiq;
        }
        for (int i = 0; i < 3; i++)
        {
            float api = A.m[p][i], aqi = A.m[q][i];
            A.m[p][i] = c * api - s * aqi;
            A.m[q][i] = s * api + c * aqi;
        }
        A.m[p][q] = A.m[q][p] = 0.0f;
        float new_app = c * c * app - 2.0f * s * c * apq + s * s * aqq;
        float new_aqq = s * s * app + 2.0f * s * c * apq + c * c * aqq;
        A.m[p][p] = new_app;
        A.m[q][q] = new_aqq;

        // V = V * J
        for (int i = 0; i < 3; i++)
        {
            float vip = V->m[i][p], viq = V->m[i][q];
            V->m[i][p] = c * vip - s * viq;
            V->m[i][q] = s * vip + c * viq;
        }
    }

    lambda[0] = A.m[0][0];
    lambda[1] = A.m[1][1];
    lambda[2] = A.m[2][2];
}

/* ---------------- Solveur Gauss-Jordan 9x9 (DtD * x = DtY) -------------- */
int solve_linear_system_9x9(float A[81], float b[9], float x[9])
{
    float tmp[81];
    float tb[9];
    memcpy(tmp, A, sizeof(tmp));
    memcpy(tb, b, sizeof(tb));
    int n = 9;
    for (int i = 0; i < n; i++)
    {
        // pivot
        int piv = i;
        float max = fabsf(tmp[i * n + i]);
        for (int r = i + 1; r < n; r++)
        {
            float v = fabsf(tmp[r * n + i]);
            if (v > max)
            {
                max = v;
                piv = r;
            }
        }
        if (max < 1e-12f)
            return -1;
        if (piv != i)
        {
            for (int c = 0; c < n; c++)
            {
                float t = tmp[i * n + c];
                tmp[i * n + c] = tmp[piv * n + c];
                tmp[piv * n + c] = t;
            }
            float tt = tb[i];
            tb[i] = tb[piv];
            tb[piv] = tt;
        }
        float div = tmp[i * n + i];
        for (int c = 0; c < n; c++)
            tmp[i * n + c] /= div;
        tb[i] /= div;
        for (int r = 0; r < n; r++)
            if (r != i)
            {
                float f = tmp[r * n + i];
                for (int c = 0; c < n; c++)
                    tmp[r * n + c] -= f * tmp[i * n + c];
                tb[r] -= f * tb[i];
            }
    }
    memcpy(x, tb, sizeof(float) * 9);
    return 0;
}

/* ---------------- Construction de la calibration finale à partir de p[] et d -------------
   p[0..8] : A11, A22, A33, A12, A13, A23, b1, b2, b3
   d : scalaire (on utilise d = -1)
*/
CalibrationEllipsoidResult_t build_calibration_from_Ab(const float p[9], float d, const MEMsSample_t *samples, int count)
{
    Matrix3f A;
    A.m[0][0] = p[0];
    A.m[1][1] = p[1];
    A.m[2][2] = p[2];
    A.m[0][1] = A.m[1][0] = p[3];
    A.m[0][2] = A.m[2][0] = p[4];
    A.m[1][2] = A.m[2][1] = p[5];
    Vector3f b = {p[6], p[7], p[8]};

    CalibrationEllipsoidResult_t res;
    res.quality = -1.0f;
    // inverse A
    Matrix3f Ainv;
    if (Matrix3f_Inverse(&A, &Ainv) != 0)
    {
        res.offset = (Vector3f){0, 0, 0};
        memset(res.M, 0, sizeof(res.M));
        return res;
    }
    Vector3f tmp = Matrix3f_MultVec(&Ainv, &b);
    Vector3f c = {-tmp.x, -tmp.y, -tmp.z}; // centre
    res.offset = c;

    // k = c^T A c - d
    Vector3f Ac = Matrix3f_MultVec(&A, &c);
    float k = Vector3f_Dot(c, Ac) - d;
    if (k <= 0.0f)
    {
        memset(res.M, 0, sizeof(res.M));
        return res;
    }

    // B = A / k
    Matrix3f B;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            B.m[i][j] = A.m[i][j] / k;

    // diag via Jacobi
    float lambda[3];
    Matrix3f V;
    jacobi_eigen_decomposition(&B, lambda, &V);

    const float LMIN = 1e-8f;
    for (int i = 0; i < 3; i++)
        if (lambda[i] < LMIN)
            lambda[i] = LMIN;

    // sqrt diag
    Matrix3f Ssqrt = {{{{sqrtf(lambda[0]), 0, 0}, {0, sqrtf(lambda[1]), 0}, {0, 0, sqrtf(lambda[2])}}}};
    Matrix3f Vt = Matrix3f_Transpose(&V);
    Matrix3f Mmat = Matrix3f_Mult(&Ssqrt, &Vt); // M = sqrt(Lambda) * V^T

    // copy to result
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            res.M[i][j] = Mmat.m[i][j];

    // quality RMS
    double errsum = 0.0;
    for (int i = 0; i < count; i++)
    {
        Vector3f v = samples[i].acc;
        Vector3f vc = Vector3f_Sub(v, c);
        Vector3f vcal = Matrix3f_MultVec(&Mmat, &vc);
        float norm = Vector3f_Norm(vcal);
        float e = norm - 1.0f;
        errsum += (double)e * (double)e;
    }
    res.quality = (float)sqrt(errsum / (double)count);
    return res;
}

/* ---------------- Fit linéaire (construction D et résolution) ----------------
   On fixe d = -1. Paramètres p[9] order as:
   [A11, A22, A33, A12, A13, A23, b1, b2, b3]
*/
int fit_ellipsoid_linear(const MEMsSample_t *samples, int count, float p_out[9])
{
    if (count < 30)
        return -1; // trop peu d'échantillons
    // D is count x 9, but we'll compute DtD and DtY directly to avoid big memory
    float DtD[9 * 9];
    float DtY[9];
    memset(DtD, 0, sizeof(DtD));
    memset(DtY, 0, sizeof(DtY));

    // Y is 1 for each row (because d = -1, we moved to RHS with +1)
    for (int k = 0; k < count; k++)
    {
        float x = samples[k].acc.x;
        float y = samples[k].acc.y;
        float z = samples[k].acc.z;
        float row[9];
        row[0] = x * x;
        row[1] = y * y;
        row[2] = z * z;
        row[3] = 2.0f * x * y;
        row[4] = 2.0f * x * z;
        row[5] = 2.0f * y * z;
        row[6] = 2.0f * x;
        row[7] = 2.0f * y;
        row[8] = 2.0f * z;
        // DtD += row^T * row
        for (int i = 0; i < 9; i++)
            for (int j = 0; j < 9; j++)
                DtD[i * 9 + j] += row[i] * row[j];
        // DtY += row * 1
        for (int i = 0; i < 9; i++)
            DtY[i] += row[i];
    }

    // solve DtD * p = DtY
    float p[9];
    if (solve_linear_system_9x9(DtD, DtY, p) != 0)
    {
        return -2;
    }
    for (int i = 0; i < 9; i++)
        p_out[i] = p[i];
    return 0;
}

/* ---------------- API principal : calibrate_ellipsoid ----------------
   Retourne 0=ok, remplit result.
*/
int calibrate_ellipsoid(const MEMsSample_t *samples, int count, CalibrationEllipsoidResult_t *result)
{
    float p[9];
    int r = fit_ellipsoid_linear(samples, count, p);
    if (r != 0)
        return r;
    float d = -1.0f;
    *result = build_calibration_from_Ab(p, d, samples, count);
    if (result->quality < 0.0f)
        return -3;
    return 0;
}

/* ---------------- Utilities pour générer matrice de rotation simple 3D ------------- */
void matrix_from_axis_angle(float ux, float uy, float uz, float angle, Matrix3f *R)
{
    float c = cosf(angle), s = sinf(angle), t = 1.0f - c;
    // normalize axis
    float norm = sqrtf(ux * ux + uy * uy + uz * uz);
    if (norm == 0.0f)
    {
        *R = (Matrix3f){{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
        return;
    }
    ux /= norm;
    uy /= norm;
    uz /= norm;
    R->m[0][0] = t * ux * ux + c;
    R->m[0][1] = t * ux * uy - s * uz;
    R->m[0][2] = t * ux * uz + s * uy;
    R->m[1][0] = t * ux * uy + s * uz;
    R->m[1][1] = t * uy * uy + c;
    R->m[1][2] = t * uy * uz - s * ux;
    R->m[2][0] = t * ux * uz - s * uy;
    R->m[2][1] = t * uy * uz + s * ux;
    R->m[2][2] = t * uz * uz + c;
}

/* inverse of 3x3 by reuse function above */
int matrix_inverse3(const Matrix3f *A, Matrix3f *inv) { return Matrix3f_Inverse(A, inv); }

/* ---------------- Test: generate synthetic data ----------------
   We generate points v_true on unit sphere, then raw = M_true^{-1} * v_true + b_true
   Then we attempt to recover c (should be b_true) and M (should be M_true).
*/
#define N_SAMPLES 800

void generate_synthetic_data(MEMsSample_t *samples, int n, Matrix3f *M_true, Vector3f b_true)
{
    // seed deterministic
    unsigned int seed = 12345;
    // Prepare M_true inverse
    Matrix3f Minv;
    matrix_inverse3(M_true, &Minv);

    for (int i = 0; i < n; i++)
    {
        float u = (float)rand_r(&seed) / (float)RAND_MAX;
        float v = (float)rand_r(&seed) / (float)RAND_MAX;
        float theta = 2.0f * M_PI * u;
        float phi = acosf(2.0f * v - 1.0f);
        // generate point on unit sphere
        Vector3f vtrue;
        vtrue.x = sinf(phi) * cosf(theta);
        vtrue.y = sinf(phi) * sinf(theta);
        vtrue.z = cosf(phi);
        // raw = M_true^{-1} * vtrue + b_true
        Vector3f tmp = Matrix3f_MultVec(&Minv, &vtrue);
        Vector3f raw = Vector3f_Add(tmp, b_true);
        samples[i].time = (float)i * 0.01f;
        samples[i].acc = raw;
        samples[i].gyr = (Vector3f){0, 0, 0};
        samples[i].mag = (Vector3f){0, 0, 0};
    }
}

/* ---------------- Helper printing matrix/vector ---------------- */
void printMatrix3(const Matrix3f *A)
{
    for (int i = 0; i < 3; i++)
        printf(" % .6f % .6f % .6f\n", A->m[i][0], A->m[i][1], A->m[i][2]);
}
void printMatF3(const float M[3][3])
{
    for (int i = 0; i < 3; i++)
        printf(" % .6f % .6f % .6f\n", M[i][0], M[i][1], M[i][2]);
}

/* ---------------- main : exécution du test ---------------- */
int main_ellipsoid(void)
{
    MEMsSample_t samples[N_SAMPLES];

    // ground truth: build M_true = R * diag(scales)  (so M_true is 3x3)
    Matrix3f R;
    matrix_from_axis_angle(0.3f, 0.7f, 0.2f, 0.6f, &R);
    float scales[3] = {1.12f, 0.92f, 1.05f};
    Matrix3f S = {{{{scales[0], 0, 0}, {0, scales[1], 0}, {0, 0, scales[2]}}}};
    Matrix3f Mtrue = Matrix3f_Mult(&S, &R); // M_true = S * R
    Vector3f btrue = {0.05f, -0.03f, 0.08f};

    generate_synthetic_data(samples, N_SAMPLES, &Mtrue, btrue);

    CalibrationEllipsoidResult_t res;
    int rc = calibrate_ellipsoid(samples, N_SAMPLES, &res);
    if (rc != 0)
    {
        printf("Calibration failed (code %d)\n", rc);
        return -1;
    }

    printf("Ground truth offset b:    x=% .6f y=% .6f z=% .6f\n", btrue.x, btrue.y, btrue.z);
    printf("Estimated center c:       x=% .6f y=% .6f z=% .6f\n", res.offset.x, res.offset.y, res.offset.z);
    printf("\nGround truth M_true (3x3):\n");
    printMatrix3(&Mtrue);
    printf("\nEstimated M (should be close to M_true):\n");
    printMatF3(res.M);
    printf("\nQuality (RMS of ||v_corr||-1) = %g\n", res.quality);

    // Compare roughly M * (raw - c) norms mean/std
    double mean_norm = 0, var_norm = 0;
    for (int i = 0; i < N_SAMPLES; i++)
    {
        Vector3f vc = Vector3f_Sub(samples[i].acc, res.offset);
        Matrix3f Mmat;
        for (int a = 0; a < 3; a++)
            for (int b = 0; b < 3; b++)
                Mmat.m[a][b] = res.M[a][b];
        Vector3f vcal = Matrix3f_MultVec(&Mmat, &vc);
        float nrm = Vector3f_Norm(vcal);
        mean_norm += nrm;
    }
    mean_norm /= (double)N_SAMPLES;
    printf("Mean norm after correction = % .6f  (should be ~1.0)\n", mean_norm);

    return 0;
}
