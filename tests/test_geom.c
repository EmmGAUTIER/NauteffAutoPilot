#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "aux_fcts.h"
#include "geom.h"

struct {
    Vector3f v1;
    Vector3f v2;
    Vector3f vr;
    float scal;
} test_vector3[] = {
    {{1.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, 2.0F},
    {{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}, {1.0F, 2.0F, 3.0F}, 1.0F},
    {{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}, {2.5F, 3.5F, 4.5F}, 2.5F},
    {{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}, {3.5F, 4.5F, 5.5F}, -1.F},
    {{1.F, -2.F, -3.F}, {-4.F, -5.F, -6.F}, {-3.F, -7.F, -9.F}, -1.F}
};

#if 0
/* radians of angles start from left and grow counterclockwise */
/* degrees of angles start fom top and grow clockwise. */

/* Values for testing normalization [0 2 pi[ of angles */
float test_val_norm_rad[][2] = {
    {        0.0F,         0.0F},
    {  0.5F * M_PIF,  0.5F * M_PIF},
    {  1.0F * M_PIF,  1.0F * M_PIF},
    {  1.5F * M_PIF,  1.5F * M_PIF},
    {  2.0F * M_PIF,         0.0F},
    {  2.1F * M_PIF,         0.1F * M_PIF},
    {  2.5F * (float)M_PIF, +0.5F * (float)M_PIF},
    { -0.5F * M_PIF, +1.5F * M_PIF},
    { -1.0F * M_PIF, +1.0F * M_PIF},
    { -1.5F * M_PIF, +0.5F * M_PIF},
    { -2.0F * M_PIF,         0.0F},
    { -2.5F * M_PIF, +1.5F * M_PIF},
    {  4.0F * M_PIF,         0.0F},
    {  4.5F * M_PIF, +0.5F * M_PIF},
    { -4.0F * M_PIF,         0.0F},
    { -4.5F * M_PIF, +1.5F * M_PIF}
};

/* Jeu de tests pour normalisation des directions en degrés */
float test_val_norm_deg[][2] = {
    {(float)0.0, (float)0.0},
    {   1.0F,   1.0F},
    { 361.0F,   1.0F},
    {  90.0F,  90.0F},
    { 180.0F, 180.0F},
    { 270.0F, 270.0F},
    { 360.0F,   0.0F},
    { 450.0F,  90.0F},
    { -90.0F, 270.0F},
    {-180.0F, 180.0F},
    {-270.0F,  90.0F},
    {-360.0F,   0.0F},
    {-450.0F, 270.0F},
    { 720.0F,   0.0F},
    { 810.0F,  90.0F},
    {-720.0F,   0.0F},
    {-810.0F, 270.0F},
    {  91.25647654F,   91.25647654F},
    { 591.25647654F,  231.25647654F},

};
#endif

/* Jeu de tests pour normalisation dans l'intervale [-pi, +pi[ des directions en degrés
 * Les directions du tableau sont en degrés pour faciliter la saisie */
float test_val_norm_deg_centered[][2] = {
    {    0.0F,    0.0F},
    {    1.0F,    1.0F},
    {  179.0F,  179.0F},
    {  180.0F,  180.0F},
    {  181.0F, -179.0F},
    {  359.0F,  -1.0F},
    {  361.0F,    1.0F},
    {  721.0F,    1.0F},
    {    0.0F,    0.0F},
    {   90.0F,  90.0F},
    {  -90.0F, -90.0F},
    { -181.0F, 179.0F},
    { -732.0F, -12.0F},
};


int test_geom()
{
    int i;
    Vector3f v1, v2, vr;
    float scalar;

    printf ("\ntest de la fonction Vector3f_getNorme()\n");
    v1 = vector3f_init(5.0F, 2.0F, 3.0F);
    assert (cmpFloats(vector3f_getNorm(v1), sqrt(5.0F * 5.0F + 2.0F * 2.0F + 3.0F * 3.0F)));

    printf ("\nTest de la fonction vector3f_add()\n");
    v1 = vector3f_init(1.0F, 2.0F, 3.0F);
    v2 = vector3f_init(4.0F, 5.0F, 6.0F);
    vr = vector3f_add(v1, v2);
    assert (cmpFloats(vr.x, 5.0F));
    assert (cmpFloats(vr.y, 7.0F));
    assert (cmpFloats(vr.z, 9.0F));

    printf ("\nTest de la fonction vector3f_sub()\n");
    v1 = vector3f_init(8.0F,  7.0F, -6.0F);
    v2 = vector3f_init(4.0F, -5.0F,  2.0F);
    vr = vector3f_sub(v1, v2);
    assert (cmpFloats(vr.x,  4.0F));
    assert (cmpFloats(vr.y, 12.0F));
    assert (cmpFloats(vr.z, -8.0F));

    printf("\nTest de la fonction vector3f_getCrossProduct()\n");
    v1 = vector3f_init( 5.0F,  7.0F,  3.0F);
    v2 = vector3f_init( 1.0F, -2.0F,  1.5F);
    vr = vector3f_getCrossProduct(v1, v2);
    assert (cmpFloats(vr.x,  16.5F));
    assert (cmpFloats(vr.y,  -4.5F));
    assert (cmpFloats(vr.z, -17.0F));

    printf ("\nTest de la fonction vector3f_dot()\n");
    // Même vecteurs que le produit vectoriel
    scalar = vector3f_getDotProduct(v1, v2);
    assert (cmpFloats(scalar, 5.0F * 1.0F + 7.0F * -2.0F + 3.0F * 1.5F));

    printf ("\nTest de la fonction vector3f_scale()\n");
    v1 = vector3f_init(1.0F, 2.0F, 3.0F);
    v2 = vector3f_init(4.0F, 5.0F, 6.0F);
    vr = vector3f_getScaled(v1, 2.0F);
    assert (cmpFloats(vr.x, 2.0F));
    assert (cmpFloats(vr.y, 4.0F));
    assert (cmpFloats(vr.z, 6.0F));

    printf ("Test de la fonction vector3f_getCosine()\n");
    v1 = vector3f_init(1.0F, 2.0F, 3.0F);
    v2 = vector3f_init(4.0F, 5.0F, 6.0F);
    scalar = vector3f_getCosine(v1, v2);
    assert (cmpFloats(scalar, 0.97463199F));

    scalar = vector3f_getCosine(unitzf, unitzf);
    assert (cmpFloats(scalar, 1.0F));

    scalar = vector3f_getCosine(unitzf, unitmzf);
    assert (cmpFloats(scalar, -1.0F));

    scalar = vector3f_getCosine(unitzf, unityf);
    assert (cmpFloats(scalar, 0.0F));

    printf ("Test de la fonction normalize_angle_rad_centered()\n");
    for (int i = 0 ; i <12 ; i++)
    {
        int res;
        float valinput = test_val_norm_deg_centered[i][0];
        float valexpected = test_val_norm_deg_centered[i][1];
        float valout = normalize_angle_rad_centered(valinput * (M_PIF/180.F));
        res = cmpFloats(valexpected * (M_PIF/180.F), valout);
        printf("%+8.2f   %+8.2f %+8.2f  %d\n",
               valinput,
               valout * (180.F/M_PIF),
               valexpected,
               res);
    }
}

