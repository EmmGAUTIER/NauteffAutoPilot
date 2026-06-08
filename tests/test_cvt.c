#include <stdio.h>
#include <math.h>
#include "geom.h"
#include "assert.h"

/*
  \brief compare two floats
  \return 1 if they are nearly equal, 0 if significantly different
*/
int cmpFloats(float a, float b)
{
    float mag;
    float diff;

    mag = fabs(a) +fabs(b);
    diff = fabs(a - b);

    if (mag == 0.0)
        return 1; /* both are zero */
    else
        if (diff/mag < 1.0e-6)
            return 1; /* they are nearly equal */
        else
            return 0; /* they are significantly different */
}

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

int main()
{
    int i;
    Vector3f v1, v2, vr;

    printf (" Essais de conversions\n");
    printf("\n");
    printf("Conversion direction en degrés vers radians\n");

    /* Test de la fonction rad2deg converting radians to degrees. */

    assert (cmpFloats(cvt_dir_rad_deg(0.)           ,  90.0));
    //assert (cmpFloats(cvt_dir_rad_deg(M_PI_2)       ,   0.0));
    assert (cmpFloats(cvt_dir_rad_deg(M_PI_4)       ,  45.0));
    assert (cmpFloats(cvt_dir_rad_deg(M_PI)         , 270.0));
    assert (cmpFloats(cvt_dir_rad_deg(M_PI)         , 270.0));
    assert (cmpFloats(cvt_dir_rad_deg(2. * M_PI)    ,  90.0));
    assert (cmpFloats(cvt_dir_rad_deg(3. * M_PI)    , 270.0));
    assert (cmpFloats(cvt_dir_rad_deg(3.5 * M_PI)   , 180.0));
    assert (cmpFloats(cvt_dir_rad_deg(-3.5 * M_PI)  ,   0.0));

   /* Test de la fonction deg2rad converting degrees to radians */
    printf("\n");
    printf("Conversion direction en radians vers degrés\n");

    assert (cmpFloats(cvt_dir_deg_rad(0.0),     M_PI_2));
    assert (cmpFloats(cvt_dir_deg_rad(45.0),    M_PI_4));
    assert (cmpFloats(cvt_dir_deg_rad(90.0),    0.0));
    assert (cmpFloats(cvt_dir_deg_rad(180.0),   1.5 * M_PI));
    //assert (cmpFloats(cvt_dir_deg_rad(270.0),   M_PI));
    assert (cmpFloats(cvt_dir_deg_rad(360),     M_PI_2));
    //assert (cmpFloats(cvt_dir_deg_rad(405),     M_PI_4));
    //assert (cmpFloats(cvt_dir_deg_rad(720.0),   M_PI_2));
    assert (cmpFloats(cvt_dir_deg_rad(-90.),    M_PI));
    assert (cmpFloats(cvt_dir_deg_rad(-270.0),  0.0));
    //assert (cmpFloats(cvt_dir_deg_rad(-360.0),  M_PI_2));
    //assert (cmpFloats(cvt_dir_deg_rad(-405),    0.75 * M_PI));

    printf("\nTest de la fonction cvt_dir_norm_deg()\n");
    for (i = 0; i < sizeof(test_val_norm_deg)/sizeof(test_val_norm_deg[0]); i++) {
        float di = test_val_norm_deg[i][0];
        float ra = test_val_norm_deg[i][1];
        float rb = cvt_dir_norm_deg(di);
        int   ok = cmpFloats(ra, rb);
        printf("%6.2f  --->  %6.2f  %6.2f   %s\n", di, ra, rb, ok==1 ? "OK" : "Échec");
    }   
    
    printf("\nTest de la fonction cvt_dir_norm_rad()\n");
    for (i = 0; i < sizeof(test_val_norm_rad)/sizeof(test_val_norm_rad[0]); i++) {
        float di = test_val_norm_rad[i][0];
        float ra = test_val_norm_rad[i][1];
        float rb = cvt_dir_norm_rad(di);
        int   ok = cmpFloats(ra, rb);
        //printf("%8.5f  --->  %8.5f  %8.5f   %s\n", di, ra, rb, ok==1 ? "OK" : "Échec");
        printf("%8.5f  --->  %20.16f  %20.16f   %s\n", di, ra, rb, ok==1 ? "OK" : "Échec");
    }

#if 0
    printf("\nTest de la fonction Vector3f_getNorm()\n");
    v1 = vector3f_init(1.0F, 2.0F, 3.0F);
    v2 = vector3f_init(1.0F, 0.0F, 0.0F);
    assert (cmpFloats(vector3f_norm(v1), sqrtf(14.0F)));
    assert (cmpFloats(vector3f_norm(v2), sqrtf(1.0F)));

    printf("\nTest de la fonction Vector3f_add()\n");
    v1 = vector3f_init(1.0F,  2.0F,  3.0F);
    v2 = vector3f_init(1.0F,  0.0F, -2.0F);
    vr = vector3f_add(v1, v2);
    assert (cmpFloats(vr.x, 2.0F));
    assert (cmpFloats(vr.y, 2.0F));
    assert (cmpFloats(vr.z, 1.0F));

    printf("\nTest de la fonction Vector3f_sub()\n");
    v1 = vector3f_init(-1.0F,  2.0F,  3.0F);
    v2 = vector3f_init( 1.0F,  3.0F, -2.0F);
    vr = vector3f_sub(v1, v2);
    assert (cmpFloats(vr.x, -2.0F));
    assert (cmpFloats(vr.y, -1.0F));
    assert (cmpFloats(vr.z,  5.0F));

    printf("\nTest de la fonction Vector3f_cross()\n");
    v1 = vector3f_init( 1.0F,  2.0F,  3.0F);
    v2 = vector3f_init( 1.0F, -2.0F,  4.0F);
    vr  = vector3f_getCrossProduct(v1, v2);
    assert (cmpFloats(vr.x,  14.0F));
    assert (cmpFloats(vr.y,  -1.0F));
    assert (cmpFloats(vr.z,  -4.0F));

#endif

    printf ("\r\n");
}


