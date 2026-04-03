#ifndef AHRS_NED_H
#define AHRS_NED_H

#include "util.h"
#include "geom.h"    /* Pour Vector3_f */
#include "quat.h" /* Pour Quaternion_f */

/* Constantes pour le filtre ASGD+Kalman en NED */
#define AHRS_ALPHA          (0.1f)    /* part of magnetometer for heading evaluation */

/* Status of device and return code */
typedef enum {
	AHRS_OK = 0,        /* Ok, data avaiable */
	AHRS_UNINITIALIZED, /* Uninitialised, needs a first call to update() */
	AHRS_ERROR_VALUES   /* Argument invalide */
} AHRSStatus_t;

/* Structure d'état du filtre AHRS en NED */
typedef struct {
	Quaternionf quaternion; /* Quaternion estimé (NED) */
	Vector3f gravity;       /* Vecteur gravité dans le repère corps */
	float roll;             /* Roulis (rad) */
	float pitch;            /* Tangage (rad) */
	float heading;              /* Cap (rad) */
	AHRSStatus_t status;    /* status */
} AHRSState_t;

/* Prototypes des fonctions */
AHRSStatus_t AHRS_init(AHRSState_t *const ahrs);

AHRSStatus_t AHRS_update(AHRSState_t *const ahrs, const Vector3f *const acc,
		const Vector3f *const gyr, const Vector3f *const mag,
		const float deltat);

INLINE Quaternionf AHRS_get_Quaternion(const AHRSState_t *const ahrs) {
	return ahrs->quaternion;
}

INLINE float AHRS_get_roll(const AHRSState_t *const ahrs) {
	return ahrs->roll;
}

INLINE float AHRS_get_pitch(const AHRSState_t *const ahrs) {
	return ahrs->pitch;
}

INLINE float AHRS_get_heading(const AHRSState_t *const ahrs) {
	return ahrs->heading;
}

INLINE Vector3f AHRS_get_gravity(const AHRSState_t *const ahrs) {
	return ahrs->gravity;
}

#endif /* AHRS_NED_H */
