/*
 * This file is part of the Nauteff Autopilot project.
 *
 * Copyright (C) 2022 Nauteff https://nauteff.com
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef NMEA_H
#define NMEA_H

#include <stdint.h>
#include "nmea.h"

#define NMEA_NB_MAX_FIELDS (25)

typedef enum
{
    Nmea0183_unk = -1,
    Nmea0183_APB = 1,
    Nmea0183_MWV,
    Nmea0183_RMB,
    Nmea0183_RMC
} Nmea0183Type;

typedef struct
{
    char valid_blink;   /* A : data valid V : else */
    char valid_lock;    /* A : data valid V : else */
    float trackError;   /* Cross track error magnitude */
    char crossTrackUnit;/* Unit of cross track error N = Nautical mile */
    char dirToSteer;    /* Direction to steer : L or R */
    char arrivalCircle; /* Arrival circle entered */
    char Perpendicular; /* Perpendicular passed at waypoint */
    float bearingOD;    /* Bearing origin to destination */
    char  bearingODMT;  /* M : Magnetic, T : True */
    float bearingPD;    /* Bearing present to destination */
    char  bearingPDMT;  /* M : Magnetic, T : True */
    float steerToWP;    /* Heading to steer to destination waypoint */
    char steerToWPMT;    /* M = Magnetic, T = True */
} Nmea0183Data_APB;

/* MWV : Wind Speed and Angle */
typedef struct
{
    float windAngle;   /* Wind Angle, 0 to 359 degrees */
    char  windRef;     /* Reference, R = Relative, T = True */
    float windSpeed;   /* Wind Speed*/
    char  speedUnit;   /* Wind Speed Units, K/M/ */
    char  status;      /* Status, A = Data Valid, V = Invalid */
} Nmea0183Data_MWV;

/* RMB - Recommended Minimum Navigation Information */
/* To be sent by a navigation receiver when a destination WP is active. */
typedef struct
{
    char valid;             /* 1 -Status, A = Active, V = Invalid          */
    float crossTrackError;  /* 2 - Cross Track error - nautical miles      */
    float directionToSteer; /* 3 - Direction to Steer, Left or Right       */
    char  origWP[12];       /* 4 - Origin Waypoint ID                      */
    char  destWP[12];        /* 5 - Destination Waypoint ID                 */
    float destWPLat;        /* 6 - Dest. WP Latitude N/S included in sign  */
    /* useless */           /* 7 - N or S in sign of destWPLat             */
    float destWPLong;       /* 8 - Dest. WP Longitude E/W included in sign */
    /* useless */           /* 9 - E or W in sign of destWPLat             */
    float rangeToDest;      /* 10 - Range to destination in nautical miles */
    float bearingToDest;    /* 11 - Bearing to destination in degrees True */
    float destVelocity;     /* 12 - Destination closing velocity in knots  */
    char status;            /* 13 - Arrival Status, A = Arrival Circle     */
                            /* Entered. V = not entered/passed             */
    char FAAIndicator;       /*14 - FAA mode indicator (NMEA 2.3 and later) */
} Nmea0183Data_RMB;

/* RMC - Recommended Minimum Navigation Information */
/* This is one of the sentences commonly emitted by GPS units. */
typedef struct
{
    /* UTC Time of position  hhmmss.cc         */
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t millisec;
    uint32_t time_ms;
    /* Status, A = Valid, V = Warning          */
    char valid;
    /* Latitude                                */
    /* N or S in sign of longitude             */
    float latitude; 
    /* Longitude                               */
    /* E or W                                  */
    float longitude; 
    /* Speed over ground, knots                */
    float speedOverGround;
    /* Track made good, degrees true           */
    float trackdir;
    /* Date, ddmmyy                            */
    uint8_t day;
    uint8_t month;
    uint8_t year;
    /* Magnetic Variation, degrees             */
    float magneticVar;
    /* E or W                                  */
    /* FAA mode indicator (NMEA 2.3 and later) */
    //time_t UTCTime;
} Nmea0183Data_RMC;

typedef union
{
    Nmea0183Data_APB apb;
    Nmea0183Data_MWV mwv;
    Nmea0183Data_RMB rmb;
    Nmea0183Data_RMC rmc;
} Nmea0183SentData;

typedef struct
{
    char originalFrame[80];
    uint16_t type;
    char origin[3];
    char typeName[4];
    Nmea0183SentData data;
} Nmea0183SentTypeData;

int nmea0183Decode(const char *sentence, Nmea0183SentTypeData *d);

#endif  /* #ifndef NMEA_H*/
