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

/*
 * @file Module "motor"
 * Ce module assure le contrôle du moteur.
 *  - Commande de l'embrayage et allumage de la LED verte de la carte Nucleo
 *  - Mise en marche et arrêt du moteur dans un sens ou l'autre
 *  - Détection de défaut ou de butée
 *  - Gestion de la durée de fonctionnement
 *  - Estimation de l'effort et ajustement de la durée de fonctionnement
 *  - Communication avec la tâche principale
 */

#include "FreeRTOS.h"
#include "queue.h"

#include "util.h"

extern QueueHandle_t msgQueueMotor;
int init_taskMotor();

typedef enum
{
    MSG_MOTOR_NONE = 0,
    MSG_MOTOR_ADC_VALUES,
    MSG_MOTOR_EMBRAYE,
    MSG_MOTOR_DEBRAYE,
    MSG_MOTOR_SET_HELM_ANGLE,
    MSG_MOTOR_MOVE_TIME,
    MSG_MOTOR_MOVE_DONE,
    MSG_MOTOR_SET_CVT_ANGLE_TIME,
    MSG_MOTOR_DISPLAY_CONFIG,
    MSG_MOTOR_DEFAULT,
} MsgMotorType_t;

typedef struct
{
    uint16_t msgType;      /* Code de message */
    uint16_t defaultCodes; /* Overcurrent, voltage drop, ... */
    union
    {
        struct
        {
            /* Ces valeurs sont envoyées par une interruption */
            /* Elles ne doivent pas être de type float */
            uint16_t adc_power;   /* Tension d'alimentation */
            uint16_t adc_current; /* Courant moteur */
        } adcValues;
        float moveTime;
        float steerAngle;
        float cvtAngleTime;
    } data;
} MsgMotor_t;

void taskMotor(void *);

/**
 * @brief Send the order "engage clutch" to motor task
 * @param none
 * Motor task stops motor if it is running and engage clutch.
 * @return none
 */
void MOTOR_MSG_letInClutch();

/**
 * @brief send the order disengage clutch to motor task
 * @param none
 * Motor task release clutch and stops motor if it is running.
 * Task motor sends a message later when motor is stopped if it was moving.
 * @return none
 */
void MOTOR_MSG_letOutClutch();

/**
 * @brief send the order move angle
 * @param angle to move radians clockwise if positive, counterclockwise if negative
 * @note The direction to steer is usually indicated clockwise.
 * @note To turn starboard, i.e. right, helm has to be moved to port and vice versa.
 * 
 * @return none
 */
void MOTOR_MSG_setHelmAngle(float angle);

/**
 * @brief send the order move for a time
 * @param time to move in seconds  counterclockwise if negative, clockwise if positive
 * Used to move helm when not in auto mode.
 * Task motor send a message when move done.
 * Previous move order for time is discarded if motor was running same direction.
 * Move order is ignored if if motor is running oposite direction.
 * Used for moving continuously with repeated pushes on button.
 * @return none
 */
void MOTOR_MSG_moveTime(float time);

/*
 * @brief Set the conversion factor between helm angle and time
 * @param cat Conversion factor angle to time
 * @return void
 */
void MOTOR_MSG_set_cvt_angle_time(float cvt);
