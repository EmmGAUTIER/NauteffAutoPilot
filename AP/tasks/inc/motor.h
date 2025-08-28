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
    MSG_MOTOR_START,
    MSG_MOTOR_STOP,
    MSG_MOTOR_EMBRAYE,
    MSG_MOTOR_DEBRAYE,
    MSG_MOTOR_ERROR,
    MSG_MOTOR_EFFORT,
    MSG_MOTOR_MOVE_ANGLE,
    MSG_MOTOR_MOVE_TIME,
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
            uint32_t SR;          /* Status register */
        } adcValues;
        float moveTime;
        float moveAngle;
    } data;
} MsgMotor_t;

void taskMotor(void *);

/**
 * @brief Send the order "engage clutch" to motor task
 * @param none
 * Motor task stops motor if it is running and engage clutch.
 * @return none
 */
void MOTOR_engage();

/**
 * @brief send the order disengage clutch to motor task
 * @param none
 * Motor task release clutch and stops motor if it is running.
 * Task motor sends a message later when motor is stopped if it was moving.
 * @return none
 */
void MOTOR_disengage();

/**
 * @brief send the order move angle
 * @param angle to move radians counterclockwise (as in trigonometric functions)
 * @return none
 */
void MOTOR_move_angle(float angle);

/**
 * @brief send the order move for a time
 * @param time to move in seconds  counterclockwise if positive, clockwise if negative
 * Used to move tiller when not in auto mode.
 * Task motor send a message when move done.
 * Previous move order for time is discarded if motor was running.
 * Used for moving continuously with repeated pushes on button.
 * @return none
 */
void MOTOR_move_time(float time);

/**
 * @brief send the order "stop motor"
 * @param none
 * @return none
 */
void MOTOR_stop();