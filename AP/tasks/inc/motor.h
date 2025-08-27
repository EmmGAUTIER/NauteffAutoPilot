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

/*
    Motor and clutch commands are connected to GPIOA pins as follows :
    - PA4 : motor command,
    - PA5 : clutch command, optionnal, connected to green LED on Nucleo board
    - PA6 : INA, motor direction
    - PA7 : INB, motor direction
*/

/*
 * \brief Run the motor to port
 * This function sets the GPIO pins to run the motor to port.
 * It is called by taskMotor.
 * \param void
 * \return void
 */


INLINE static void motorRunToPort(void)
{
    /* Set PWN and INA, reset INB */

    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4 | LL_GPIO_PIN_6);
}

/*
 * \brief Run the motor to starboard
 * This function sets the GPIO pins to run the motor to starboard.
 * It is called by taskMotor.
 * \param void
 * \return void
 * \note The motor is run to starboard when the INA pin is set to high and the INB pin is set to low.
 *       The motor is run to port when the INA pin is set to low and the INB pin is set to high.
 *       The motor is stopped when both INA and INB pins are set to low.
 */

INLINE static void motorRunToStarboard(void)
{
    /* Set PWN and INB, reset INA */
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4 | LL_GPIO_PIN_7);
}

/*
 * \brief Stop the motor
 * This function sets the GPIO pins to stop the motor.
 * It is called by taskMotor or ADC interrupt if overcurrent is detected.
 * \param void
 * \return void
 */

INLINE static void motorStop(void)
{
    /* Reset PWN, INA and INB */
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
}

/*
 * \brief Engage the clutch
 * This function sets the GPIO pin to engage the tiller.
 * pin is connected to the motor driver
 * it is also connected to a green LED on the Nucleo board.
 * It is called by taskMotor.
 * \param void
 * \return void
 */

INLINE static void motorTillerEngage(void)
{
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
}

/*
 * \brief Disengage the clutch
 * This function sets the GPIO pin to disengage the tiller.
 * \param void
 * \return void
 */

INLINE static void motorTillerDisengage(void)
{
    /* Reset Clutch (and LED), INA, INB and motor */
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
}

void MOTOR_engage();
void MOTOR_disengage();
void MOTOR_move_angle(float angle);
void MOTOR_move_time(float time);
void MOTOR_stop();