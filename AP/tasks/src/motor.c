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
 * Ces fonction permettent le contrôle de l'actuateur, elles assurent :
 *   - la commande d'embrayage ;
 *   - la commande du moteur et le sens pendant un temps imparti ou un angle estimé ;
 *   - la mesure et la surveillance du courant pendant le fonctionnement du moteur ;
 *   - la mesure et  la surveillance de la tension d'alimentation ;
 *   - l'arrêt du moteur en cas de surcharge, de court-cicuit ou de détection de buttée ;
 *   - Une évaluation de la position de la barre avec le courant et la tension d'alimentation ;
 */

#include <math.h>

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

#include <stm32l452xx.h>
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_hal_adc.h"

#include "util.h"
#include "printf.h"
#include "rlib.h"
#include "service.h"
// #include "aux_usart.h"

#include "autopilot.h"
#include "motor.h"
#include "apdialog.h"

#define DBG_MOTOR_PRINT(X) (X)
#define DBG_ADC_PRINT(X) 

extern TIM_HandleTypeDef htim3;

#define MOTOR_EVENT_STALLED (0x1 << 0)
#define MOTOR_EVENT_STOP (0x1 << 1)

#define MOTOR_V_CURRENT_NONE (0.F)
#define MOTOR_V_CURRENT_FREE (.35F)
#define MOTOR_V_CURRENT_BLOCKED (1.0F)     /*  */
#define ADC_PERIOD (0.01F)                 /* 100 ms */
#define MOTOR_TIME_TO_STOP (0.1F)          /* 100 ms */
#define MOTOR_MAX_TIME_OVERCURRENT (0.05F) /* 200 ms */
#define MOTOR_CVT_ANGLE_TIME (10.0F)       /* Estimated conversion between time and rudder move angle */
#define ADC_CVT_TO_VOLTAGE (0.0097F)       /* Ratio ADC val. and power voltage */
#define ADC_CVT_TO_CURRENT (0.0004)        /* Ratio  ADC val. and current */

typedef enum
{
    MotorIdle,
    MotorMoveAngle,
    MotorMoveTime,
    MotorStopping,
    MotorStalled,
    MotorDefault,
} MotorState_t;

typedef enum
{
    MotorDirNone = 0,
    MotorDirPort = 1,
    MotorDirStbd = -1,
} MotorDirection_t;

typedef enum
{
    MotorClutchOut = 0,
    MotorClutchIn,
    // MotorClutchEngaging,
    // MotorClutchDisengaging,
} MotorClutchStatus_t;

QueueHandle_t msgQueueMotor = (QueueHandle_t)0;

static uint16_t adc_values[2];

static volatile int moveTime = 0;
static volatile int moveAngle = 0;

/*
 * States of the motor :
 *
 *  - Moving and powered :
 *       moving = 1
 *       direction = +1 (starboard) or -1 (port)
 *
 *  - Idle and ending move (motor descelerating
 *       moving = 2
 *       direction = +1 or -1
 *
 *  - Idle and stopped :
 *       moving    = 0
 *       direction = 0
 *
 *  - Blocked :
 *       +1 : blocked to port
 *       -1 : blocked  to starboard
 *       0  : no blocking condition
 *
 *  - Overcurrent :
 *       overcurrent = 1 if overcurrent has been detected
 *       overcurrent = 0 if no overcurrent
 *       becomes 1 if current / power voltage is more than twice
 *       the ration when motor is blocked
 */

/*
 * \brief Structure to hold motor data
 */

typedef struct
{

    /* Status */
    MotorState_t status;
    int engaged;   /* 0 tilller disengaged, 1 engaged */
    int direction; /* direction : -1 starboard, +1 port, 0 : idle*/
    // int blocked;   /* 0 : unblocked, +1 : blocked port, -1  :blocked starboard */

    /* Data for duration and move */
    float turnAngleReq;       /* Turn angle requested (rad counterclockwise ie with sign) */
    float turnAngleRemaining; /* Turn angle remaining positive */
    float turnTimeReq;        /* Turn angle requested (rad counterclockwise ie with sign) */
    float turnTimeRemaining;  /* Turning time Remaining positive */
    float stopTimeRemaining;  /* Time since motor powered off */
    float overCurrentTime;    /* Time since start of overcurrent */

    /* Values of calibration */
    float vcurrentNone;    /* adc value of current when not moving */
    float vcurrentFree;    /* adc value of current when moving with no effort */
    float vcurrentBlocked; /* adc value of current when motor blocked */
    float vPowerStandard;  /* Standard power voltage */
    float vPowerMin;       /* Minimum power voltage */
    float vPowerMax;       /* Maximum power voltage */
    float timeStartStop;   /* Time to start or stop the motor */

    float currentStalled; /* Current when motor is stalled */

    /* measured values */
    float vPower;   /* Actual voltage*/
    float vCurrent; /* Actual current */
} MotorData;

MotorData motorData = {

    .status = MotorIdle,
    .engaged = 0,
    .direction = 0,

    .turnAngleReq = 0.F,
    .turnAngleRemaining = 0.F,
    .turnTimeReq = 0.F,
    .turnTimeRemaining = 0.F,

    .vcurrentNone = MOTOR_V_CURRENT_NONE,
    .vcurrentFree = MOTOR_V_CURRENT_FREE,
    .vcurrentBlocked = MOTOR_V_CURRENT_BLOCKED,
    .timeStartStop = 0.1F,
    .currentStalled = 1.0F,

    .vPowerStandard = 12.F,
    .vPowerMin = 10.,
    .vPowerMax = 15.F,
    .vPower = 0.F,
    .vCurrent = 0.F};

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

INLINE static void Motor_LL_runToPort(void)
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

INLINE static void Motor_LL_runToStarboard(void)
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

INLINE static void Motor_LL_stop(void)
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

INLINE static void Motor_LL_tillerEngage(void)
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

INLINE static void Motor_LL_tillerDisengage(void)
{
    /* Reset Clutch (and LED), INA, INB and motor */
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
}

void MOTOR_order_engage()
{
    static MsgMotor_t msg = {.msgType = MSG_MOTOR_EMBRAYE};
    xQueueSend(msgQueueMotor, &msg, 0);
}

void MOTOR_order_disengage()
{
    static MsgMotor_t msg = {.msgType = MSG_MOTOR_DEBRAYE};
    xQueueSend(msgQueueMotor, &msg, 0);
}

void MOTOR_order_move_angle(float angle)
{
    static MsgMotor_t msg = {.msgType = MSG_MOTOR_MOVE_ANGLE};
    msg.data.moveAngle = angle;
    xQueueSend(msgQueueMotor, &msg, 0);
}

void MOTOR_order_move_time(float time)
{
    static MsgMotor_t msg = {.msgType = MSG_MOTOR_MOVE_TIME};
    msg.data.moveAngle = time;
    xQueueSend(msgQueueMotor, &msg, 0);
}

/*
 * @brief Update motor status with new values of voltage and current
 *
 * This function updates the motor status with new values of voltage and current
 * and stops or start th motor.
 * It is called by taskMotor when it receives new ADC values.
 *
 * @param deltat Time since last call in seconds
 * @param vPower Voltage of power supply
 * @param iMotor Current through motor
 * @return void
 */
unsigned Motor_newValues(float deltat, float vPower, float iMotor)
{
    unsigned motorEvent = 0;

    /* vPower and iCurrent have to be used later for better
     * estimating the rudder angle
     */
    (void)vPower;
    (void)iMotor;

    /* First check if motor stalled */
    if (iMotor > motorData.currentStalled * .7F)
    {
        motorData.overCurrentTime += deltat;
        if (motorData.overCurrentTime > MOTOR_MAX_TIME_OVERCURRENT)
        {
            Motor_LL_stop();
            motorData.status = MotorStalled;
            motorData.turnAngleRemaining = 0.F;
            motorData.turnAngleReq = 0.F;
            motorData.turnTimeRemaining = 0.F;
            motorData.turnTimeReq = 0.F;
            motorEvent |= MOTOR_EVENT_STALLED;
        }
    }
    else
    {
        motorData.overCurrentTime = 0.F;
    }

    switch (motorData.status)
    {

    case MotorIdle:
    case MotorStalled:
    case MotorDefault:

        /* Nothing to do */
        break;

    case MotorMoveAngle:

        motorData.turnAngleRemaining -= MOTOR_CVT_ANGLE_TIME * deltat;
        if (motorData.turnAngleRemaining <= 0.F)
        {
            Motor_LL_stop();
            motorData.status = MotorStopping;
            motorData.turnAngleRemaining = 0.F;
            motorData.turnAngleReq = 0.F;
            motorData.stopTimeRemaining = MOTOR_TIME_TO_STOP;
        }
        break;

    case MotorMoveTime:

        motorData.turnTimeRemaining -= deltat;
        if (motorData.turnTimeRemaining <= 0.F)
        {
            Motor_LL_stop();
            motorData.status = MotorStopping;
            motorData.turnTimeRemaining = 0.F;
            motorData.turnTimeReq = 0.F;
            motorData.stopTimeRemaining = MOTOR_TIME_TO_STOP;
        }
        break;

    case MotorStopping:

        motorData.stopTimeRemaining -= deltat;
        if (motorData.stopTimeRemaining <= 0.F)
        {
            motorData.status = MotorIdle;
            motorData.direction = 0;
            motorData.stopTimeRemaining = 0.F;
            motorEvent |= MOTOR_EVENT_STOP;
        }
        break;

    default:

        break;
    }

    return motorEvent;
}

/*
 * @brief Move the motor for a given time
 * @param timeToMove Time to move in seconds, positive to port, negative to stbd
 * If motor is running for a specified time in the same direction as timeToMove,
 * it sets the time to move to timeToMove;
 * @return void
 */
void Motor_moveAngle(float angleToMove)
{

    int dirtomove = (angleToMove > 0.F) ? MotorDirPort : MotorDirStbd;

    if ((motorData.status == MotorIdle) || ((motorData.status == MotorStalled) &&
                                            motorData.direction * dirtomove < 0))
    {
        /*
         * Motor Idle (and stopped) or
         * motor stalled and request for opposite direction
         */
        motorData.turnAngleReq = angleToMove;
        motorData.turnAngleRemaining = fabs(angleToMove);
        dirtomove > 0 ? Motor_LL_runToPort() : Motor_LL_runToStarboard();
        motorData.direction = dirtomove;
        motorData.status = MotorMoveAngle;
    }
    else if (motorData.status == MotorMoveAngle &&
             motorData.direction * dirtomove > 0)
    {
        /*
         * Motor moving in the same direction as requested and
         * other request to move : simply add angle to move
         */
        motorData.turnAngleReq += angleToMove;
        motorData.turnAngleRemaining += fabs(angleToMove);
    }
    else if (motorData.status == MotorMoveAngle &&
             motorData.direction * dirtomove < 0)
    {
        /*
         * Motor moving in the opposite direction as requested :
         * store request to turn and stop motor
         * when motor is stopped, it will turn in the requested direction.
         */
        Motor_LL_stop();
        motorData.status = MotorStopping;
        motorData.stopTimeRemaining = MOTOR_TIME_TO_STOP;
        motorData.turnAngleReq = angleToMove;
        motorData.turnAngleRemaining = fabs(angleToMove);
    }
    else if (motorData.status == MotorStopping &&
             motorData.direction * dirtomove < 0)
    {
        /*
         * Motor was moving in the opposite direction as requested and is stopping :
         * store request to turn, no need to stop motor
         * when motor is stopped, it will turn in the requested direction.
         */
        motorData.turnAngleReq = angleToMove;
        motorData.turnAngleRemaining = fabs(angleToMove);
    }
    else if (motorData.status == MotorStopping &&
             motorData.direction * dirtomove > 0)
    {
        /*
         * Motor was moving in the same direction as requested and is stopping :
         * add angle to move, no need to stop motor
         * when motor is stopped, it will turn in the requested direction.
         */
        motorData.status = MotorMoveAngle;
        motorData.turnAngleReq = angleToMove;
        motorData.turnAngleRemaining = fabs(angleToMove);
        dirtomove > 0 ? Motor_LL_runToPort() : Motor_LL_runToStarboard();
    }

    return;
}

/*
 * @brief Move the motor for a given time
 * @param timeToMove Time to move in seconds, positive to port, negative to stbd
 * If motor is running for a specified time in the same direction as timeToMove,
 * it sets the time to move to timeToMove;
 * @return void
 */
void Motor_moveTime(float timeToMove)
{

    int dirtomove = (timeToMove > 0.F) ? MotorDirPort : MotorDirStbd;

    if (((motorData.status == MotorMoveTime ||
          motorData.status == MotorIdle ||
          motorData.status == MotorStopping) &&
         motorData.direction * dirtomove >= 0) ||
        ((motorData.status == MotorStalled) && (motorData.direction * dirtomove < 0)))
    {
        motorData.turnTimeReq = timeToMove;
        motorData.turnTimeRemaining = fabs(timeToMove);
        dirtomove > 0 ? Motor_LL_runToPort() : Motor_LL_runToStarboard();
        motorData.direction = dirtomove;
        motorData.status = MotorMoveTime;
    }

    return;
}

void Motor_engageTiller()
{
    Motor_LL_tillerEngage();
    motorData.engaged = 1;
}

void Motor_disengageTiller()
{
    Motor_LL_stop();
    Motor_LL_tillerEngage();
    motorData.engaged = 0;
    if (motorData.status == MotorMoveAngle || motorData.status == MotorMoveTime)
        motorData.status = MotorStopping;
    else
        motorData.status = MotorIdle;
}

void Motor_stop()
{
    Motor_LL_stop();
    if (motorData.status != MotorIdle)
    {
        motorData.status = MotorStopping;
    }
}

/*
 * \brief Initialise the motor task
 * This function creates the queue
 */
int init_taskMotor()
{
    msgQueueMotor = xQueueCreate(10, sizeof(MsgMotor_t));
    if (msgQueueMotor == (QueueHandle_t)0)
    {
        return -1;
    }
    else
    {
        return 1;
    }
}

/**
 * \brief Motor control task
 */

void taskMotor(void *parameters)
{
    (void)parameters; /* parameters ignored, avoids warning */

    size_t ret;
    unsigned motorEvent;
    MsgMotor_t msgMoteur;
    char message[200];
    unsigned counter = 0;

    DBG_MOTOR_PRINT((
        snprintf(message, sizeof(message),
                 "taskMotor Départ\n"),
        svc_UART_Write(&svc_uart1, message, strlen(message), 0U)));

    HAL_TIM_Base_Start(&htim3);

    extern ADC_HandleTypeDef hadc1;

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, 2);

    for (;;)
    {

        msgMoteur.msgType = MSG_MOTOR_NONE;

        ret = xQueueReceive(msgQueueMotor,
                            &msgMoteur,
                            pdMS_TO_TICKS(500));

        if (ret != (size_t)0)
        {
            switch (msgMoteur.msgType)
            {

            case MSG_MOTOR_ADC_VALUES:

                motorData.vPower = (float)msgMoteur.data.adcValues.adc_power * ADC_CVT_TO_VOLTAGE;
                motorData.vCurrent = (float)msgMoteur.data.adcValues.adc_current * ADC_CVT_TO_CURRENT;

                motorEvent = Motor_newValues(ADC_PERIOD,
                                             (float)msgMoteur.data.adcValues.adc_power * ADC_CVT_TO_VOLTAGE,
                                             (float)msgMoteur.data.adcValues.adc_current * ADC_CVT_TO_CURRENT);

                if (counter % 5 == 0)
                {
                    DBG_ADC_PRINT((
                        snprintf(message, sizeof(message), "ADC  %6f  %6f\n", motorData.vPower, motorData.vCurrent),
                        svc_UART_Write(&svc_uart2, message, strlen(message), 0U)));
                    // snprintf(message, sizeof(message), "ADC %u %6u  %6u\n",
                    // xTaskGetTickCount(),
                    // msgMoteur.data.adcValues.adc_power,
                    // msgMoteur.data.adcValues.adc_current);
                }
                if (motorEvent & MOTOR_EVENT_STALLED)
                {
                    svc_UART_Write(&svc_uart2, "MOTOR stalled\n", 14, 0U);
                    autopilot_sendMsgMotorStall();
                }
                if (motorEvent & MOTOR_EVENT_STOP)
                {
                    svc_UART_Write(&svc_uart2, "MOTOR stop\n", 11, 0U);
                }

                break;

            case MSG_MOTOR_DEBRAYE:
                Motor_disengageTiller();

                DBG_MOTOR_PRINT((
                    svc_UART_Write(&svc_uart2, "MOTOR disengage\n", 16, 0U)));

                break;

            case MSG_MOTOR_EMBRAYE:
                Motor_engageTiller();

                DBG_MOTOR_PRINT((
                    svc_UART_Write(&svc_uart2, "MOTOR engage\n", 13, 0U)));

                break;

            case MSG_MOTOR_MOVE_ANGLE:
                Motor_moveAngle(msgMoteur.data.moveAngle);
                DBG_MOTOR_PRINT((
                    snprintf(message, sizeof(message), "MOTOR move angle %.4f\n", msgMoteur.data.moveAngle),
                    svc_UART_Write(&svc_uart2, message, strlen(message), 0U)));

                break;

            case MSG_MOTOR_MOVE_TIME:
                Motor_moveTime(msgMoteur.data.moveTime);
                DBG_MOTOR_PRINT((
                    snprintf(message, sizeof(message), "MOTOR move time %.3f\n", msgMoteur.data.moveTime),
                    svc_UART_Write(&svc_uart2, message, strlen(message), 0U)));

                break;

            case MSG_MOTOR_DISPLAY_CONFIG:
            {
                int nbcar = snprintf(message, sizeof(message),
                                     "MOTOR config : not yet implemented\n");
                svc_UART_Write(&svc_uart2, message, nbcar, 0U);
            }

            default:
                break;
            }

            counter++;
        }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

    MsgMotor_t msgMotor;
    // uint32_t vpower = 0U;
    // uint32_t vcurrent = 0U;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t ret;

    msgMotor.msgType = MSG_MOTOR_ADC_VALUES;
    msgMotor.data.adcValues.adc_power = adc_values[0];
    msgMotor.data.adcValues.adc_current = adc_values[1];
    ret = xQueueSendToBackFromISR(msgQueueMotor,
                                  &msgMotor,
                                  &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    (void)ret;

    return;
}