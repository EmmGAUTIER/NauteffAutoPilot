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

#include "util.h"
#include "printf.h"
#include "rlib.h"
#include "service.h"
//#include "aux_usart.h"

#include "autopilot.h"
#include "motor.h"
#include "apdialog.h"

#define MOTOR_V_CURRENT_NONE (0.F)
#define MOTOR_V_CURRENT_FREE (.35F)
#define MOTOR_V_CURRENT_BLOCKED (5.F)
#define ADC_PERIOD (0.01F)        /* 10 ms */
#define MOTOR_TIME_TO_STOP (0.1F) /* 100 ms */
#define MOTOR_CVT_ANGLE_TIME (1.0F)
#define ADC_CVT_TO_VOLTAGE (0.0097F)
#define ADC_CVT_TO_CURRENT (0.002F) /* Ratio  ADC val. and current */

typedef enum
{
    MotorIdle,
    MotorMoveAngle,
    MotorMoveTime,
    MotorStopping,
    MotorDefault,
    MotorBlocked
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
    int blocked;   /* 0 : unblocked, +1 : blocked port, -1  :blocked starboard */

    /* Data for duration and move */
    float turnAngleReq;  /* Turn angle requested (rad counterclockwise) */
    float turnAngleDone; /* Estimated turn angle done */
    float turnTimeReq;   /* Turn angle requested (rad counterclockwise) */
    float turnTimeDone;  /* Estimated turn angle done */
    float timeStopping;  /* Time since motor powered off */

    /* Values of calibration */
    float vcurrentNone;    /* adc value of current when not moving */
    float vcurrentFree;    /* adc value of current when moving with no effort */
    float vcurrentBlocked; /* adc value of current when motor blocked */
    float vPowerStandard;  /* Standard power voltage */
    float vPowerMin;       /* Minimum power voltage */
    float vPowerMax;       /* Maximum power voltage */
    float timeStartStop;   /* Time to start or stop the motor */

    /* measured values */
    float vPower;   /* Actual voltage*/
    float vCurrent; /* Actual current */
} MotorData;

MotorData motorData = {

    .status = MotorIdle,
    .engaged = 0,
    .direction = 0,

    .turnAngleReq = 0.F,
    .turnAngleDone = 0.F,
    .turnTimeReq = 0.F,
    .turnTimeDone = 0.F,

    .vcurrentNone = MOTOR_V_CURRENT_NONE,
    .vcurrentFree = MOTOR_V_CURRENT_FREE,
    .vcurrentBlocked = MOTOR_V_CURRENT_BLOCKED,
    .timeStartStop = 0.1F,

    .vPowerStandard = 12.F,
    .vPowerMin = 10.,
    .vPowerMax = 15.F,
    .vPower = 0.F,
    .vCurrent = 0.F

};

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

void trigger_adc_conversion(TimerHandle_t th)
{
    (void)th;

    // adc1->CR |= ADC_CR_ADSTART;
}

void sendADCValues(uint16_t v1, uint16_t v2)
{
    static MsgMotor_t msgMotor;
    static BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;

    msgMotor.msgType = MSG_MOTOR_ADC_VALUES;
    msgMotor.defaultCodes = 0;
    msgMotor.data.adcValues.adc_power = v1;
    msgMotor.data.adcValues.adc_current = v2;

    /* Send the message to the queue */
    /* check msgQueueMotor has been created before sending */
    if (msgQueueMotor != (QueueHandle_t)0)
    {
        xQueueSendFromISR(msgQueueMotor, &msgMotor, &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * \brief Motor control task
 */

void taskMotor(void *parameters)
{
    (void)parameters; /* parameters ignored, avoids warning */

    size_t ret;
    MsgMotor_t msgMoteur;
    MsgAutoPilot_t msgAutoPilot;
    char message[60];
    unsigned nbtours = 0;
    unsigned nb2 = 0;
    int status = 0;
    int dirdmd = 0;
    TimerHandle_t timerTriggerConv;
    // StaticTimer_t triggerTimerBuffer;

    sprintf(message, "taskMotor Départ\n");
    svc_UART_Write(&svc_uart1, message, strlen(message), 0U);

    /* TODO : Faire un déclenchement des conversions ADC par timer */
    /* Actuellement il faut le faire par un timer géré par freertos */
    timerTriggerConv = xTimerCreate("ADC trig.",
                                    pdMS_TO_TICKS(25),
                                    pdTRUE,
                                    0U,
                                    trigger_adc_conversion);

    if (timerTriggerConv == NULL)
    {
        sprintf(message, "taskMotor : timerTriggerConv creation failed\n");
        svc_UART_Write(&svc_uart1, message, strlen(message), 0U);
    }
    else
    {
        xTimerStart(timerTriggerConv, (TickType_t)0);
    }

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
                /* get ADC values */
                motorData.vPower = (float)msgMoteur.data.adcValues.adc_power * ADC_CVT_TO_VOLTAGE;
                motorData.vCurrent = (float)msgMoteur.data.adcValues.adc_current * ADC_CVT_TO_CURRENT;
                nb2++;

                /* For debugging purpose, prints values of ADC */
                if (nbtours <= 8)
                {
                    nbtours++;
                }
                else
                {
                    sprintf(message, "ADC %3d %f %f\n", nb2, motorData.vPower, motorData.vCurrent);
                    sprintf(message, "ADC %4d %6f %6f\n", nb2, motorData.vPower, motorData.vCurrent);
                    svc_UART_Write(&svc_uart1, message, strlen(message), 0U);
                    nbtours = 0;
                }

                /* Check Values */
                /* TODO */

                switch (motorData.status)
                {
                case MotorIdle:
                    break;

                case MotorMoveAngle:
                    motorData.turnAngleDone += MOTOR_CVT_ANGLE_TIME * ADC_PERIOD;
                    if (motorData.turnAngleDone >= motorData.turnAngleReq)
                    {
                        motorStop();
                        motorData.status = MotorStopping;
                        motorData.direction = 0;
                        motorData.turnAngleDone = 0.F;
                        motorData.turnAngleReq = 0.F;
                        motorData.timeStopping = 0.F;
                        sprintf(message, "Motor angle  stop  %f\n", motorData.turnAngleDone);
                        svc_UART_Write(&svc_uart1, message, strlen(message), 0U);
                    }
                    break;

                case MotorMoveTime:
                    motorData.turnTimeDone += ADC_PERIOD;
                    if (motorData.turnTimeDone >= motorData.turnTimeReq)
                    {
                        motorData.status = MotorIdle;
                        motorStop();
                        motorData.status = MotorStopping;
                        motorData.direction = 0;
                        motorData.turnTimeDone = 0.F;
                        motorData.turnTimeReq = 0.F;
                        motorData.timeStopping = 0.F;
                        sprintf(message, "Motor time  stop  %f\n", motorData.turnTimeDone);
                        svc_UART_Write(&svc_uart1, message, strlen(message), 0U);
                    }
                    break;

                case MotorStopping:
                    motorData.timeStopping += ADC_PERIOD;
                    if (motorData.timeStopping >= MOTOR_TIME_TO_STOP)
                    {
                        motorData.status = MotorIdle;
                        motorData.timeStopping = 0.F;
                        sprintf(message, "MOTOR stopped\n");
                        svc_UART_Write(&svc_uart1, message, strlen(message), 0U);
                        msgAutoPilot.msgType = AP_MSG_MOTOR_STOPPED;
                        msgAutoPilot.data.moveReport.effort = 0.;
                        xQueueSend(msgQueueAutoPilot, &msgAutoPilot, pdMS_TO_TICKS(0));
                    }
                    break;

                case MotorDefault:
                    /* Something to do */
                    break;

                case MotorBlocked:
                    /* Something to do */
                    break;

                default:
                    /* Shouldn't happen */
                    break;
                }
                break;

            case MSG_MOTOR_STOP:
                motorStop();
                motorData.timeStopping = 0.F;
                motorData.status = MotorStopping;
                break;

            case MSG_MOTOR_EMBRAYE:
                motorStop();
                motorTillerEngage();
                break;

            case MSG_MOTOR_DEBRAYE:
                motorStop();
                motorTillerDisengage();
                break;

            case MSG_MOTOR_ERROR:
                break;

            case MSG_MOTOR_MOVE_TIME:
                /*
                 * Motor Idle : ok
                 * MotorBlocked : ok if backward move ie blocked and direction opposite
                 * MotorMoveAngle : let it turn
                 * MotorMoveTime : ok if direction is the same, adds time
                 * MotorStopping : ok if direction is the same, like idle
                 * MotorDefault : no move
                 */
                status = motorData.status;
                dirdmd = msgMoteur.data.moveTime > 0.F ? 1 : -1;
                if ((status == MotorIdle) ||
                    (status == MotorMoveTime && dirdmd == motorData.direction) ||
                    (status == MotorStopping && dirdmd != motorData.direction) ||
                    (status == MotorBlocked && dirdmd != motorData.direction))
                {
                    motorData.status = MotorMoveTime;
                    motorData.direction = dirdmd;
                    motorData.turnTimeReq = msgMoteur.data.moveTime * motorData.direction;
                    motorData.turnTimeDone = 0.F;
                    /* Run order useless if motor still running */
                    (motorData.direction == 1) ? motorRunToPort() : motorRunToStarboard();

                    sprintf(message, "MOTOR turn %s time %f\n", motorData.direction > 0 ? "port" : "starboard", motorData.turnTimeReq);
                    svc_UART_Write(&svc_uart1, message, strlen(message), 0U);
                }
                break;

            case MSG_MOTOR_MOVE_ANGLE:
                if (motorData.status == MotorIdle)
                {
                    motorData.status = MotorMoveAngle;
                    motorData.direction = msgMoteur.data.moveAngle > 0.F ? 1 : -1;
                    motorData.turnAngleReq = msgMoteur.data.moveAngle * motorData.direction;
                    motorData.turnAngleDone = 0.F;
                    motorData.direction > 0 ? motorRunToPort() : motorRunToStarboard();

                    sprintf(message, "MOTOR turn %s angle %f\n", motorData.direction > 0 ? "port" : "starboard", motorData.turnAngleReq);
                    svc_UART_Write(&svc_uart1, message, strlen(message), 0U);
                }
                break;

            default:
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); /* Délai de garde pour éviter une famine */
    }
}
