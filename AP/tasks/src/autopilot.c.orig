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

/* PID coefficients */
#define AP_KP 1.0F /* Proportional coefficient */
#define AP_KI 0.0F /* Integral coefficient */
#define AP_KD 0.0F /* Derivative coefficient */

/* Maximum integrated angle */
#define AP_MAX_INTEGRAL_GAP  (30.F * (M_PI / 180.))

 /* Low pass filter coefficient for yaw rate */
#define AP_LPF_YAW_RATE 0.8F

#define AP_TIME_ONE_MOVE (0.2F) /* time to move for one order to motor in seconds */

#define DB_PRINT_ORDERS(X) (X)
#define DB_PRINT_MEMS_MSGS(X) (X)
#define DB_PRINT_PID(X) (X)

#include "math.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "message_buffer.h"
#include "stream_buffer.h"

#include "util.h"
#include "printf.h"
#include "rlib.h"
#include "geom.h"

#include <stm32l452xx.h>
#include <stm32l4xx_ll_gpio.h>

#include "service.h"

#include "motor.h"
#include "ahrs.h"
#include "mems.h"
#include "autopilot.h"

void timerAPCallback(TimerHandle_t xTimer);

APStatus_t APStatus;
QueueHandle_t msgQueueAutoPilot;

/* Names of PID parameters,
 * must be coherent with APParam_t declared in autopilot.h */
const char *APParameterNames[] = {
    "Kp",
    "Ki",
    "Kd"
};

void AP_init(APStatus_t *aps);
int AP_set_mode_idle(APStatus_t *aps);
int AP_set_mode_heading(APStatus_t *aps);
int AP_turn(APStatus_t *aps, float angle);
int AP_get_engaged(APStatus_t *aps);
float AP_get_heading_dir(APStatus_t *aps);
int AP_new_values(APStatus_t *aps, float deltat, float heading, float yawRate);
int AP_compute(APStatus_t *aps);

int AP_MSG_send_AHRS_values(TickType_t timeStamp, float heading, float roll, float pitch, float yawRate)
{

    MsgAutoPilot_t msg =
    {
        .msgType = AP_MSG_AHRS,
        .data.IMUData.heading = heading,
        .data.IMUData.roll = roll,
        .data.IMUData.pitch = pitch,
        .data.IMUData.yawRate = yawRate
    };

    if(xQueueSend(msgQueueAutoPilot, &msg, pdMS_TO_TICKS(0)) != pdPASS)
    {
        return -1; // Failed to send message
    }

    return 0; // Success
}

int AP_MSG_MotorStalled()
{
    MsgAutoPilot_t msg =
    {
        .msgType = AP_MSG_MOTOR_STALLED,
    };

    if(xQueueSend(msgQueueAutoPilot, &msg, pdMS_TO_TICKS(0)) != pdPASS)
    {
        return -1; // Failed to send message
    }

    return 1;
}

int AP_MSG_MotorStopped()
{
    MsgAutoPilot_t msg =
    {
        .msgType = AP_MSG_MOTOR_STALLED,
    };

    if(xQueueSend(msgQueueAutoPilot, &msg, pdMS_TO_TICKS(0)) != pdPASS)
    {
        return -1; // Failed to send message
    }

    return 1;
}

int AP_MSG_Select_AHRS(AHRS_Types ahrsType)
{
    MsgAutoPilot_t msg =
    {
        .msgType = AP_MSG_AHRS_SELECT,
        .data.ahrsType = ahrsType
    };

    if(xQueueSend(msgQueueAutoPilot, &msg, pdMS_TO_TICKS(0)) != pdPASS)
    {
        return -1; // Failed to send message
    }

    return 1;
}

int AutoPilot_task_init(void)
{

    AP_init(&APStatus);

    /* Create incoming queue */
    msgQueueAutoPilot = xQueueCreate(10, sizeof(MsgAutoPilot_t));

    if(msgQueueAutoPilot == (QueueHandle_t)0)
    {
        return -1;
    }

    return 1;
}

void __attribute__((noreturn)) AutoPilot_task(void *args __attribute__((unused)))
{
    MsgAutoPilot_t msg;
    long unsigned int compteur = 0L; /* for debugging purpose only */
    static char message[200];
    int nbcar = 0;
    unsigned memsdata = 0;
    float deltat;  /* delta time (seconds) between to AHRS data messages */
    BaseType_t timestamp, timestamp_2;

    // xTimerStart(timerAP, 0);
    timestamp = xTaskGetTickCount();

    for(;;)
    {
        if(xQueueReceive(msgQueueAutoPilot, &msg, portMAX_DELAY) == pdPASS)
        {

            switch(msg.msgType)
            {

            case AP_MSG_AHRS:

                memsdata++;
                timestamp_2 = xTaskGetTickCount();
                deltat = ((float)(timestamp_2 - timestamp)) / ((float)configTICK_RATE_HZ);
                timestamp = timestamp_2;
                AP_new_values(&APStatus, deltat, msg.data.IMUData.heading, msg.data.IMUData.yawRate);

                DB_PRINT_MEMS_MSGS((snprintf(message, sizeof(message) - 1,
                                                "AP AHRS %6.3f  %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\n",
                                                deltat,
                                                cvt_dir_rad_deg(msg.data.IMUData.heading),
                                                msg.data.IMUData.roll * (180. / M_PI),
                                                msg.data.IMUData.pitch * (180. / M_PI),
                                                msg.data.IMUData.yawRate * (180. / M_PI),
                                                APStatus.currentGap * (180. / M_PI),
                                                APStatus.integratedGap * (180. / M_PI)),
                                       svc_UART_Write(&svc_uart2, message, strlen(message), 0)));

                break; /* case AP_MSG_AHRS: */

            case AP_MSG_MODE_HEADING:

                AP_set_mode_heading(&APStatus);
                DB_PRINT_ORDERS((nbcar = snprintf(message, sizeof(message), "AP Mode heading %.1f\n", AP_get_heading_dir(&APStatus)),
                                 svc_UART_Write(&svc_uart2, message, nbcar, 0U)));
                timestamp = xTaskGetTickCount();

                break; /* case AP_MSG_MODE_HEADING */

            case AP_MSG_MODE_IDLE: /* set mode idle */

                DB_PRINT_ORDERS((nbcar = snprintf(message, sizeof(message), "AP Mode idle\n"),
                                 svc_UART_Write(&svc_uart2, message, nbcar, 0U)));
                AP_set_mode_idle(&APStatus);

                break; /* case AP_MSG_MODE_IDLE */

            case AP_MSG_TURN:

                AP_turn(&APStatus, msg.data.reqTurnAngle * (- M_PI / 180.F));

                DB_PRINT_ORDERS((nbcar = snprintf(message, sizeof(message), "AP turn %8f\n", msg.data.reqTurnAngle),
                                 svc_UART_Write(&svc_uart2, message, nbcar, 0U)));

                break; /* case AP_MSG_TURN */

            case AP_MSG_TICK:

                break; /* case AP_MSG_TICK: */

            case AP_MSG_MOTOR_STALLED:

                break; /* case AP_MSG_MOTOR_STALLED */

            case AP_MSG_PARAM:

                DB_PRINT_ORDERS((nbcar = snprintf(message, sizeof(message) - 1, "AP param %s %f\n",
                                                  APParameterNames[(int)msg.data.coefficient.param_number],
                                                  msg.data.coefficient.param_value),
                                 svc_UART_Write(&svc_uart2, message, nbcar, 0U)));

                // APStatus.headingToSteer = msg.data.reqHeading;
                switch(msg.data.coefficient.param_number) {

                case AP_PARAM_KP:

                    APStatus.kp = msg.data.coefficient.param_value;

                    break; /* case AP_PARAM_KP: */

                case AP_PARAM_KI:

                    APStatus.ki = msg.data.coefficient.param_value;

                    break; /* case AP_PARAM_KI */

                case AP_PARAM_KD:

                    APStatus.kd = msg.data.coefficient.param_value;

                    break; /* case AP_PARAM_KD: */

                default:
                    break; /* default: */
                }

                break; /* case AP_MSG_PARAM */

            case AP_MSG_START_CALIBRATE:

                if(APStatus.engaged)
                {
                    DB_PRINT_ORDERS((nbcar = snprintf(message, sizeof(message) - 1, "AP MEMS calibrate impossible while AP engaged\n"),
                                     svc_UART_Write(&svc_uart2, message, nbcar, 0U)));
                }
                else
                {
                    DB_PRINT_ORDERS((nbcar = snprintf(message, sizeof(message) - 1, "AP: calibrate MEMS\n"),
                                     svc_UART_Write(&svc_uart2, message, nbcar, 0U)));
                    static MEMS_Msg_t msgMEMs = {.msgType = MEMS_MSG_CALIBRATE};
                    xQueueSend(msgQueueMEMs, &msgMEMs, pdMS_TO_TICKS(10));
                }

                break; /* case AP_MSG_START_CALIBRATE: */

            case AP_MSG_MEMS_READY:

                DB_PRINT_MEMS_MSGS((
                                       nbcar = snprintf(message, sizeof(message) - 1, "AP MEMS ready\n"),
                                       svc_UART_Write(&svc_uart2, message, nbcar, 0U)));
                APStatus.MEMsReady = 1;

                break; /* case AP_MSG_MEMS_READY: */

            case AP_MSG_DISPLAY_CONFIG:

                nbcar = snprintf(message, sizeof(message) - 1,
                                  "AP config: Kp %8f  Ki %8f  Kd %8f\n",
                                  APStatus.kp,
                                  APStatus.ki,
                                  APStatus.kd,
                svc_UART_Write(&svc_uart2, message, nbcar, 0U));

                break; /* case AP_MSG_DISPLAY_CONFIG: */
            
            case AP_MSG_AHRS_SELECT: /* Change type of AHRS algorithm */

                if ( ! APStatus.engaged)
                {
                    snprintf(message, sizeof(message) - 1, "AP select AHRS type %d\n", (int)msg.data.ahrsType);
                    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
                    static MEMS_Msg_t msgMEMs = {.msgType = MEMS_MSG_SET_AHRS_TYPE};
                    msgMEMs.data.ahrsType = msg.data.ahrsType;
                    xQueueSend(msgQueueMEMs, &msgMEMs, pdMS_TO_TICKS(10));
                }

                break; /* case AP_MSG_AHRS_SELECT: */

            default:

                break; /* default: */
            }
        }

        compteur++;
    }
}

/****************************************************************************\
*     Functions manage the autopilot                                         *
******************************************************************************
*                                                                            *
*                                                                            *
*                                                                            *
* The prototypes of theses function is :                                     *
* int | void AP_XXX (APStatus_t aps, ...)                                    *
* They use AP status stored in an APStatus_t struct.                         *
*                                                                            *
*                                                                            *
*                                                                            *
\****************************************************************************/

/*
 * @brief Initialize AP status
 * @param APStatus_t *aps pointer to AP status structure to initialize
 * @return none
 * This function musr bes called to initialise the AP status
 * before using any other AP function.
 */
void AP_init(APStatus_t *aps)
{
    aps->MEMsReady = 0;
    aps->engaged = 0;
    aps->headingToSteerDegrees = 0;
    aps->headingToSteerRadians = 0.;
    aps->currentHeading = 0;
    aps->currentGap = 0;
    aps->integratedGap = 0;
    aps->heading = 0;
    aps->memorizedHeading = 0;
    aps->roll = 0;
    aps->pitch = 0;
    aps->yawRate = 0;
    aps->rollRate = 0;
    aps->pitchRate = 0;

    /* PID parameters */
    aps->kd = AP_KD;
    aps->ki = AP_KI;
    aps->kp = AP_KP;

    return;
}

/*
 * @brief Set Idle mode
 * @param APStatus_t *aps AutoPilot structure
 * @return none
 */
int AP_set_mode_idle(APStatus_t *aps)
{
    aps->engaged = 0;
    aps->currentGap = 0.F;
    aps->integratedGap = 0.F;

    MOTOR_MSG_letOutClutch();

    return 0;
}

/*
 * @brief Set heading mode (maintaining heading)
 * @param APStatus_t *aps AutoPilot structure
 * @return none
 */
int AP_set_mode_heading(APStatus_t *aps)
{
    /* change state engage to 1 */
    /* No order to send to motor since heading to steer is actual heading */
    if(aps->engaged == 0)
    {
        aps->engaged = 1;
        aps->headingToSteerRadians = aps->currentHeading;
        aps->currentGap = 0;
        aps->integratedGap = 0;

        MOTOR_MSG_letInClutch();
    }

    /* else : Already engaged, Nothing to do*/
    return 0;
}

/*
 * @brief Turn helm or change direction to steer
 * @param APStatus_t *aps AutoPilot structure
 * @param angle Angle to turn in radians counterclockwise
 * If autopilot is disengaged, send order to motor task to move for a time
 * If autopilot is engaged, update heading to steer
 */
int AP_turn(APStatus_t *aps, float angle)
{
    if(aps->engaged == 0)
    {
        MOTOR_MSG_moveTime(angle >= 0. ? - AP_TIME_ONE_MOVE : AP_TIME_ONE_MOVE);
    }
    else
    {
        /* AP idle, send move order to motor task */
        aps->headingToSteerRadians += angle;
        // AP_compute(aps);
    }

    return 1;
}

/*
 * @brief Get AP engaged or not
 * @param APStatus_t *aps AutoPilot structure
 * @return 1 if AP is engaged, 0 if AP is idle
 */
int AP_get_engaged(APStatus_t *aps)
{
    return aps->engaged;
}

/*
 * @brief Get heading to steer in radians
 * @param APStatus_t *aps AutoPilot structure
 * @return heading to steer in radians
 */
float AP_get_heading_dir(APStatus_t *aps)
{
    return aps->headingToSteerRadians;
}

/*
 * @brief Update AP status with new values and compute new orders
 * @param APStatus_t *aps AutoPilot structure
 * @param deltat time since last update in seconds
 * @param heading current heading in radians
 * @param yawRate current yaw rate in radians/s
 * @return 0
 * This function is called by taskAutoPilot when it receives new AHRS values.
 * It updates the AP status with new values
 * and computes new orders to send to motor task if needed.
 */
int AP_new_values(APStatus_t *aps, float deltat, float heading, float yawRate)
{
    int nbcar = 0;
    static char message[120];
    float steerReq; /* halm angle to steer, sent to motor task*/
    /*
     * The orders to send to motor task are computed with a PID controller.
     * The gap is the difference between heading to steer and current heading.
     * The integrated gap is the integral of the gap since the last time it was reset.
     * The order to send to motor task is the sum of a proportionnal term, an integral term and a derivative term.
     * The proportionnal term is the gap multiplied by Kp
     * The integral term is the integrated gap multiplied by Ki
     * The derivative term is the yaw rate multiplied by Kd
     * The coefficients are stored in APStatus_t structure and can be updated by sending a message to AP task.
     * The order to send to motor task is the sum of the three terms and is sent to motor task by calling MOTOR_MSG_setHelmAngle() function.
     * The order to send to motor task is also stored in APStatus_t structure for information.
    */

    aps->currentHeading = heading;

    if(aps->engaged)
    {
        aps->currentGap = (aps->headingToSteerRadians - heading);
        aps->integratedGap += aps->currentGap * deltat;
        //aps->yawRate = -yawRate;
        aps->yawRate = aps->yawRate * (1.0F - AP_LPF_YAW_RATE) -yawRate * AP_LPF_YAW_RATE;

        /* Integrated gap is maintained in [ - AP_MAX_INTEGRAL_GAP +AP_MAX_INTEGRAL_GAP] */
        if(aps->integratedGap > AP_MAX_INTEGRAL_GAP)
        {
            aps->integratedGap =  AP_MAX_INTEGRAL_GAP;
        }

        if(aps->integratedGap < -AP_MAX_INTEGRAL_GAP)
        {
            aps->integratedGap = -AP_MAX_INTEGRAL_GAP;
        }

        steerReq = (aps->currentGap * aps->kp)      /* Proportional */
                   + (aps->integratedGap * aps->ki) /* Integral */
                   + (aps->yawRate * aps->kd);      /* Derivative */

        DB_PRINT_PID((nbcar = snprintf(message, sizeof(message) - 1,
                                          "AP PID %8f %8f %8f %8f\n",
                                          aps->currentGap,
                                          aps->integratedGap,
                                          aps->yawRate,
                                          steerReq),
                         svc_UART_Write(&svc_uart2, message, nbcar, 0U)));

        MOTOR_MSG_setHelmAngle(steerReq);

    }

    return 0;
}