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
#define AP_KI 1.0F /* Integral coefficient */
#define AP_KD 1.0F /* Derivative coefficient */

/* minimum angle to send an order to motor */
#define AP_MOTOR_THRESHOLD (0.2F * (M_PI / 180.F))

#define DB_PRINT_ORDERS(X) (X)
#define DB_PRINT_MEMS_MSGS(X) (X)

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
#include "mems.h"
#include "autopilot.h"

void timerAPCallback(TimerHandle_t xTimer);

#if 0
static TimerHandle_t timerAP;
#endif
APStatus_t APStatus;
QueueHandle_t msgQueueAutoPilot;

void AP_init(APStatus_t *aps);
int AP_set_mode_idle(APStatus_t *aps);
int AP_set_mode_heading(APStatus_t *aps);
int AP_turn(APStatus_t *aps, float angle);
int AP_get_engaged(APStatus_t *aps);
float AP_get_heading_dir(APStatus_t *aps);
int AP_new_values(APStatus_t *aps, float deltat, float heading, float yawRate);
int AP_compute(APStatus_t *aps);

int autopilot_sendValues(TickType_t timeStamp, float heading, float roll, float pitch, float yawRate)
{

    MsgAutoPilot_t msg = {
        .msgType = AP_MSG_AHRS,
        .data.IMUData.heading = heading,
        .data.IMUData.roll = roll,
        .data.IMUData.pitch = pitch,
        .data.IMUData.yawRate = yawRate};

    if (xQueueSend(msgQueueAutoPilot, &msg, pdMS_TO_TICKS(0)) != pdPASS)
    {
        return -1; // Failed to send message
    }

    return 0; // Success
}

int init_taskAutoPilot(void)
{

    AP_init(&APStatus);

    /* Create incoming queue */
    msgQueueAutoPilot = xQueueCreate(10, sizeof(MsgAutoPilot_t));
    if (msgQueueAutoPilot == (QueueHandle_t)0)
    {
        return -1;
    }

#if 0
    /* create tick timer */
    timerAP = xTimerCreate("MEMs",
                           pdMS_TO_TICKS(AP_PERIOD_TICKS),
                           pdTRUE,    /* Auto reload (repeat indefinitely) */
                           (void *)0, /* Timer ID, not used */
                           timerAPCallback);
    if (timerAP == (TimerHandle_t)0)
    {
        return -1;
    }
#endif

    return 1;
}

#if 0
void timerAPCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

    static MsgAutoPilot_t command = {
        .msgType = AP_MSG_TICK};

    xQueueSend(msgQueueAutoPilot, (const void *)&command, (TickType_t)0);

    return;
}
#endif

void __attribute__((noreturn)) taskAutoPilot(void *args __attribute__((unused)))
{
    MsgAutoPilot_t msg;
    long unsigned int compteur = 0L;
    static char message[100];
    int nbcar;
    unsigned memsdata = 0;
    float deltat;
    BaseType_t timestamp, timestamp_2;

    // xTimerStart(timerAP, 0);
    timestamp = xTaskGetTickCount();

    for (;;)
    {
        if (xQueueReceive(msgQueueAutoPilot, &msg, portMAX_DELAY) == pdPASS)
        {
            // nbcar = 0;
            // message[0] = '\0';
            // nbcar = snprintf(message, sizeof(message) - 1, "AP: type message  %d\n", msg.msgType);
            // USART_write(usart1, message, nbcar, 0U);

            switch (msg.msgType)
            {
            case AP_MSG_MODE_HEADING:
                AP_set_mode_heading(&APStatus);
                DB_PRINT_ORDERS((
                    nbcar = snprintf(message, sizeof(message), "AP Mode heading %.1f\n", AP_get_heading_dir(&APStatus)),
                    svc_UART_Write(&svc_uart2, message, nbcar, 0U)));
                timestamp = xTaskGetTickCount();
                break;

            case AP_MSG_MODE_IDLE:
                DB_PRINT_ORDERS((
                    nbcar = snprintf(message, sizeof(message), "AP Mode idle\n"),
                    svc_UART_Write(&svc_uart2, message, nbcar, 0U)));
                AP_set_mode_idle(&APStatus);
                break;

            case AP_MSG_TURN:
                AP_turn(&APStatus, msg.data.reqTurnAngle * (-M_PI / 180.F));
                if (AP_get_engaged(&APStatus))
                {
                    DB_PRINT_ORDERS((
                        nbcar = snprintf(message, sizeof(message), "AP Mode heading %8f\n", AP_get_heading_dir(&APStatus)),
                        svc_UART_Write(&svc_uart2, message, nbcar, 0U)));
                }
                else
                {
                    DB_PRINT_ORDERS((
                        nbcar = snprintf(message, sizeof(message), "AP turn %8f\n", msg.data.reqTurnAngle),
                        svc_UART_Write(&svc_uart2, message, nbcar, 0U)));
                }
                break;

            case AP_MSG_AHRS:
                // nbcar = snprintf(message, sizeof(message) - 1, "AP AHRS %8f\n", cvt_dir_rad_deg(msg.data.IMUData.heading));
                // USART_write(usart1, message, nbcar, 0U);
                memsdata++;
                timestamp_2 = xTaskGetTickCount();
                deltat = ((float)(timestamp_2 - timestamp)) / ((float)configTICK_RATE_HZ);
                timestamp = timestamp_2;
                AP_new_values(&APStatus, deltat, msg.data.IMUData.heading, msg.data.IMUData.yawRate);

                DB_PRINT_MEMS_MSGS((
                    snprintf(message, sizeof(message) - 1,
                             "AP AHRS %8f  %8f %8f %8f %8f %8f %8f\n",
                             deltat,
                             cvt_dir_rad_deg(msg.data.IMUData.heading),
                             msg.data.IMUData.roll,
                             msg.data.IMUData.pitch,
                             msg.data.IMUData.yawRate,
                             APStatus.currentGap,
                             APStatus.integratedGap),
                    svc_UART_Write(&svc_uart2, message, strlen(message), 0)));
                // svc_UART_Write(&svc_uart2, "AP 00\n", 6, 0);
                // svc_UART_Write(&svc_uart2, "AP 01\n", 6, 0);
                // svc_UART_Write(&svc_uart2, "AP 02\n", 6, 0);
                // svc_UART_Write(&svc_uart2, "AP 03\n", 6, 0);

                break;

            case AP_MSG_TICK:
                break;

            case AP_MSG_PARAM:
                DB_PRINT_ORDERS((
                    nbcar = snprintf(message, sizeof(message) - 1, "AP: param %d %f\n", (int)msg.data.coefficient.param_number, msg.data.coefficient.param_value),
                    svc_UART_Write(&svc_uart2, message, nbcar, 0U)));
                // APStatus.headingToSteer = msg.data.reqHeading;
                switch (msg.data.coefficient.param_number)
                {
                case AP_PARAM_PROPORTIONNAL:
                    APStatus.kp = msg.data.coefficient.param_value;
                    break;
                case AP_PARAM_INTEGRAL:
                    APStatus.ki = msg.data.coefficient.param_value;
                    break;
                case AP_PARAM_DERIVATIVE:
                    APStatus.kd = msg.data.coefficient.param_value;
                    break;
                default:
                    break;
                }
                break;

            case AP_MSG_CALIBRATE_MEMS:
                if (APStatus.engaged)
                {
                    DB_PRINT_ORDERS((
                        nbcar = snprintf(message, sizeof(message) - 1, "AP: MEMS calibrate impossible while AP engaged\n"),
                        svc_UART_Write(&svc_uart2, message, nbcar, 0U)));
                }
                else
                {
                    // nbcar = snprintf(message, sizeof(message) - 1, "AP: calibrate MEMS\n");
                    // svc_UART_Write(&svc_uart2, message, nbcar, 0U);
                    static MEMS_Msg_t msgMEMs = {.msgType = MEMS_MSG_CALIBRATE};
                    xQueueSend(msgQueueMEMs, &msgMEMs, pdMS_TO_TICKS(10));
                }
                break;

            case AP_MSG_MEMS_READY:
                DB_PRINT_MEMS_MSGS((
                    nbcar = snprintf(message, sizeof(message) - 1, "AP: MEMS ready\n"),
                    svc_UART_Write(&svc_uart2, message, nbcar, 0U)));
                APStatus.MEMsReady = 1;
                break;

            default:
                break;
            }
        }
        compteur++;
    }
}

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

    /*Motor parameters*/
    aps->motorThreshold = AP_MOTOR_THRESHOLD;
    aps->steerAngle = 0.F;

    return;
}

/**
 * @brief Set Idle mode
 * @param APStatus_t *aps AutoPilot structure
 * @return none */

int AP_set_mode_idle(APStatus_t *aps)
{
    aps->engaged = 0;
    aps->currentGap = 0.F;
    aps->integratedGap = 0.F;

    MOTOR_disengage();

    return 0;
}

int AP_set_mode_heading(APStatus_t *aps)
{
    /* change state engage to 1 */
    /* No order to do since heading to steer is actual heading */
    if (aps->engaged == 0)
    {
        aps->engaged = 1;
        aps->headingToSteerRadians = aps->currentHeading;
        aps->currentGap = 0;
        aps->integratedGap = 0;
        aps->steerAngle = 0.F;

        MOTOR_engage();
    }
    /* else : Already engaged, Nothing to do*/
    return 0;
}

int AP_turn(APStatus_t *aps, float angle)
{
    if (aps->engaged == 0)
    {
        MOTOR_move_time(angle >= 0. ? -.2F : .2F);
    }
    else
    { /* AP idle, send move order to motor task */
        aps->headingToSteerRadians += angle;
        // AP_compute(aps);
    }
    return 1;
}

int AP_get_engaged(APStatus_t *aps)
{
    return aps->engaged;
}

float AP_get_heading_dir(APStatus_t *aps)
{
    return aps->headingToSteerRadians;
}

int AP_new_values(APStatus_t *aps, float deltat, float heading, float yawRate)
{
    int nbcar;
    static char message[120];
    float steerReq;

    aps->currentHeading = heading;
    if (aps->engaged)
    {
        aps->currentGap = (heading - aps->headingToSteerRadians);
        aps->integratedGap += aps->currentGap * deltat;
        aps->yawRate = yawRate;

        steerReq = aps->currentGap * aps->kp + aps->integratedGap * aps->ki + aps->yawRate * aps->kd;

        nbcar = snprintf(message, sizeof(message) - 1, "AP GAP %8f  %8f  %8f  %8f\n", aps->currentGap, aps->integratedGap, aps->yawRate, steerReq);
        svc_UART_Write(&svc_uart2, message, nbcar, 0U);

        if (fabsf(steerReq - aps->steerAngle) > aps->motorThreshold)
        {
            MOTOR_move_angle(steerReq);
            aps->steerAngle += steerReq;
        }
    }

    return 0;
}
