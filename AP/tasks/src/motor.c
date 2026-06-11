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

/****************************************************************************\
 *             Contrôle du moteur et de l'embrayage                          *
 *****************************************************************************
 *                                                                           *
 * Ces fonction permettent le contrôle de l'actuateur, elles assurent :      *
 *   - la commande d'embrayage ;                                             *
 *   - la commande du moteur pendant un temps imparti ou un angle estimé ;   *
 *   - la mesure et la surveillance du courant quand le moteur tourne ;      *
 *   - la mesure et  la surveillance de la tension d'alimentation ;          *
 *   - l'arrêt du moteur en cas de surcharge, de court-cicuit ou             *
 *     de détection de buttée ;                                              *
 *   - Une évaluation de la position de la barre.                            *
 *                                                                           *
 * La position de barre est estimée avec le courant et la tension            *
 * d'alimentation; le contrôle de l'actuateur est réalisé par une tâche.     *
 * Elle reçoit des ordres et informations par une file de messages (queue).  *
 * La fonction réflexe HAL_ADC_ConvCpltCallback envoie les valeurs de tension*
 * et de courant à la tâche moteur par cette file de messages.               *
 * Les fonction MOTER_MSG_XXX() envoient les messages à la tâche moteur.     *
 * Les Fonctions de bas niveau MOTOR_LL_XXX() commandent l'actuateur et      *
 * l'embrayage.                                                              *
 * L'état du moteur est stocké dans une structure MotorData,                 *
 * Des fonction Motor_XXX() mettent à jour cette structure et commandent     *
 * l'actuateur et l'embrayage.                                               *
 * Rappel : L'angle de barre est positif vers tribord et négatif à babord.   *
 *                                                                           *
 \***************************************************************************/

#include <math.h>

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_gpio.h"
#include <stdint.h>
#include <stm32l452xx.h>

#include "printf.h"
#include "rlib.h"
#include "service.h"
#include "util.h"

#include "apdialog.h"
#include "autopilot.h"
#include "motor.h"

/* Définition de M_PI, il est parfois non défini */
#ifndef M_PI
#define M_PI ((float)3.14159265358979323846)
#endif

/*
 * Affichages de messages de mise au point
 */

#define DBG_MOTOR_PRINT(X) (X)
#define DBG_MOTOR_LL_PRINT(X) (X)
#define DBG_ADC_PRINT(X) (X)

/****************************************************************************\
*                 Default values for parameters                              *
*  Rappel : 2 PI / 360 = 0,01745329..                                        *
*****************************************************************************/

#define MOTOR_HPF_COEF       (0.0F)
#define MOTOR_THRESHOLD      (1.0F * ((float)M_PI / 180.F)) /* threshold 1 deg. */
#define MOTOR_CVT_ANGLE_TIME (2.0F) /* Estimated conversion between time and helm move angle */
#define MOTOR_TIME_START     (0.1F) /* Maximum time to allow over current when starting motor (s) */
#define MOTOR_TIME_STOP      (0.5F) /* Time to wait for motor to stop before opposite move order (s) */

#define MOTOR_EVENT_STALLED  (0x1 << 0)
#define MOTOR_EVENT_STOP     (0x1 << 1)
#define MOTOR_EVENT_STOPPING (0x1 << 2)

//#define DELTA_ANGLE_NEAR (0.5F * ( (float)M_PI / 180.F))      /* 0.5 degree */
//#define DELTA_ANGLE_THRESHOLD (0.5F * ( (float)M_PI / 180.F)) /* 0.5 degree */

#define MOTOR_V_CURRENT_NONE    (0.F)
#define MOTOR_V_CURRENT_FREE    (.1F)
#define MOTOR_V_CURRENT_BLOCKED (1.0F)     /*  */
#define MOTOR_V_POWER_STANDARD  (12.0F)
#define ADC_PERIOD (0.01F)                 /* 100 ms */
#define MOTOR_TIME_TO_STOP (0.1F)          /* 100 ms */
#define MOTOR_MAX_TIME_OVERCURRENT (0.02F) /* 20 ms */
#define ADC_CVT_TO_VOLTAGE (0.0091F) /* Ratio ADC val. and power voltage */
#define ADC_CVT_TO_CURRENT (0.0004F) /* Ratio  ADC val. and current */

/* status bits of motor */
/* More than minimum required so easier to code and test */
#define MOTOR_STATUS_ENGAGED (0x1 << 0)       /* 0 clutch out, 1 : clutch in*/
#define MOTOR_STATUS_MOVING_TIME (0x1 << 1)   /* moving for time */
#define MOTOR_STATUS_MOVING_ANGLE (0x1 << 2)  /* moving helm to angle */
#define MOTOR_STATUS_DIR_STARBOARD (0x1 << 3) /* moving to starboard */
#define MOTOR_STATUS_DIR_PORT (0x1 << 4)      /* moving to port*/
#define MOTOR_STATUS_IDLE (0x1 << 5)          /* motor idle i.e. stopped */
#define MOTOR_STATUS_RUNNING (0x1 << 6)       /* motor running */
#define MOTOR_STATUS_STOPPING (0x1 << 7)      /* motor stopping */
#define MOTOR_STATUS_STALLED (0x1 << 8)       /* motor stalled */
#define MOTOR_STATUS_STARTING_RUN (0x1 << 9)  /* motor starting to run ie accelerating */

/*
 * Global vars
 */
/* Message queue for Motor task */
static QueueHandle_t msgQueueMotor = (QueueHandle_t)0;
/* Device handles of timer and ADC */
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

/* Every ADC_PERIOD timer TIM3 triggers two ADC conversions (Vpower and Imot),
 * then ADC do the conversions,  and fire
 * an interrupt handler that stores the values in adc_values[]
 * that has to be common to interrupt handler and task_motor.
 */
static uint16_t adc_values[2];

/*****************************************************************************\
*       Low level commands of motor and clutch                                *
 ******************************************************************************
 *                                                                            *
 *   Motor and clutch commands are connected to GPIOA pins as follows :       *
 *   - PA4 : motor command,                                                   *
 *   - PA5 : clutch command, optional, connected to green LED on Nucleo board *
 *   - PA6 : INA, motor direction                                             *
 *   - PA7 : INB, motor direction                                             *
 *                                                                            *
 * These functions are low level functions that set and reset GPIO pins       *
 * to run the motor and control the clutch.                                   *
 * Theses functions are called by higher level functions.                     *
 * they have the prefix Motor_LL_, their protoype is :                        *
 * void Motor_LL_(void)                                                       *
 * They use LL_GPIO_[Re]setOutputPin functions.                               *
 *                                                                            *
 * Note:The motor is run to starboard when the INA pin is set to high and the *
 * INB pin is set to low. The motor is run to port when the INA pin is set to *
 * low and the INB pin is set to high. The motor is stopped when both INA and *
 * INB pins are set to low.                                                   *
 * Port is left side of boat and starboard is right side of boat              *
 * Those function do not check if orders are safe according to state of motor *
 * and just set the pins, it is the responsibility of higher level functions  *
 * to check if orders are safe.                                               *
 * Those function are only used by motor functions and are declared inline;   *
 * they are 'private' to motor.c                                              *
 * The device that controls the motor is a VNH5019 from ST Microelectronics.  *
 * see https://www.st.com/en/automotive-analog-and-power/vnh5019a-e.html      *
 * The motor is a brushed motor                                               *
 * Another function that accesses GPIO pins to control motor and clutch       *
 * stops the motor in case of panic.                                          *
 *                                                                            *
 \****************************************************************************/

/**
 * @brief Run the motor to port
 * This function sets the GPIO pins to run the motor to port.
 * It is called by taskMotor.
 * @param void
 * @return void
 */

INLINE static void Motor_LL_runToPort(void)
{
    /* Set PWN and INA, reset INB */

    LL_GPIO_ResetOutputPin(GPIOA,
                           LL_GPIO_PIN_4 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4 | LL_GPIO_PIN_6);

    DBG_MOTOR_LL_PRINT(svc_UART_Write(&svc_uart2, "MOTOR LL run to port\n", 21, 0U));
}

/**
 * @brief Run the motor to starboard
 * This function sets the GPIO pins to run the motor to starboard.
 * It is called by taskMotor.
 * @param void
 * @return void
 */
INLINE static void Motor_LL_runToStarboard(void)
{
    /* Set PWN and INB, reset INA */
    LL_GPIO_ResetOutputPin(GPIOA,
                           LL_GPIO_PIN_4 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4 | LL_GPIO_PIN_7);

    DBG_MOTOR_PRINT(
        svc_UART_Write(&svc_uart2, "MOTOR LL run to starboard\n", 26, 0U));
}

/**
 * @brief Stop the motor
 * This function sets the GPIO pins to stop the motor.
 * It is called by taskMotor or ADC interrupt if overcurrent is detected.
 * They deactivate PWM, INA and INB pins, outputs are free and motor stops. 
 * @param void
 * @return void
 */

INLINE static void Motor_LL_stop(void)
{
    /* Reset PWN, INA and INB */
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);

    DBG_MOTOR_PRINT(svc_UART_Write(&svc_uart2, "MOTOR LL stop\n", 14, 0U));
}

/**
 * @brief Engage the actuator.
 * This function sets the GPIO pin to engage the actuator.
 * pin is connected to the motor driver
 * It stops tho motor if it was running.
 * it is also connected to a green LED on the Nucleo board.
 * It is called by taskMotor.
 * @param void
 * @return void
 */

INLINE static void Motor_LL_engage_actuator(void)
{
    /* Stop motor if it was running */
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
    /* engage actuator */
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);

    DBG_MOTOR_PRINT(svc_UART_Write(&svc_uart2, "MOTOR LL engage actuator\n", 23, 0U));
}

/**
 * @brief Disengage the actuator
 * This function sets the GPIO pin to disengage the actuator.
 * @param void
 * @return void
 */

INLINE static void Motor_LL_disengage_actuator(void)
{
    /* Reset Clutch (and LED), INA, INB and motor */
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4 | LL_GPIO_PIN_5 |
                           LL_GPIO_PIN_6 | LL_GPIO_PIN_7);

    DBG_MOTOR_PRINT(svc_UART_Write(&svc_uart2, "MOTOR LL disengage actuator\n", 26, 0U));
}

/****************************************************************************\
*     Function to send orders to MOTOR task                                  *
 ******************************************************************************
 *                                                                            *
 * These functions send commands to the motor task by sending messages        *
 * to its message queue. They are a easy way for creating messages and        *
 * sending them to motor task without having to know message structure.       *
 *                                                                            *
 * Note : Messages to the task come from several tasks: mems, dialog and AP.  *
 * Messages are put in a queue by these functions.                            *
 *                                                                            *
 * They have no return value.                                                 *
 * Their prototype is :                                                       *
 * void Motor_msg_XXX(...)                                                    *
 *                                                                            *
 \****************************************************************************/

/*
 * @brief Send command to engage the actuator to motor task.
 * Upon reception of this command the motor task engages the clutch
 * and waits for steering angles from autopilot task.
 * @param void
 * @return void
 */
void Motor_msg_engage_actuator(void)
{
    char message [40];
    snprintf(message, sizeof(message), "MOTOR msg engage actuator\n");
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
    Motor_msg_t msg = {.msgType = MOTOR_MSG_EMBRAYE};
    xQueueSend(msgQueueMotor, &msg, 0);
}

/*
 * @brief Send command to let out the clutch to motor task
 * Upon reception of this command the motor task disengages the actuator
 * and stops motor.
 * @param void
 * @return void
 */
void Motor_msg_disengage_actuator(void)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_DEBRAYE};
    xQueueSend(msgQueueMotor, &msg, 0);
}

/*
 * @brief Send steering angle to motor task
 * Upon reception of this command if the angle
 * is significantly different from angle the motor task
 * steers the helm to the given angle.
 * The command has no effect if the Cactuator is disengaged.
 * @param angle angle to steer is in radians
 * @return void
 */
void Motor_msg_set_helm_angle(float angle)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_SET_HELM_ANGLE};
    msg.data.steerAngle = angle;
    xQueueSend(msgQueueMotor, &msg, 0);
}

/*
 * @brief Send command to let out the clutch to autopilot
 * Upon reception of this command turn the motor for given time.
 * @param time in seconds to run the motor, positive: starboard, negative: port.
 * @return void
 */
void Motor_msg_move_time(float time)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_MOVE_TIME};
    msg.data.moveTime = time;
    xQueueSend(msgQueueMotor, &msg, 0);
}

void Motor_msg_set_cvt_angle_time(float cvt)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_SET_CVT_ANGLE_TIME,
                       .data.cvtAngleTime = cvt};
    xQueueSend(msgQueueMotor, &msg, 0);
}

void Motor_msg_set_hpf_coeff(float coef)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_SET_HPF_COEF,
                       .data.hpf_coeff = coef };
    xQueueSend(msgQueueMotor, &msg, 0);
}

void Motor_msg_set_threshold(float thr)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_SET_THRESHOLD,
                       .data.threshold = thr };
    xQueueSend(msgQueueMotor, &msg, 0);
}

void Motor_msg_display_config(void)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_DISPLAY_CONFIG};
    xQueueSend(msgQueueMotor, &msg, 0);
}

/****************************************************************************\
*     Functions and structures that control the motor                        *
******************************************************************************
*                                                                            *
* These functions control the motor, they :                                  *
*  - send commands to the motor and clutch,                                  *
*  - receive new values of voltage and current,                              *
*  - estimate the helm angle,                                                *
*  - stop motor if helm is at angle or in case of overcurrent,               *
*  - keep state of the motor and estimated helm angle.                       *
*  - send messages to autopilot task in case of stall or stop.               *
*                                                                            *
* Theses functions use MotorData struct to store state of motor              *
* They use MOTOR_LLXXX() function to send commands to motor driver           *
* They are called by taskMotor()                                             *
*                                                                            *
\****************************************************************************/

/*
* Motor status structure
*/
typedef struct
{
    /* Status */
    uint32_t status;

    /* Tuning Data */
    float threshold; /* Threshold motor command */
    float hpf_coeff; /* High pass filter coefficient */
    /**/
    /* Data for duration and move */
    float helm_angle_estimated; /* Estimated helm angle (rad) */
    float helm_angle_requested; /* Requested steer angle (rad) */
    float turn_time_req;        /* Turn angle requested */
    /* (rad counterclockwise ie with sign) */
    float turn_time_remaining;  /* Turning time Remaining positive */
    float stop_time_remaining;  /* Time since motor powered off */
    float over_current_time;    /* Time since start of overcurrent */
    ;     /* Values of calibration */
    float vcurrent_free;        /* adc value of current when moving with no effort */
    float vcurrent_stalled;     /* adc value of current when motor blocked */
    float v_power_standard;     /* Standard power voltage */
    float time_to_start;        /* Time to start the motor */
    float time_to_stop;         /* Time to stop the motor */
    float cvt_angle_time;       /* Conversion helm angle to time */
    /* measured values */
    float supply_voltage;       /* Actual voltage*/
    float motor_current;        /* Actual current */

} Motor_t;

void Motor_init(Motor_t* motor)
{
    /* status is set to stopping with time to stop set
     * Stop motor order is sent
     * Motor is in state stopping to make sure that no opposite order
     * is sent if it was running */
    Motor_LL_stop();
    Motor_LL_disengage_actuator();
    motor->status = MOTOR_STATUS_STOPPING;
    motor->stop_time_remaining = MOTOR_TIME_TO_STOP;
    motor->helm_angle_estimated = 0.F;
    motor->helm_angle_requested = 0.F;
    motor->turn_time_req = 0.F;
    motor->turn_time_remaining = 0.F;
    motor->over_current_time = 0.F;
    motor->vcurrent_free = MOTOR_V_CURRENT_FREE;
    motor->vcurrent_stalled = MOTOR_V_CURRENT_BLOCKED;
    motor->v_power_standard = MOTOR_V_POWER_STANDARD;
    motor->time_to_start = MOTOR_TIME_START;
    motor->time_to_stop = MOTOR_TIME_STOP;
    motor->cvt_angle_time = MOTOR_CVT_ANGLE_TIME;
    motor->hpf_coeff = MOTOR_HPF_COEF;
    motor->threshold = MOTOR_THRESHOLD;

    return;
}

/*
 * @brief set the conversion factor between helm angle and time
 * @param cvt conversion factor in rad/s
 * function to be called by motor task
 */
void Motor_set_cvt_angle_time(Motor_t *motor, float cvt)
{
    motor->cvt_angle_time = cvt;

    return;
}

/*
 * @brief set the coefficient of the high pass filter for estimating motor position
 * @param hpf_coeff coefficient
 * function to be called by motor task
 */
void Motor_set_hpf_coeff(Motor_t *motor, float hpf_coeff)
{
    motor->hpf_coeff = hpf_coeff;

    return;
}

/*
 * @brief set the conversion factor between helm angle and time
 * @param cvt conversion factor in rad/s
 * function to be called by motor task
 */
void Motor_set_threshold(Motor_t *motor, float threshold)
{
    motor->threshold = threshold;

    return;
}

void Motor_display_config(Motor_t *motor)
{

    return;
}

/*
 * @brief Update motor status with new values of voltage and current
 *
 * This function updates the motor status with new values of voltage and current
 * and stops or start the motor.
 * It is called by taskMotor when it receives new ADC values.
 *
 * @param deltat Time since last call in seconds
 * @param vPower Voltage of power supply
 * @param iMotor Current through motor, always positive or zero
 * @return motorEvent
 */
uint32_t Motor_new_values(Motor_t* motor,
                             float deltat,
                             float suply_voltage,
                             float motor_current)
{
    uint32_t motor_event = 0U;
    uint32_t dir_eng_keep; /* keep motor dir ie stbd or port and engaged status */

    /******************************************\
    * First : check for overcurrent            *
    \******************************************/
    if(motor_current > motor->vcurrent_stalled * .7F)
    {
        /* over current allowed when starting running motor (for short time) */
        /* but not allowed when running */
        if (motor->status & MOTOR_STATUS_STARTING_RUN)
        {
            motor->over_current_time += deltat;
            if(motor->over_current_time > motor->time_to_start)
            {
                Motor_LL_stop();
                dir_eng_keep = motor->status & (MOTOR_STATUS_DIR_STARBOARD
                                             | MOTOR_STATUS_DIR_PORT
                                             | MOTOR_STATUS_ENGAGED);
                motor->status = MOTOR_STATUS_STALLED | MOTOR_STATUS_STOPPING | dir_eng_keep;
                motor->stop_time_remaining = motor->time_to_stop;
                DBG_MOTOR_PRINT(svc_UART_Write(&svc_uart2, "MOTOR stalled starting\n", 23, 0U));
                motor_event |= MOTOR_EVENT_STALLED;
            }
        }
        else /* motor not starting, if over current stop immediately */
        {
            Motor_LL_stop();
            dir_eng_keep = motor->status & (MOTOR_STATUS_DIR_STARBOARD | MOTOR_STATUS_DIR_PORT);
            motor->status |= MOTOR_STATUS_STALLED | MOTOR_STATUS_STOPPING;
            motor->stop_time_remaining = motor->time_to_stop;
            DBG_MOTOR_PRINT(svc_UART_Write(&svc_uart2, "MOTOR stalled running\n", 22, 0U));
            motor_event |= MOTOR_EVENT_STALLED;
        }
    }

    /******************************************\
    * If motor is stopping                     *
    \******************************************/
    if (motor->status & MOTOR_STATUS_STOPPING)
    {
        /* decrease time remaining to stop by deltat */
        motor->stop_time_remaining -= deltat;

        /* if stopping since sufficient time ie since at least stop_time_remaining
         * motor has stopped and is idle */
        if(motor->stop_time_remaining <= 0.F)
        {
            /* motor stopped, update status */
            /* motor idle and maybe actuatopr is engaged */
            motor->status = MOTOR_STATUS_IDLE | (motor->status & MOTOR_STATUS_ENGAGED);

            motor_event |= MOTOR_EVENT_STOP;

            DBG_MOTOR_PRINT(svc_UART_Write(&svc_uart2, "MOTOR stopped\n", 14, 0U));
        }
    }

    /******************************************\
    * If motor is moving for time              *
    \******************************************/
    if (motor->status & MOTOR_STATUS_MOVING_TIME)
    {
        /* compute time remaining */
        motor->turn_time_remaining -= deltat;

        /* if moving for time and time remaining is negative
         * motor has stopped and is stopping */
        if(motor->turn_time_remaining <= 0.F)
        {
            Motor_LL_stop();
            dir_eng_keep = motor->status & (MOTOR_STATUS_ENGAGED | MOTOR_STATUS_DIR_STARBOARD | MOTOR_STATUS_DIR_PORT);
            motor->status = MOTOR_STATUS_STOPPING |dir_eng_keep;
            motor->stop_time_remaining = motor->time_to_stop;

            motor_event |= MOTOR_EVENT_STOPPING;

            DBG_MOTOR_PRINT(svc_UART_Write(&svc_uart2, "MOTOR end moving time\n", 22, 0U));
        }
    }

    /******************************************\
    * If motor is moving for Angle             *
    \******************************************/
    if (motor->status & MOTOR_STATUS_MOVING_ANGLE)
    {
        int turn_dir;      /* Direction of turn */
        float delta_angle; /* */

        turn_dir = (motor->status & MOTOR_STATUS_DIR_STARBOARD) ? +1 : -1;
        delta_angle = motor->helm_angle_estimated - motor->helm_angle_requested;
        if (turn_dir*delta_angle > 0.F)
        {
            Motor_LL_stop();

            dir_eng_keep = motor->status & (MOTOR_STATUS_ENGAGED | MOTOR_STATUS_DIR_STARBOARD | MOTOR_STATUS_DIR_PORT);
            motor->status = MOTOR_STATUS_STOPPING | dir_eng_keep;
            motor->stop_time_remaining = motor->time_to_stop;
            motor_event |= MOTOR_EVENT_STOPPING;

            DBG_MOTOR_PRINT(svc_UART_Write(&svc_uart2, "MOTOR end moving angle\n", 23, 0U));
        }
    }

    return motor_event;
}

/**
 * @brief Move the motor for a given time
 * @param timeToMove Time to move in seconds, positive to port, negative to stbd
 * If motor is running for a specified time in the same direction as timeToMove,
 * it sets the time to move to timeToMove;
 * @return void
 */
void Motor_move_time(Motor_t *motor, float time_to_move)
{
    int dirSignMoving; /* direction of actual move of motor*/
    int dirSignToMove; /* direction to move motor */
    bool ok_to_turn = false;

    /* determine if running port : -1 or starboard +1 or idle : 0 
     * When stalled one and only one of MOTOR_STATUS_DIR_STARBOARD and
     * MOTOR_STATUS_DIR_PORT is set. */
    switch(motor->status & (MOTOR_STATUS_DIR_STARBOARD | MOTOR_STATUS_DIR_PORT))
    {
    case MOTOR_STATUS_DIR_STARBOARD:
        dirSignMoving = +1;
        break;

    case MOTOR_STATUS_DIR_PORT:
        dirSignMoving = -1;
        break;

    default:
        dirSignMoving = 0;
        break;
    }

    dirSignToMove = (time_to_move > 0.F) ? +1 : -1;

    /*
     * If motor is stalled, only allow to turn if order is in opposite
     * direction of actual move and motor is stopped.
     * If stopping and stalled wait to be shure that motor is stopped.
     */
    if(motor->status & MOTOR_STATUS_STALLED && (!(motor->status & MOTOR_STATUS_STOPPING)))
    {
        /* Stalled and stopped : can turn opposite dir */
        if(dirSignMoving * dirSignToMove < 0)
        {
            ok_to_turn = 1;
        }
    }

    /*
     * If motor is idle it can move port or starboard.
     */
    if (motor->status & MOTOR_STATUS_IDLE)
    {
        /* Idle : can turn either port or starboard */
        ok_to_turn = 1;
    }

    /*
    * If motor is moving for time or stopping, it can move in same direction.
    */
    if (motor->status & (MOTOR_STATUS_MOVING_TIME | MOTOR_STATUS_STOPPING))
    {
        if (dirSignMoving * dirSignToMove >= 0)
        {
            ok_to_turn = 1;
        }
    }

    if(ok_to_turn == 1)
    {
        motor->turn_time_req = fabs(time_to_move);

        motor->turn_time_remaining = motor->turn_time_req;

        if(dirSignToMove > 0)
        {
            Motor_LL_runToStarboard();
            motor->status |= MOTOR_STATUS_DIR_STARBOARD |
                                MOTOR_STATUS_RUNNING | MOTOR_STATUS_MOVING_TIME;

            motor->status &=
                ~(MOTOR_STATUS_DIR_PORT | MOTOR_STATUS_MOVING_ANGLE |
                  MOTOR_STATUS_IDLE | MOTOR_STATUS_STALLED |
                  MOTOR_STATUS_STOPPING);
        }
        else
        {
            Motor_LL_runToPort();
            motor->status |= MOTOR_STATUS_DIR_PORT | MOTOR_STATUS_RUNNING |
                                MOTOR_STATUS_MOVING_TIME;
            motor->status &=
                ~(MOTOR_STATUS_DIR_STARBOARD | MOTOR_STATUS_MOVING_ANGLE |
                  MOTOR_STATUS_IDLE | MOTOR_STATUS_STALLED |
                  MOTOR_STATUS_STOPPING);
        }
    }

    return;
}

void Motor_set_helm_angle(Motor_t *motor, float angle)
{

    if(motor->status & MOTOR_STATUS_ENGAGED)
    {
        motor->helm_angle_requested = angle;
    }
    /* If motor has to run or stop it will do it
     *after next call of Motor_new_values() */

    return;
}

void Motor_engage_actuator(Motor_t *motor)
{
    Motor_LL_engage_actuator();

    motor->status |= MOTOR_STATUS_ENGAGED;
    motor->status &= ~(MOTOR_STATUS_MOVING_TIME | MOTOR_STATUS_MOVING_ANGLE);

    /* Motor idle : nothing to do */
    /* Motor stalled : nothing can be done */
    /* Motor stopping : nothing to do */
    /* Motor running : stop it by clearing RUNNING bit and setting STOPPING bit*/
    if (motor->status & MOTOR_STATUS_RUNNING)
    {
        Motor_LL_stop();
        motor->status |= MOTOR_STATUS_STOPPING;
        motor->status &= ~(MOTOR_STATUS_RUNNING|MOTOR_STATUS_STARTING_RUN);
    }

    motor->helm_angle_estimated = 0.F;

    return;
}

void Motor_disengage_actuator(Motor_t *motor)
{
    Motor_LL_disengage_actuator();

    motor->status &= ~(MOTOR_STATUS_ENGAGED);

    if (motor->status & MOTOR_STATUS_RUNNING)
    {
        Motor_LL_stop();
        motor->status |= MOTOR_STATUS_STOPPING;
        motor->status &= ~(MOTOR_STATUS_RUNNING|MOTOR_STATUS_STARTING_RUN);
    }
    Motor_LL_stop();


    return;
}

/**
 * @brief Initialise the motor task
 * This function must be called before starting Motor task.
 */
int Motor_task_init()
{
    /* creates the message queue */
    msgQueueMotor = xQueueCreate(10, sizeof(Motor_msg_t));

    if(msgQueueMotor == (QueueHandle_t)0)
    {
        return -1;
    }
    else
    {
        return 1;
    }
}

/**
 * @brief Motor control task
 *
 *
 */

void Motor_task(void *parameters)
{
    (void)parameters; /* parameters ignored, avoids warning */

    size_t ret;
    unsigned int motorEvent;  /* Motor event type */
    Motor_msg_t msgMoteur;     /* Message to motor task */
    char message[200];        /* Text buffer for messaging */
    int nbcar;                /* Number of characters put in message */
    unsigned int counter = 0; /* Counter for periodic printing of values */
    float vPower;            /* Power supply voltage */
    float vCurrent;          /* Current through motor */
    Motor_t motors_status;
    Motor_t *motor = &motors_status;

    DBG_MOTOR_PRINT(svc_UART_Write(&svc_uart1, "Motor start task\n", 17, 0U));

    /*
     * Motor and power supply monitoring is made by ADC triggered periodicaly
     * by a timer and DMA that are to be started.
     */
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, 2);

    Motor_init(motor);

    for(;;)  /* boucle infernale */
    {
        msgMoteur.msgType = MOTOR_MSG_NONE;

        ret = xQueueReceive(msgQueueMotor, &msgMoteur, pdMS_TO_TICKS(500));

        if(ret != (size_t)0)
        {
            switch(msgMoteur.msgType)
            {
            /* New ADC values :
             * Check that vPower and vCurrent: no important power drop or
             * overcurrent If motor running estimate helm angle or time
             * running
             */
            case MOTOR_MSG_ADC_VALUES:

                vPower =
                    ((float)msgMoteur.data.adcValues.adc_power) *
                    ADC_CVT_TO_VOLTAGE;
                vCurrent =
                    ((float)msgMoteur.data.adcValues.adc_current) *
                    ADC_CVT_TO_CURRENT;

                motorEvent = Motor_new_values(motor, ADC_PERIOD, vPower, vCurrent);

                if(counter % (((motor->status & MOTOR_STATUS_RUNNING)) ? 1 : 20) == 0)
                {
                    DBG_ADC_PRINT(
                        (snprintf(message, sizeof(message),
                                  "ADC  %d %5.2f %5.3f\n", counter,
                                  vPower, vCurrent),
                         svc_UART_Write(&svc_uart2, message,
                                        strlen(message), 0U)));
                }

                /* Vérifie que le moteur n'est pas bloqué */
                if(motorEvent & MOTOR_EVENT_STALLED)
                {
                    /* if stalled send a message (UART2) and warn autopilot task */
                    svc_UART_Write(&svc_uart2, "MOTOR stalled\n", 14, 0U);
                    AP_MSG_MotorStalled();
                }

                break; /* case MSG_MOTOR_ADC_VALUES: */

            /* Set helm angle : */
            case MOTOR_MSG_SET_HELM_ANGLE:

                Motor_set_helm_angle(motor, msgMoteur.data.steerAngle);

                break; /* case MSG_MOTOR_SET_HELM_ANGLE: */

            /* Disengage Motor */
            case MOTOR_MSG_DEBRAYE:

                Motor_disengage_actuator(motor);

                break; /* case MSG_MOTOR_DEBRAYE: */

            case MOTOR_MSG_EMBRAYE: /* Engage Motor */

                Motor_engage_actuator(motor);

                break; /* case MSG_MOTOR_ENBRAYE: */

            case MOTOR_MSG_MOVE_TIME: /* Move motor for time */

                Motor_move_time(motor, msgMoteur.data.moveTime);
                DBG_MOTOR_PRINT((snprintf(message, sizeof(message),
                                          "MOTOR move time %.3f\n",
                                          msgMoteur.data.moveTime),
                                 svc_UART_Write(&svc_uart2, message,
                                                strlen(message), 0U)));

                break; /* case MSG_MOTOR_MOVE_TIME: */

            case MOTOR_MSG_DISPLAY_CONFIG: /* Display configuration  */

                nbcar = snprintf(message, sizeof(message),
                                 "MOTOR config : cvt angle time %f hpfcoeff %f "
                                 "threshold %f\n",
                                 motor->cvt_angle_time, motor->hpf_coeff,
                                 motor->threshold);
                svc_UART_Write(&svc_uart2, message, nbcar, 0U);

                break; /* case MSG_MOTOR_DISPLAY_CONFIG: */

            /* Set conversion coefficient between angle and time */
            case MOTOR_MSG_SET_CVT_ANGLE_TIME:

                Motor_set_cvt_angle_time(motor, msgMoteur.data.cvtAngleTime);

                snprintf(message, sizeof(message),
                         "MOTOR param cvt angle time %6f\n",
                         msgMoteur.data.cvtAngleTime);
                svc_UART_Write(&svc_uart2, message, strlen(message), 0U);

                break; /* case MSG_MOTOR_SET_CVT_ANGLE_TIME: */

            /* Set coefficient of high pass filter of estimated position
             */
            case MOTOR_MSG_SET_HPF_COEF:

                motor->hpf_coeff = msgMoteur.data.hpf_coeff;

                snprintf(message, sizeof(message),
                         "MOTOR param hpf coefficient %6f\n",
                         msgMoteur.data.moveTime);
                svc_UART_Write(&svc_uart2, message, strlen(message), 0U);

                break; /* MSG_MOTOR_SET_HPF_COEF:  */

            /* Set threshold of a command motor */
            case MOTOR_MSG_SET_THRESHOLD:

                motor->threshold = msgMoteur.data.threshold;

                snprintf(message, sizeof(message),
                         "MOTOR param set threshold %6f\n",
                         msgMoteur.data.moveTime);
                svc_UART_Write(&svc_uart2, message, strlen(message), 0U);

                break; /* case MSG_MOTOR_SET_THRESHOLD:  */

            default:
                /* should never happen */
                break; /*  default */
            }

            counter++;
        }
    }
}

/*
 * @brief ADC interrupt callback
 *
 * @param hadc ADC handle pointer
 * @return none
 *
 * ADC conversions are triggered by a timer and stored in adc_values[0..1]
 * This function is called at the end of the conversion and sends integer
 * values to the motor task.
 * Values are put in a static array and are integer values.
 * Since this function is called by an interrupt handler it mustn't use
 * float values so it sends integers.
 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* One interrupt, so static variable */
    static Motor_msg_t msgMotor; /* Message to the task with ADC values */

    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    msgMotor.msgType = MOTOR_MSG_ADC_VALUES;
    /* ADC values are stored in adc_values[0..1], interrupt is fired after */
    msgMotor.data.adcValues.adc_power = adc_values[0];
    msgMotor.data.adcValues.adc_current = adc_values[1];

    xQueueSendToBackFromISR(msgQueueMotor, &msgMotor,
                            &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return;
}
