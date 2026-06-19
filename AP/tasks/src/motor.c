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

#define NB_VAL_ADC  10
struct
{
    float vPower;
    float iMotor;
} adcs[NB_VAL_ADC];
int idxadc = 0;

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

#define MOTOR_HPF_COEF       (0.05F)
#define MOTOR_THRESHOLD      (1.0F * ((float)M_PI / 180.F)) /* threshold 1 deg. */
#define MOTOR_CVT_ANGLE_TIME (3.2F)  /* Estimated conversion between time and helm move angle s/rad */
#define MOTOR_TIME_START     (0.05F) /* Maximum time to allow over current when starting motor (s) */
#define MOTOR_TIME_STOP      (0.2F)  /* Time to wait for motor to stop before opposite move order (s) */
#define MOTOR_DELTA_DIR_GAIN (0.05F) /* Tiller move difference part to add to estimated starborad move */
/* and to substract to estimated port move*/

#define MOTOR_V_CURRENT_NONE    (0.F)
#define MOTOR_V_CURRENT_FREE    (.1F)
#define MOTOR_V_CURRENT_BLOCKED (1.0F)     /*  */
#define MOTOR_V_POWER_STANDARD  (12.0F)
#define MOTOR_TIME_TO_STOP (0.1F)          /* 100 ms */
#define MOTOR_MAX_TIME_OVERCURRENT (0.02F) /* 20 ms */
#define ADC_CVT_TO_VOLTAGE (0.0091F) /* Ratio ADC val. and power voltage */
#define ADC_CVT_TO_CURRENT (0.0004F) /* Ratio  ADC val. and current */

/* events returned by motor functions */
#define MOTOR_EVENT_STALLED  (0x1 << 0)
#define MOTOR_EVENT_STOP     (0x1 << 1)
#define MOTOR_EVENT_STOPPING (0x1 << 2)

/* status bits of motor */
/* More than minimum required so easier to code and test */
#define MOTOR_STATUS_ENGAGED (0x1 << 0)       /* 0 clutch out, 1 : clutch in*/
#define MOTOR_STATUS_MOVING_TIME (0x1 << 1)   /* moving for time */
#define MOTOR_STATUS_MOVING_ANGLE (0x1 << 2)  /* moving helm to angle */
#define MOTOR_STATUS_DIR_STARBOARD (0x1 << 3) /* moving to starboard */
#define MOTOR_STATUS_DIR_PORT (0x1 << 4)      /* moving to port*/
#define MOTOR_STATUS_IDLE (0x1 << 5)          /* motor idle i.e. stopped */
//#define MOTOR_STATUS_RUNNING (0x1 << 6)       /* motor running */
#define MOTOR_STATUS_STOPPING (0x1 << 7)      /* motor stopping */
#define MOTOR_STATUS_STALLED (0x1 << 8)       /* motor stalled */
#define MOTOR_STATUS_STARTING_RUN (0x1 << 9)  /* motor starting to run ie accelerating */
/* shortcut for stbd and port dir, may be used to keep direction with &= on motor status */
#define MOTOR_STATUS_DIR_ANY (MOTOR_STATUS_DIR_STARBOARD | MOTOR_STATUS_DIR_PORT)
#define MOTOR_STATUS_RUNNING (MOTOR_STATUS_MOVING_TIME | MOTOR_STATUS_MOVING_ANGLE)

/*****************************************************************************\
 * Global vars                                                                *
 ******************************************************************************
 * Queue handle for messages to motor task : msgQueueMotor                    *
 * For ADC values :                                                           *
 * htim3 : timer handler for timer triggering conversion                      *
 * hadc1 : ADC handler that make ADC conversions                              *
 * htim3 and hadc1 declared by code generated code generated by STM32CubeMX   *
 * adc_values : array of converted values                                     *
 \****************************************************************************/
/* Message queue for Motor task */
static QueueHandle_t msgQueueMotor = (QueueHandle_t)0;
/* Device handles of timer and ADC */
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

/* Every ADC period timer TIM3 triggers two ADC conversions (Vpower and Imot),
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
 *   - PA4 : motor command (also called PWM),                                 *
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
 * Port is left side of boat and starboard is right side of boat.             *
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

/*
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

/*
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

    DBG_MOTOR_PRINT(svc_UART_Write(&svc_uart2, "MOTOR LL engage actuator\n", 25, 0U));
}

/*
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

    DBG_MOTOR_PRINT(svc_UART_Write(&svc_uart2, "MOTOR LL disengage actuator\n", 28, 0U));
}

/****************************************************************************\
*     Function to send orders to MOTOR task                                  *
 ******************************************************************************
 *                                                                            *
 * These functions send commands to the motor task by sending messages        *
 * to its message queue. They are a easy way for creating messages and        *
 * sending them to motor task without having to know message structure.       *
 *                                                                            *
 * Note : Messages to the task come from several tasks: mems and autopilot.   *
 * Messages are put in a queue by these functions.                            *
 *                                                                            *
 * They have no return value.                                                 *
 * Their prototype is :                                                       *
 * void Motor_msg_XXX(...)                                                    *
 * Doc for doxygen in motor.h                                                 *
 *                                                                            *
 \****************************************************************************/

/* Declare an enum for the message types */
typedef enum
{
    MOTOR_MSG_NONE = 0,
    MOTOR_MSG_ADC_VALUES,
    MOTOR_MSG_EMBRAYE,
    MOTOR_MSG_DEBRAYE,
    MOTOR_MSG_SET_HELM_ANGLE,
    MOTOR_MSG_MOVE_TIME,
    MOTOR_MSG_MOVE_DONE,
    MOTOR_MSG_DISPLAY_CONFIG,
    MOTOR_MSG_DISPLAY_STATUS,
    MOTOR_MSG_DEFAULT,
    MOTOR_MSG_SET_CVT_ANGLE_TIME,
    MOTOR_MSG_SET_DELTA_DIR_GAIN,
    MOTOR_MSG_SET_HPF_COEF,
    MOTOR_MSG_SET_THRESHOLD,
} Motor_msg_type_t;

/* Struct of motor message type and data in union */
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
        float deltaDirGain;
        float hpf_coeff;
        float threshold;
    } data;
} Motor_msg_t;

void Motor_msg_engage_actuator(void)
{
    char message [40];
    snprintf(message, sizeof(message), "MOTOR msg engage actuator\n");
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
    Motor_msg_t msg = {.msgType = MOTOR_MSG_EMBRAYE};
    xQueueSend(msgQueueMotor, &msg, 0);
}

void Motor_msg_disengage_actuator(void)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_DEBRAYE};
    xQueueSend(msgQueueMotor, &msg, 0);
}

void Motor_msg_set_helm_angle(float angle)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_SET_HELM_ANGLE};
    msg.data.steerAngle = angle;
    xQueueSend(msgQueueMotor, &msg, 0);
}

void Motor_msg_move_time(float time)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_MOVE_TIME};
    msg.data.moveTime = time;
    xQueueSend(msgQueueMotor, &msg, 0);
}

void Motor_msg_set_cvt_angle_time(float cvt)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_SET_CVT_ANGLE_TIME,
                       .data.cvtAngleTime = cvt
                      };
    xQueueSend(msgQueueMotor, &msg, 0);
}

void Motor_msg_set_delta_dir_gain(float ddg)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_SET_DELTA_DIR_GAIN,
                       .data.deltaDirGain = ddg
                      };
    xQueueSend(msgQueueMotor, &msg, 0);

    return;
}

void Motor_msg_set_hpf_coeff(float coef)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_SET_HPF_COEF,
                       .data.hpf_coeff = coef
                      };
    xQueueSend(msgQueueMotor, &msg, 0);
}

void Motor_msg_set_threshold(float thr)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_SET_THRESHOLD,
                       .data.threshold = thr
                      };
    xQueueSend(msgQueueMotor, &msg, 0);
}

void Motor_msg_display_config(void)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_DISPLAY_CONFIG};
    xQueueSend(msgQueueMotor, &msg, 0);
}

void Motor_msg_display_status(void)
{
    Motor_msg_t msg = {.msgType = MOTOR_MSG_DISPLAY_STATUS};
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
    /* Data for duration and move */
    float helm_angle_estimated;   /* Estimated helm angle (rad) */
    float helm_angle_requested;   /* Requested steer angle (rad) */
    float turning_time_requested; /* Turn angle requested */
    /* (rad counterclockwise ie with sign) */
    float turning_time;           /* Turning time Remaining positive */
    float stopping_time;          /* Time since motor powered off */
    float starting_time;          /* Time since start of overcurrent */
    float overcurrent_start_time; /* time of over current condition when starting */
    /* Values of calibration */
    float vcurrent_free;          /* adc value of current when moving with no effort */
    float vcurrent_stalled;       /* adc value of current when motor blocked */
    float v_power_standard;       /* Standard power voltage */
    float time_to_start;          /* Time to start the motor */
    float time_to_stop;           /* Time to stop the motor */
    float cvt_angle_time;         /* Conversion helm angle to time */
    float delta_dir_gain;         /* gain to add for starboard and substract for port move */
    /* measured values */
    float supply_voltage;         /* Actual voltage*/
    float motor_current;          /* Actual current */

} Motor_t;

void Motor_init(Motor_t* motor)
{
    /* status is set to stopping with time to stop set
     * Stop motor order is sent
     * Motor is in state stopping to make sure that no opposite order
     * is sent if it was running */
    Motor_LL_stop();
    Motor_LL_disengage_actuator();
    motor->status                 = MOTOR_STATUS_STOPPING | MOTOR_STATUS_DIR_ANY;
    motor->stopping_time          = 0.F;
    motor->helm_angle_estimated   = 0.F;
    motor->helm_angle_requested   = 0.F;
    motor->turning_time_requested = 0.F;
    motor->turning_time           = 0.F;
    motor->starting_time          = 0.F;
    motor->overcurrent_start_time = 0.F;
    motor->vcurrent_free          = MOTOR_V_CURRENT_FREE;
    motor->vcurrent_stalled       = MOTOR_V_CURRENT_BLOCKED;
    motor->v_power_standard       = MOTOR_V_POWER_STANDARD;
    motor->time_to_start          = MOTOR_TIME_START;
    motor->time_to_stop           = MOTOR_TIME_STOP;
    motor->delta_dir_gain         = MOTOR_DELTA_DIR_GAIN;
    motor->cvt_angle_time         = MOTOR_CVT_ANGLE_TIME;
    motor->hpf_coeff              = MOTOR_HPF_COEF;
    motor->threshold              = MOTOR_THRESHOLD;
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

void Motor_set_delta_dir_gain(Motor_t *motor, float ddg)
{
    motor->delta_dir_gain = ddg;

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

/*
 * @brief Give a short text with configuration of motor
 * @motor pointer to a Motor_t struct
 * @str pointer to character string
 * @len max len of string
 */
int Motor_get_text_config(Motor_t *motor, char* str, int len)
{
    int nbcar;

    nbcar = snprintf(str, len,
                     "MOTOR config : cvt angle time %f hpfcoeff %f ddg %f"
                     "threshold %f\n",
                     motor->cvt_angle_time, motor->hpf_coeff,
                     motor->delta_dir_gain, motor->threshold);

    return nbcar;
}

/*
 * @brief Give a short text with status of motor
 * @motor pointer to a Motor_t struct
 * @str pointer to character string
 * @len max len of string
 */
int Motor_get_text_status(Motor_t *motor, char* str, int len)
{
    int nbcar;

    nbcar = snprintf(str, len,
                     "MOTOR status : %7s %4s %5s %4s %8s %7s %4s %4s %7s %6.3f %6.3f ct %6.3f\n",
                     (motor->status & MOTOR_STATUS_ENGAGED)       ? "engaged"  : "",
                     (motor->status & MOTOR_STATUS_IDLE)          ? "idle"     : "",
                     (motor->status & MOTOR_STATUS_MOVING_ANGLE)  ? "angle"    : "",
                     (motor->status & MOTOR_STATUS_MOVING_TIME)   ? "time"     : "",
                     (motor->status & MOTOR_STATUS_STARTING_RUN)  ? "starting" : "",
                     (motor->status & MOTOR_STATUS_RUNNING)       ? "running"  : "",
                     (motor->status & MOTOR_STATUS_DIR_STARBOARD) ? "stbd"     : "",
                     (motor->status & MOTOR_STATUS_DIR_PORT)      ? "port"     : "",
                     (motor->status & MOTOR_STATUS_STALLED)       ? "stalled"  : "",
                     motor->overcurrent_start_time,
                     motor->helm_angle_requested,
                     motor->helm_angle_estimated);

    return nbcar;
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
    char message[100];

    uint32_t motor_event = 0U; /* Value to be returned, contains detected events */

    /***********************************************\
    * First : check for overcurrent                 *
    *************************************************
    * Two cases : while starting and while running. *
    * While starting for time < motor-time_to_start *
    * overcurrent is accepted                       *
    * While running stop immediatly and             *
    * set STALLED bit                               *
    ************************************************/

    if(motor->status & MOTOR_STATUS_STARTING_RUN)
    {
        /* Motor starting overcurrent accepted for short time ( < time_to_start) */
        motor->starting_time += deltat;

        /* Au fait ça dure combien de temps ce courant élevé au démarrage ? */
        if(motor_current > motor->vcurrent_stalled * .6F)
        {
            motor->overcurrent_start_time = motor->starting_time;
        }

        if(motor->starting_time > motor->time_to_start)
        {
            motor->status &= ~(MOTOR_STATUS_STARTING_RUN);
        }
    }

    if(!(motor->status & MOTOR_STATUS_STARTING_RUN) && (motor_current > motor->vcurrent_stalled * .6F))
    {
        /* stop immediatly */
        Motor_LL_stop();
        motor->status &= MOTOR_STATUS_DIR_ANY | MOTOR_STATUS_ENGAGED;
        motor->status |= MOTOR_STATUS_STALLED | MOTOR_STATUS_STOPPING;
        motor->stopping_time = 0.F;

        DBG_MOTOR_PRINT(svc_UART_Write(&svc_uart2, "MOTOR stalled\n", 14, 0U));
        motor_event |= MOTOR_EVENT_STALLED;
    }

    /******************************************\
    * If motor is stopping                     *
    \******************************************/
    if(motor->status & MOTOR_STATUS_STOPPING)
    {
        /* decrease time remaining to stop by deltat */
        motor->stopping_time += deltat;

        /* if stopping since sufficient time ie since at least stop_time_remaining
         * motor has stopped and is idle */
        if(motor->stopping_time >= motor->time_to_stop)
        {
            /* motor stopped, update status */
            /* motor idle and maybe actuator is engaged */
            //motor->status = MOTOR_STATUS_IDLE | (motor->status &( MOTOR_STATUS_ENGAGED | MOTOR_STATUS_STALLED));

            if(motor->status & MOTOR_STATUS_STALLED)
            {
                motor->status &= MOTOR_STATUS_DIR_ANY
                                 | MOTOR_STATUS_ENGAGED
                                 | MOTOR_STATUS_STALLED;
            }
            else
            {
                motor->status &= MOTOR_STATUS_ENGAGED;
                motor->status |= MOTOR_STATUS_IDLE;
            }

            motor_event |= MOTOR_EVENT_STOP;

            DBG_MOTOR_PRINT(svc_UART_Write(&svc_uart2, "MOTOR stopped\n", 14, 0U));
        }
    }

    /******************************************\
    * If motor is moving for time              *
    \******************************************/
    if(motor->status & MOTOR_STATUS_MOVING_TIME)
    {
        /* compute time remaining */
        motor->turning_time += deltat;

        /* if moving for time and time remaining is negative
         * motor has stopped and is stopping */
        if(motor->turning_time >= motor->turning_time_requested)
        {
            Motor_LL_stop();
            motor->status &= ~(MOTOR_STATUS_MOVING_TIME
                               |  MOTOR_STATUS_MOVING_ANGLE
                               |  MOTOR_STATUS_STARTING_RUN);
            motor->status |= MOTOR_STATUS_STOPPING;

            motor_event |= MOTOR_EVENT_STOPPING;

            DBG_MOTOR_PRINT(svc_UART_Write(&svc_uart2, "MOTOR end moving time\n", 22, 0U));
        }
    }

    /******************************************\
    * If motor is moving for angle             *
    \******************************************/
    if(motor->status & MOTOR_STATUS_ENGAGED)
    {
        float delta_angle; /* diff between helm (estimated) position and helm angle request */

        delta_angle = motor->helm_angle_requested - motor->helm_angle_estimated;

        /******************************************\
        * If motor engaged and is moving for Angle *
        \******************************************/
        if(motor->status & MOTOR_STATUS_MOVING_ANGLE)
        {
            int turning_dir;
            float angle_move;

            /*****************************************************\
            * Estimate Helm position                              *
            \*****************************************************/
            /* turning dir is +1 for moving to starboard or -1 for turning port */
            turning_dir = (motor->status & MOTOR_STATUS_DIR_STARBOARD) ? +1 : -1;
            /* angle move is proportionnal to time since las estimate, conversion factor
             * which are always positive , it has to multiplied by dir */
            angle_move = deltat * motor->cvt_angle_time * turning_dir;

            /* First correction : */
            /* Motor turns more in one direction than in the other */
            /* so we apply delta gains */
            if(angle_move > 0.F)
            {
                angle_move *= (1.F + motor->delta_dir_gain);
            }
            else
            {
                angle_move *= (1.F - motor->delta_dir_gain);
            }

            /* second correction : */
            /* As we don't know the helm position and it deviates from estimated angle */
            /* we move slighter to 0 at each estimation with a hight pass filter */
            if(angle_move * motor->helm_angle_estimated > 0.F)
            {
                angle_move *= (1 - motor->hpf_coeff);
            }
            else
            {
                angle_move *= (1 + motor->hpf_coeff);
            }

            /* angle move is added to estimated angle */
            motor->helm_angle_estimated += angle_move;
            delta_angle = motor->helm_angle_requested - motor->helm_angle_estimated;

            if(turning_dir * delta_angle < 0.F)
            {
                Motor_LL_stop();

                motor->status &= ~(MOTOR_STATUS_MOVING_ANGLE);

                motor->status |= MOTOR_STATUS_STOPPING;

                motor->stopping_time = 0.F;
                motor_event |= MOTOR_EVENT_STOPPING;

                DBG_MOTOR_PRINT(svc_UART_Write(&svc_uart2, "MOTOR end moving angle\n", 23, 0U));
                DBG_MOTOR_PRINT((
                                    snprintf(message, sizeof(message), "MOTOR helm estimated angle %f\n", motor->helm_angle_estimated),
                                    svc_UART_Write(&svc_uart2, message, strlen(message), 0U)));
            }
        }
        else
        {
            /* test if helm angle estimated is far from helm angle requested */
            if(fabs(delta_angle) >= motor->threshold)
            {
                /* Has to turn */
                if(delta_angle > 0.F)
                {
                    /* Has to turn starboard */
                    if((motor->status & MOTOR_STATUS_IDLE)
                            || ((motor->status & MOTOR_STATUS_STALLED) && (motor->status & MOTOR_STATUS_DIR_PORT)))
                    {
                        Motor_LL_runToStarboard();
                        motor->status |= MOTOR_STATUS_MOVING_ANGLE
                                         | MOTOR_STATUS_DIR_STARBOARD
                                         | MOTOR_STATUS_STARTING_RUN;
                        motor->status &= ~(MOTOR_STATUS_IDLE | MOTOR_STATUS_STALLED);
                        motor->starting_time = 0.F;
                    }
                }
                else
                {
                    /* Has to turn port */
                    if((motor->status & MOTOR_STATUS_IDLE)
                            || ((motor->status & MOTOR_STATUS_STALLED) && (motor->status & MOTOR_STATUS_DIR_STARBOARD)))
                    {
                        Motor_LL_runToPort();
                        motor->status |= MOTOR_STATUS_MOVING_ANGLE
                                         | MOTOR_STATUS_DIR_PORT
                                         | MOTOR_STATUS_STARTING_RUN;
                        motor->status &= ~(MOTOR_STATUS_IDLE | MOTOR_STATUS_STALLED);
                        motor->starting_time = 0.F;
                    }
                }
            }
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
    char message[100];
    snprintf(message, sizeof(message), "MOTOR move time %f\n", time_to_move);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);

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
        /* Motor not moving and not stalled or
         * unknown state when starting motor control */
        dirSignMoving = 0;
        break;
    }

    dirSignToMove = (time_to_move > 0.F) ? +1 : -1;

    /*
     * If motor is stalled, only allow to turn if order is in opposite
     * direction of actual move and motor is stopped.
     * If stopping and stalled do nothing and wait to be sure that motor is stopped.
     */
    if((motor->status & MOTOR_STATUS_STALLED) && (!(motor->status & MOTOR_STATUS_STOPPING)))
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
    if(motor->status & MOTOR_STATUS_IDLE)
    {
        /* Idle : can turn either port or starboard */
        ok_to_turn = 1;
    }

    /*
    * If motor is moving for time or stopping, it can move in same direction.
    * actual time to move will be replaced by new value of time.
    */
    if(motor->status & (MOTOR_STATUS_MOVING_TIME | MOTOR_STATUS_STOPPING))
    {
        if(dirSignMoving * dirSignToMove >= 0)
        {
            ok_to_turn = 1;
        }
    }

    if(ok_to_turn == 1)
    {
        motor->turning_time_requested = fabs(time_to_move);
        motor->turning_time = 0.F;
        motor->starting_time = 0.F;
        motor->overcurrent_start_time = 0.F;

        if(dirSignToMove > 0)
        {
            Motor_LL_runToStarboard();
            motor->status |= MOTOR_STATUS_DIR_STARBOARD | MOTOR_STATUS_STARTING_RUN
                             | MOTOR_STATUS_MOVING_TIME;
            motor->status &=  ~(MOTOR_STATUS_DIR_PORT | MOTOR_STATUS_MOVING_ANGLE |
                                MOTOR_STATUS_IDLE     | MOTOR_STATUS_STALLED |
                                MOTOR_STATUS_STOPPING);
        }
        else
        {
            Motor_LL_runToPort();
            motor->status |= MOTOR_STATUS_DIR_PORT | MOTOR_STATUS_STARTING_RUN
                             | MOTOR_STATUS_MOVING_TIME;
            motor->status &= ~(MOTOR_STATUS_DIR_STARBOARD | MOTOR_STATUS_MOVING_ANGLE |
                               MOTOR_STATUS_IDLE          | MOTOR_STATUS_STALLED |
                               MOTOR_STATUS_STOPPING);
        }
    }

    return;
}

/*
 * @brief set the requested helm angle
 * @param motor pointer to a Motor_t struct
 * @param angle to set the helm.
 * angle is in radians.
 * angle is positive if helm move has to turn starboard (right).
 * angle is negative if helm move has to turn port (left).
 * @return void
 */
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

/*
 * @brief sengage actuator
 * @param motor pointer to a Motor_t struct
 * Eengage the clutch
 * If motor is running it stops it and changes the state to stopping.
 * @return void
 */
void Motor_engage_actuator(Motor_t *motor)
{
    Motor_LL_engage_actuator();

    motor->status |= MOTOR_STATUS_ENGAGED;

    /* Motor idle : nothing to do */
    /* Motor stalled : nothing can be done */
    /* Motor stopping : nothing to do */
    /* Motor running : stop it by clearing RUNNING bit and setting STOPPING bit*/
    if(motor->status & MOTOR_STATUS_RUNNING)
    {
        Motor_LL_stop();
        motor->status |= MOTOR_STATUS_STOPPING;
        motor->status &= ~(MOTOR_STATUS_RUNNING | MOTOR_STATUS_STARTING_RUN);
    }

    motor->helm_angle_estimated = 0.F;
    motor->helm_angle_requested = 0.F;

    return;
}

/*
 * @brief disengage actuator
 * @param motor pointer to a Motor_t struct
 * Disengage the clutch
 * If motor is running it stops it and changes the state to stopping.
 * @return void
 */
void Motor_disengage_actuator(Motor_t *motor)
{
    Motor_LL_disengage_actuator();

    motor->status &= ~(MOTOR_STATUS_ENGAGED);

    if(motor->status & MOTOR_STATUS_RUNNING)
    {
        Motor_LL_stop();
        motor->status |= MOTOR_STATUS_STOPPING;
        motor->status &= ~(MOTOR_STATUS_RUNNING | MOTOR_STATUS_STARTING_RUN);
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
 * @brief Motor control function task
 *
 * @param ignored
 * Function task that control the motor.
 * @return mustn't return
 */

void Motor_task(void *parameters)
{
    (void)parameters; /* parameters ignored, avoids warning */

    size_t ret;
    unsigned int motorEvent;  /* Motor event type */
    Motor_msg_t msgMoteur;    /* Message to motor task */
    char message[200];        /* Text buffer for messaging */
    int nbcar;                /* Number of characters put in message */
    unsigned int counter = 0; /* Counter for periodic printing of values */
    float vPower;             /* Power supply voltage */
    float vCurrent;           /* Current through motor */
    Motor_t motors_status;
    Motor_t *motor = &motors_status;
    float deltat;  /* delta time (seconds) between 2 ADC data messages */
    BaseType_t timestamp, timestamp_2;

    timestamp = xTaskGetTickCount();

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

                vPower = ((float)msgMoteur.data.adcValues.adc_power) * ADC_CVT_TO_VOLTAGE;
                vCurrent = ((float)msgMoteur.data.adcValues.adc_current) * ADC_CVT_TO_CURRENT;
                idxadc++;
                idxadc %= NB_VAL_ADC;
                adcs[idxadc].iMotor = vPower;
                adcs[idxadc].iMotor = vCurrent;

                timestamp_2 = xTaskGetTickCount();
                deltat = ((float)(timestamp_2 - timestamp)) / ((float)configTICK_RATE_HZ);
                timestamp = timestamp_2;

                motorEvent = Motor_new_values(motor, deltat, vPower, vCurrent);

                if(counter % (((motor->status & (MOTOR_STATUS_MOVING_TIME | MOTOR_STATUS_MOVING_ANGLE))) ? 1 : 20) == 0)
                {
                    DBG_ADC_PRINT(
                        (snprintf(message, sizeof(message),
                                  "ADC %d %f %5.2f %5.3f\n",
                                  counter, deltat,
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

                DBG_MOTOR_PRINT((snprintf(message, sizeof(message),
                                          "MOTOR set helm angle %.3f\n",
                                          msgMoteur.data.steerAngle),
                                 svc_UART_Write(&svc_uart2, message, strlen(message), 0U)));
                Motor_get_text_status(motor, message, sizeof(message));
                svc_UART_Write(&svc_uart2, message, strlen(message), 0U);

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
                Motor_get_text_status(motor, message, sizeof(message));
                svc_UART_Write(&svc_uart2, message, strlen(message), 0U);

                break; /* case MSG_MOTOR_MOVE_TIME: */

            case MOTOR_MSG_DISPLAY_CONFIG: /* Display configuration  */

                nbcar = Motor_get_text_config(motor, message, sizeof(message));
                svc_UART_Write(&svc_uart2, message, nbcar, 0U);

                break; /* case MSG_MOTOR_DISPLAY_CONFIG: */

            case MOTOR_MSG_DISPLAY_STATUS : /* Display motor status */

                nbcar = Motor_get_text_status(motor, message, sizeof(message));
                svc_UART_Write(&svc_uart2, message, nbcar, 0U);

                break; /* MOTOR_MSG_DISPLAY_STATUS */

            /* Set conversion coefficient between angle and time */
            case MOTOR_MSG_SET_CVT_ANGLE_TIME:

                Motor_set_cvt_angle_time(motor, msgMoteur.data.cvtAngleTime);

                snprintf(message, sizeof(message),
                         "MOTOR param cvt angle time %6f\n",
                         msgMoteur.data.cvtAngleTime);
                svc_UART_Write(&svc_uart2, message, strlen(message), 0U);

                break; /* case MSG_MOTOR_SET_CVT_ANGLE_TIME: */

            /* Set conversion coefficient between angle and time */
            case MOTOR_MSG_SET_DELTA_DIR_GAIN:

                Motor_set_delta_dir_gain(motor, msgMoteur.data.cvtAngleTime);

                snprintf(message, sizeof(message),
                         "MOTOR param delta dir gain %6f\n",
                         msgMoteur.data.deltaDirGain);
                svc_UART_Write(&svc_uart2, message, strlen(message), 0U);

                break; /* case MSG_MOTOR_SET_DELTAT_DIR_GAIN: */

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
                         msgMoteur.data.threshold);
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
