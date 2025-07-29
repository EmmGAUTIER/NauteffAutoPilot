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
#define AP_KD 1.0F /* derivative coefficient */

/* minimum angle to send an order to motor */
#define AP_MOTOR_THRESHOLD (0.2F * (M_PI / 180.F))

#define AP_PERIOD_TICKS 250
#define AP_FREQ_HZ 4.0
#define AP_PERIOD_SEC (1.0 / AP_FREQ_HZ)

typedef enum
{
    AP_IDLE = 0,
    AP_AUTO_HEADING,
} AP_Mode_t;

typedef enum
{
    AP_MSG_NONE = 0,
    AP_MSG_TICK,             /* */
    AP_MSG_ADC,              /* */
    AP_MSG_MODE_HEADING,     /* Steer current heading */
    AP_MSG_MODE_HEADING_DIR, /* Steer indicated heading */
    AP_MSG_MODE_IDLE,        /* set idle */
    AP_MSG_TURN,             /* turn indicated angle */
    AP_MSG_AHRS,             /* AHRS data : heading, ...  */
    AP_MSG_PARAM,            /* set params */
    AP_MSG_CALIBRATE_MEMS,
    AP_MSG_MEMS_READY,
    AP_MSG_MEMS,
    AP_MSG_MOTOR_STOPPED,
    AP_MSG_MOTOR_STOPPING,
} MsgAutoPilotType_t;

typedef enum
{
    AP_PARAM_NONE = 0,
    AP_PARAM_PROPORTIONNAL,
    AP_PARAM_INTEGRAL,
    AP_PARAM_DERIVATIVE
} APParam_t;

typedef struct
{
    int MEMsReady;
    int engaged;
    int headingToSteerDegrees;
    float headingToSteerRadians;
    float currentHeading;
    float currentGap;
    float integratedGap;
    float derivedGap;
    float heading;
    float roll;
    float pitch;
    float yawRate;
    float rollRate;
    float pitchRate;
    float steerThreshold;

    /* constantes du régulateur PID */
    float kp; /* Proportional */
    float ki; /* Integrali */
    float kd; /* Derivate */

    /* Paramètres moteur */
    float motorThreshold;
    float steerAngle;

    /* historique */
    int memorizedHeading;
    int idxStart;
    int idxEnd;
    float headingHistory[1024];
} APStatus_t;

extern APStatus_t APStatus;
extern QueueHandle_t msgQueueAutoPilot;

typedef struct
{
    uint16_t msgType;      /* Code de message */
    uint16_t defaultCodes; /* Overcurrent, voltage drop, ... */

    union
    {

        int32_t reqHeading; /* requested heading to steer */

        // float helmAngle;   /* Helm angle to turn */

        int32_t reqTurnAngle; /* requested angle to turn */

        struct
        {
            float heading;
            float roll;
            float pitch;
            float yawRate;
            // float rollRate;
            // float pitchRate;
            // float cavalement;
            // float embardee;
            // float pilonnement;
        } IMUData;
        struct
        {
            unsigned param_number;
            float param_value;
        } coefficient;
        struct
        {
            float effort;
        } moveReport;
    } data;
} MsgAutoPilot_t;

int init_taskAutoPilot(void);
void taskAutoPilot(void *parameters);

// int ap_init(APStatus_t *ap);
// int ap_set_heading(APStatus_t *ap, int heading);
//  INLINE int ap_get_heading(APStatus_t *ap);
