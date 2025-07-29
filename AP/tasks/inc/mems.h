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

int init_taskMEMs(void);

void taskMEMs(void *pvParameters);

extern QueueHandle_t msgQueueMEMs;

typedef enum
{
    MEMS_MSG_NONE = 0,
    MEMS_MSG_ACC_READY,
    MEMS_MSG_GYR_READY,
    MEMS_MSG_MAG_READY,
    MEMS_MSG_SEND
} MEMS_MsgType_t;

typedef struct
{
    MEMS_MsgType_t msgType;
    unsigned number; // For debug purpose
} MEMS_Msg_t;

void MEMSSendTimerCallback(TimerHandle_t xTimer);

#define MEMS_STANDARD_GRAVITY 9.80665F
#define MEMS_STANDARD_MAGNETIG 0.000048F

/* Full scales of devices , full scale is +- */
#define MEMS_FULL_SCALE_ACC 2.F
#define MEMS_FULL_SCALE_GYR (245.F * (M_PI / 180.F))
