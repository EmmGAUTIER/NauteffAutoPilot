/*
 MIT License

 Copyright (c) 2026 Emmanuel Gautier / Nauteff

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
*    Test task : send messages to tasks for debugging                        *
******************************************************************************
*                                                                            *
\****************************************************************************/

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "printf.h"
#include "rlib.h"
#include "service.h"
#include "motor.h"
#include "test.h"

/* Définition de M_PI, il est parfois non défini */
#ifndef M_PI
#define M_PI ((float)3.14159265358979323846)
#endif

Test_call_t test0 [] =
{
    {"engage",    1000, Motor_msg_engage_actuator,     ARG_NONE, {}},
    {"disengage", 1000, Motor_msg_disengage_actuator,  ARG_NONE, {}},
    {"turn +1",   1000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = 1.0F}},
    {"display",     10, Motor_msg_display_status,      ARG_NONE,  {}},
    {"display",    190, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn -1",   5000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = -1.0F}},
    {"display",     10, Motor_msg_display_status,      ARG_NONE,  {}},
    {"display",    190, Motor_msg_display_status,      ARG_NONE,  {}},
    {"engage",    2000, Motor_msg_engage_actuator,     ARG_NONE, {}},
    {"disengage", 1000, Motor_msg_disengage_actuator,  ARG_NONE, {}},
    {"end", 0, (void*)0, ARG_NONE, {} }
};

Test_call_t test1 [] =
{
    {"display",       100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"disengage",    2000, Motor_msg_disengage_actuator,  ARG_NONE, {}},
    {"display",       100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn +0.5s",   2000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = +0.5F}},
    {"display",       100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn +0.1s",   2000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = +0.1F}},
    {"display",       100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn +0.5s",   2000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = +0.5F}},
    {"display",       100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn -0.5s",   2000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = -0.5F}},
    {"display",       100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn -0.1s",   2000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = -0.1F}},
    {"display",       100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn -0.5s",   2000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = -0.5F}},
    {"end", 0, (void*)0, ARG_NONE, {} }
};

Test_call_t test2 [] =
{
    {"disengage",    1000, Motor_msg_disengage_actuator,  ARG_NONE,  {}},
    {"display",       100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn +0.5s",   1000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = +0.5F}},
    {"display",       100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn -0.5s",   1000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = -0.5F}},
    {"display",       100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn +0.5s",   1000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = +0.5F}},
    {"display",       100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn -0.5s",   1000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = -0.5F}},
    {"display",       100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn +0.5s",   1000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = +0.5F}},
    {"display",       100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn -0.5s",   1000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = -0.5F}},
    {"display",       100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"end", 0, (void*)0, ARG_NONE, {} }
};

Test_call_t test3 [] =
{
    {"disengage    ",   10, Motor_msg_disengage_actuator,  ARG_NONE,  {}},
    {"display",        100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"to stall stbd",  100, Motor_msg_move_time,           ARG_FLOAT, {.value.f = +10.F}},
    {"display",        100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn -0.5s",    1000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = -0.5F}},
    {"display",        100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"to stall stbd", 1000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = +10.F}},
    {"display",        100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn -0.5s",    1000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = -0.5F}},
    {"display",        100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"turn +0.2s",    1000, Motor_msg_move_time,           ARG_FLOAT, {.value.f = +0.2F}},
    {"display",        100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"end", 0, (void*)0, ARG_NONE, {} }
};

Test_call_t test4 [] =
{
    {"engage",          10, Motor_msg_engage_actuator,     ARG_NONE,  {}},
    {"display",        100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"hdg 1.0",       1000, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = +1.F}},
    {"display",         10, Motor_msg_display_status,      ARG_NONE,  {}},
    {"hdg 0.5",         10, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = +0.5F}},
    {"hdg -0.5",       100, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = -0.5F}},
    {"hdg retour 0",  2000, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = -0.F}},
    {"disengage",     1000, Motor_msg_disengage_actuator,  ARG_NONE,  {}},
    {"display",        100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"end", 0, (void*)0, ARG_NONE, {} }
};

Test_call_t test5 [] =
{
    {"engage    ",      10, Motor_msg_engage_actuator,     ARG_NONE,  {}},
    {"display",        100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"helm retour 0.", 500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = 0.F * (M_PI / 180.F)}},
    {"helm +5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = +5.F * (M_PI / 180.F)}},
    {"helm -5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = -5.F * (M_PI / 180.F)}},
    {"helm +5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = +10.F * (M_PI / 180.F)}},
    {"helm -5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = -10.F * (M_PI / 180.F)}},
    {"helm +5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = +5.F * (M_PI / 180.F)}},
    {"helm -5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = -5.F * (M_PI / 180.F)}},
    {"helm +5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = +5.F * (M_PI / 180.F)}},
    {"helm -5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = -5.F * (M_PI / 180.F)}},
    {"helm +5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = +5.F * (M_PI / 180.F)}},
    {"helm -5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = -5.F * (M_PI / 180.F)}},
    {"helm +5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = +5.F * (M_PI / 180.F)}},
    {"helm -5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = -5.F * (M_PI / 180.F)}},
    {"helm retour 0.", 500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = 0.F * (M_PI / 180.F)}},
    {"display",        000, Motor_msg_display_status,      ARG_NONE,  {}},
    {"disengage    ",   10, Motor_msg_disengage_actuator,  ARG_NONE,  {}},
    {"display",        100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"end", 0, (void*)0, ARG_NONE, {} }
};

Test_call_t test6 [] =
{
    {"engage    ",      10, Motor_msg_engage_actuator,     ARG_NONE,  {}},
    {"display",        100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"helm retour 0.", 500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f =   0.F * (M_PI / 180.F)}},
    {"helm +15 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = +15.F * (M_PI / 180.F)}},
    {"helm  +5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f =  +5.F * (M_PI / 180.F)}},
    {"helm +15 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = +15.F * (M_PI / 180.F)}},
    {"helm  +5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f =  +5.F * (M_PI / 180.F)}},
    {"helm +15 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = +15.F * (M_PI / 180.F)}},
    {"helm  +5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f =  +5.F * (M_PI / 180.F)}},
    {"helm +15 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = +15.F * (M_PI / 180.F)}},
    {"helm  +5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f =  +5.F * (M_PI / 180.F)}},
    {"helm +15 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = +15.F * (M_PI / 180.F)}},
    {"helm  +5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f =  +5.F * (M_PI / 180.F)}},
    {"helm retour 0.", 500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f =   0.F * (M_PI / 180.F)}},
    {"display",        100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"display",       2000, Motor_msg_display_status,      ARG_NONE,  {}},
    {"helm -15 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = -15.F * (M_PI / 180.F)}},
    {"helm  -5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f =  -5.F * (M_PI / 180.F)}},
    {"helm -15 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = -15.F * (M_PI / 180.F)}},
    {"helm  -5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f =  -5.F * (M_PI / 180.F)}},
    {"helm -15 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = -15.F * (M_PI / 180.F)}},
    {"helm  -5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f =  -5.F * (M_PI / 180.F)}},
    {"helm -15 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = -15.F * (M_PI / 180.F)}},
    {"helm  -5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f =  -5.F * (M_PI / 180.F)}},
    {"helm -15 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f = -15.F * (M_PI / 180.F)}},
    {"helm  -5 deg.",   500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f =  -5.F * (M_PI / 180.F)}},
    {"helm retour 0.", 500, Motor_msg_set_helm_angle,      ARG_FLOAT, {.value.f =   0.F * (M_PI / 180.F)}},
    {"display",       1000, Motor_msg_display_status,      ARG_NONE,  {}},
    {"disengage    ",   10, Motor_msg_disengage_actuator,  ARG_NONE,  {}},
    {"display",        100, Motor_msg_display_status,      ARG_NONE,  {}},
    {"end", 0, (void*)0, ARG_NONE, {} }
};

Test_call_t* tests_list[] = {test0, test1, test2, test3, test4, test5, test6};

static QueueHandle_t test_msg_queue = NULL;
static TimerHandle_t test_timer = NULL;

/*
* @brief Send command to to test task to start a test.
* @param test_number number of test to start
* @return void
*/
void Test_msg_start(int test_number)
{
    Test_msg_t msg;

    msg.msgType = TEST_MSG_START;
    msg.data.test_number = test_number;

    xQueueSend(test_msg_queue, &msg, pdMS_TO_TICKS(0));

    return;
}

/*
* @brief Send command to to test task to stop a test.
* @return void
*/
void Test_msg_stop(void)
{
    /* message struct may be static because it is never changed */
    static Test_msg_t msg = {.msgType = TEST_MSG_STOP};

    xQueueSend(test_msg_queue, &msg, pdMS_TO_TICKS(0));

    return;
}

/*
 * @brief Sends a tick message to the tests task
 *
 * Callback of the timer test_timer.
 * It is not called periodically.
 *
 * This function sends a tick message to the Test task.
 *
 * @ param xTimer The timer handle (not used)
 * @ return None
 */
void Test_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer; /* statement to avoid unused parameter warning */

    /* The message is contained in a struct.
     * it is declared static to avoid using stack.
     * Only one blink task, so only one timer and callback execution at a time,
     * so no risk of overwriting the message before it is sent.
     */
    static Test_msg_t msg = {.msgType = TEST_MSG_TICK};

    xQueueSend(test_msg_queue, (const void *)&msg, (TickType_t)0);

    return;
}

/*
 * @brief Creates necessary stuff for the Test tast.
 *
 * @param none
 * @return none
 *
 * Tasks have to use queues and timers. These have to be created
 * before starting scheduler so when tasks begin they can use them.
 * The queue is used to send ticks and messages to the task.
 *
 */
void Test_task_init()
{
    /* MEMs task queue creation */
    test_msg_queue = xQueueCreate(1, sizeof(Test_msg_t));

    /* Create a timer that sends messages periodically to the task */
    test_timer = xTimerCreate("TEST", pdMS_TO_TICKS(1000),
                              pdFALSE,   /* This timer is only used for a test */
                              (void *)0, /* Timer ID, not used */
                              Test_timer_callback);
}

/*
 * @brief blink a LED for debugging purpose only
 *
 * Tests the green LED on the Nucleo board.
 * This task is used for debugging and to check if the system is running.
 *
 */

void Test_task(void *param)
{
    (void)param;  /* avoid compiler warning, unused variable */

    char message[100];                          /* buffer for messages for debugging */
    BaseType_t ret;
    Test_msg_t test_msg;                        /* Message struct containing info to send to the task */
    Test_call_t* test_calls = (Test_call_t*)0;  /* Pointer to a table of calls */
    Test_call_t* call       = (Test_call_t*)0;  /* Ptr to call struct : delay, name, fct ptr, args */
    int test_idx = -1;                          /* index of current function call */

    for(;;)  /* infinite loop */
    {
        ret = xQueueReceive(test_msg_queue, &test_msg, pdMS_TO_TICKS(0));

        if(ret == pdPASS)
        {
            switch(test_msg.msgType)
            {
            case TEST_MSG_TICK:
                if((test_idx >= 0) && (test_calls[test_idx].func_ptr != (void*)0))
                {
                    snprintf(message, sizeof(message), "TEST\nTEST tick %d  %s\n", test_idx, test_calls[test_idx].name);
                    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);

                    /* Call the function with the argument if any */
                    switch(test_calls[test_idx].arg_type)
                    {
                    case ARG_NONE:
                        ((func_ptr_t)test_calls[test_idx].func_ptr)();
                        break;

                    case ARG_INT:
                        ((func_ptr_int_t)test_calls[test_idx].func_ptr)(test_calls[test_idx].arg.value.i);
                        break;

                    case ARG_FLOAT:
                        ((func_ptr_float_t)test_calls[test_idx].func_ptr)(test_calls[test_idx].arg.value.f);
                        break;

                    default:
                        /* should never happen */
                        break;
                    }

                    test_idx++;
                }

                if(test_calls[test_idx].func_ptr != (void*)0)
                {
                    xTimerStart(test_timer, pdMS_TO_TICKS(call->delay));
                }

                break;

            case TEST_MSG_START:
                if(test_msg.data.test_number >= 0 && test_msg.data.test_number < sizeof(tests_list) / sizeof(tests_list[0]))
                {
                    snprintf(message, sizeof(message), "TEST start %d\n", test_msg.data.test_number);
                    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
                    test_calls = tests_list[test_msg.data.test_number];
                    test_idx = 0;
                    call = &test_calls[test_idx];

                    if(call->func_ptr != (void*)0)
                    {
                        xTimerStart(test_timer, pdMS_TO_TICKS(call->delay));
                    }
                }

                break;

            case TEST_MSG_STOP:
                /* stop test */
                xTimerStop(test_timer, pdMS_TO_TICKS(0));
                break;

            default:
                /* should never happen */
                break;
            }
        }

    }

    /* Mustn't be reached */
}
