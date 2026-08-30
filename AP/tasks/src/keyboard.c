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
*    Keyboard task : Read keybord and sends messages when key pressed        *
******************************************************************************
*                                                                            *
* The use of interrupts for pressing and releasing keys may generate         *
* spurious interrupts so the task reads the state of keys repeatedly.        *
*                                                                            *
*                                                                            *
\****************************************************************************/

/****************************************************************************\
*    Tuning parameters : delays/frequencies                                  *
*****************************************************************************/
#define KEYBOARD_SCAN_PERIOD_MS      100 /* Period in ms of keyboard scan */
#define KEYBOARD_DELAY_REPEAT_MS     500 /* Delay before repeat when key pressed */
#define KEYBOARD_PERIOD_REPEAT_MS    200 /* Period for repeating */
#define KEYBOARD_DELAY_BEFORE_START  500 /* Delay before reading keyboard */

#include <stm32l452xx.h>
#include <stm32l4xx_ll_gpio.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include "task.h"
#include "semphr.h"
#include "stream_buffer.h"

#include "keyboard.h"
#include "service.h"
#include "autopilot.h"
#include "printf.h"
#include "rlib.h"

#include <stm32l452xx.h>
#include <stm32l4xx_ll_gpio.h>

/****************************************************************************\
*    GPIO port and pin numbers                                               *
*****************************************************************************/
#define KEYBOARD_GPIO_PORT          GPIOC      /* Keys connected to port C */
#define KEYBOARD_PIN_MODE_AUTO      (0x1 << 0) /* Pin 0 : Mode auto, heading */
#define KEYBOARD_PIN_MODE_IDLE      (0x1 << 1) /* Pin 1 : Mode idle */
#define KEYBOARD_PIN_STBD_1         (0x1 << 2) /* Pin 2 : +1 */
#define KEYBOARD_PIN_PORT_1         (0x1 << 3) /* Pin 3 : -1 */
#define KEYBOARD_PIN_STBD_10        (0x1 << 4) /* Pin 4 : +10 */
#define KEYBOARD_PIN_PORT_10        (0x1 << 5) /* Pin 5 : -10 */
#define KEYBOARD_PIN_AUX_1          (0x1 << 7) /* Pin 7 : Auxiliary 1 */
#define KEYBOARD_PIN_AUX_2          (0x1 << 8) /* Pin 8 : Auxiliary 2 */

#define KEYBOARD_PIN_MASK           ( KEYBOARD_PIN_MODE_AUTO \
                                    | KEYBOARD_PIN_MODE_IDLE \
                                    | KEYBOARD_PIN_STBD_1    \
                                    | KEYBOARD_PIN_PORT_1    \
                                    | KEYBOARD_PIN_STBD_10   \
                                    | KEYBOARD_PIN_PORT_10   \
                                    | KEYBOARD_PIN_AUX_1     \
                                    | KEYBOARD_PIN_AUX_2)

#define KEYBOARD_PIN_ACTIVE_MASK    ( KEYBOARD_PIN_MODE_AUTO \
                                    | KEYBOARD_PIN_MODE_IDLE \
                                    | KEYBOARD_PIN_STBD_1    \
                                    | KEYBOARD_PIN_PORT_1    \
                                    | KEYBOARD_PIN_STBD_10   \
                                    | KEYBOARD_PIN_PORT_10)

#define KEYBOARD_PIN_SHIFTS_MASK    ( KEYBOARD_PIN_AUX_1     \
                                    | KEYBOARD_PIN_AUX_2)

#define KEYBOARD_PIN_REPEAT_MASK    ( KEYBOARD_PIN_STBD_1    \
                                    | KEYBOARD_PIN_PORT_1    \
                                    | KEYBOARD_PIN_STBD_10   \
                                    | KEYBOARD_PIN_PORT_10)

static QueueHandle_t keyboard_Queue_Msg = NULL;
static TimerHandle_t keyboard_Timer = NULL;

/*
 * @brief Timer callback function for the keyboard task.
 * This function is called periodically by the timer.
 * It sends a message to the keyboard task to read the keyboard state.
 * @global keyboard_Queue_Msg The message queue for the keyboard task.
 * @param xTimer The timer handle (not used).
 * @return None
 */
void Keyboard_Timer_Callback(TimerHandle_t xTimer)
{
    (void)xTimer; /* statement to avoid unused parameter warning */

    /* The message is in a struct, declared static to avoid using stack.*/
    static Keyboard_Msg_t command = {.msgType = KEYBOARD_MSG_READ};

    xQueueSend(keyboard_Queue_Msg, (const void *)&command, (TickType_t)0);

    return;

}

/*
 * @brief Initializes the keyboard task.
 * This function creates the message queue and the timer for the keyboard task.
 * It must be called before starting the FreeRTOS scheduler.
 * @global keyboard_Queue_Msg The message queue for the keyboard task.
 * @global keyboard_Timer The timer for the keyboard task.
 * @param None
 * @return None
 */
void Keyboard_task_init()
{

    /* MEMs task queue creation */
    keyboard_Queue_Msg  = xQueueCreate(1, sizeof(Keyboard_Msg_t));

    /* Create a timer that sends messages periodically to the task */
    keyboard_Timer = xTimerCreate("KBD", pdMS_TO_TICKS(KEYBOARD_SCAN_PERIOD_MS),
                               pdTRUE, /* Auto reload (repeat indefinitely) */
                               (void *)0, /* Timer ID, not used */
                               Keyboard_Timer_Callback);
}

/*
 * @brief The main function for the keyboard task.
 * This function runs in an infinite loop, waiting for messages from the timer.
 * When a message is received, it reads the keyboard state and sends commands
 * to AP task.
 * @global keyboard_Queue_Msg The message queue for the keyboard task.
 * @param param The parameter passed to the task (not used).
 * @return None, mustn't return
 */
void Keyboard_task(void *param)
{
    (void) param; /* Avoid unused parameter warning */

    BaseType_t     ret;           /* return val */
    Keyboard_Msg_t msgKeyboard;   /* Message received from queue */
    uint32_t       cptrpt;        /* number of times key is pressed */
    uint16_t       keys;          /* keys pressed */
    uint16_t       nbActiveKeys;  /* number of active key pressed */
    bool           jamkeys;       /* too many key pressed */

    char           message[100];  /* for debugging purpose only */

    cptrpt  = 0U;
    jamkeys = false;

    /* Wait before reading keyboard and sending commands */
    vTaskDelay(pdMS_TO_TICKS(KEYBOARD_DELAY_BEFORE_START));

    xTimerStart(keyboard_Timer, (TickType_t)0);

    for (;;) /* infinite loop */
    {
        ret = xQueueReceive(keyboard_Queue_Msg, &msgKeyboard, pdMS_TO_TICKS(0));
        if (ret == pdPASS)
        {
            keys = LL_GPIO_ReadInputPort(KEYBOARD_GPIO_PORT);
            /* push keys connected to 0V and pins, pullup resistors of GPIO
             * pulls to VDD so when key not pressed pin value is
             * 1 when key unpressed and 0 when pressed, invert with ~ operator */
            keys = ~keys;
            /* read 16 bits of GPIO port even */
            keys = keys & KEYBOARD_PIN_MASK;

#if 0
            snprintf (message, sizeof (message), "KBD : 0x%4x    %x %x   %x %x   %x %x   %x %x\n",
                      keys & KEYBOARD_PIN_MASK,
                      ((keys & KEYBOARD_PIN_MODE_AUTO) !=0 ),
                      ((keys & KEYBOARD_PIN_MODE_IDLE) !=0 ),
                      ((keys & KEYBOARD_PIN_STBD_1)    !=0 ),
                      ((keys & KEYBOARD_PIN_PORT_1)    !=0 ),
                      ((keys & KEYBOARD_PIN_STBD_10)   !=0 ),
                      ((keys & KEYBOARD_PIN_PORT_10)   !=0 ),
                      ((keys & KEYBOARD_PIN_AUX_1)     !=0 ),
                      ((keys & KEYBOARD_PIN_AUX_2)     !=0 )
                      );
            svc_UART_Write(&SERVICE_UART_LOG, message, strlen(message), pdMS_TO_TICKS(1));
#endif

            if (keys == 0U)
            {
                cptrpt  = 0U;
                jamkeys = false;
            }
            else
            {
                /* Count active keys pressed */
                nbActiveKeys=0;
                if (keys & KEYBOARD_PIN_MODE_AUTO)
                    nbActiveKeys++;
                if (keys & KEYBOARD_PIN_MODE_IDLE)
                    nbActiveKeys++;
                if (keys & KEYBOARD_PIN_STBD_1)
                    nbActiveKeys++;
                if (keys & KEYBOARD_PIN_PORT_1)
                    nbActiveKeys++;
                if (keys & KEYBOARD_PIN_STBD_10)
                    nbActiveKeys++;
                if (keys & KEYBOARD_PIN_PORT_10)
                    nbActiveKeys++;

                /* No more than on active key pressed */
                if (nbActiveKeys > 1)
                {
                    jamkeys = true;
                }
                if ((nbActiveKeys == 1U) && (jamkeys == false))
                {
                    if ((cptrpt == 0) || ((cptrpt >= KEYBOARD_DELAY_REPEAT_MS)  && ((cptrpt-KEYBOARD_DELAY_REPEAT_MS) % KEYBOARD_PERIOD_REPEAT_MS) == 0U) )
                    {
                        switch(keys){

                        case KEYBOARD_PIN_MODE_AUTO:

                            AutoPilot_msg_mode_heading();

                            break;

                        case KEYBOARD_PIN_MODE_IDLE:

                            AutoPilot_msg_mode_idle();

                            break;

                        case KEYBOARD_PIN_STBD_1:

                            AutoPilot_msg_turn_deg(+1);

                            break;

                        case KEYBOARD_PIN_PORT_1:

                            AutoPilot_msg_turn_deg(-1);

                            break;

                        case KEYBOARD_PIN_STBD_10:

                            AutoPilot_msg_turn_deg(+10);

                            break;

                        case KEYBOARD_PIN_PORT_10:

                            AutoPilot_msg_turn_deg(-10);

                            break;

                        default:/* no key pressed */

                            break;
                        } /* switch (keys)*/
                    }

                    cptrpt += KEYBOARD_SCAN_PERIOD_MS;
                }

            } /* else if (keys==0) */

        } /* if (ret == pdPASS) message read keys and process keys*/

    } /* end of infinite loop */

    /* Mustn't happen */
}
