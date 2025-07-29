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

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "message_buffer.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "util.h"
#include "nassert.h"
#include "gpio.h"
#include "nvic.h"
#include "rcc.h"
#include "usart.h"
#include "tim.h"
#include "adc.h"

#include "dialog.h"
#include "mems.h"
#include "nauteff.h"
#include "motor.h"
#include "autopilot.h"

void taskBlink(void * /* parameters*/)
/*
 * \brief Blink the green LED on the Nucleo board.
 * This task is used for debugging and to check if the system is running.
 */
{
    for (;;)
    {
        GPIO_pin_set(GPIOA, 5);
        vTaskDelay(pdMS_TO_TICKS(200));

        GPIO_pin_reset(GPIOA, 5);
        vTaskDelay(pdMS_TO_TICKS(800));
    }
}

void taskEssais(void *)
/* This task is used for debugging */
{
    for (;;)
    {
        auxPinUp();
        // USART_raw_put_char(USART1, 'V');
        // USART_raw_put_char(USART2, 'V');
        vTaskDelay(25);
        auxPinDown();
        // USART_raw_put_char(USART1, 'U');
        // USART_raw_put_char(USART2, 'U');
        vTaskDelay(175);
    }
}

int main()
{

    /* PA0 : VPWR, Power voltage */
    GPIO_init_pin_adc(GPIOA, 0);

    /* PA1 : CURRENT, motor current */
    GPIO_init_pin_adc(GPIOA, 1);

    /* PA2 : USART2 TX */
    GPIO_init_pin_altfct(GPIOA, 2,
                         GPIO_OTYPER_PUSHPULL,
                         GPIO_OSPEEDR_HIGHSPEED,
                         GPIO_PUPDR_NONE, 7);

    /* PA3 : USART2 RX */
    GPIO_init_pin_altfct(GPIOA, 3,
                         GPIO_OTYPER_PUSHPULL,
                         GPIO_OSPEEDR_HIGHSPEED,
                         GPIO_PUPDR_NONE, 7);

    /* PA4 : output : Motor run command */
    GPIO_init_pin_output(GPIOA, 4,
                         GPIO_OTYPER_PUSHPULL,
                         GPIO_OSPEEDR_LOWSPEED,
                         GPIO_PUPDR_NONE);

    /* PA5 : outputs : clutch and green LED  of Nucleo board */
    GPIO_init_pin_output(GPIOA, 5,
                         GPIO_OTYPER_PUSHPULL,
                         GPIO_OSPEEDR_LOWSPEED,
                         GPIO_PUPDR_NONE);

    /* PA6 : INA, motor direction */
    GPIO_init_pin_output(GPIOA, 6,
                         GPIO_OTYPER_PUSHPULL,
                         GPIO_OSPEEDR_LOWSPEED,
                         GPIO_PUPDR_NONE);

    /* PA7 : INB, motor direction */
    GPIO_init_pin_output(GPIOA, 7,
                         GPIO_OTYPER_PUSHPULL,
                         GPIO_OSPEEDR_LOWSPEED,
                         GPIO_PUPDR_NONE);

    /* PA8 altfct 0 : MCO or auxiliary output */
    GPIO_init_pin_altfct(GPIOA, 8,
                         GPIO_OTYPER_PUSHPULL,
                         GPIO_OSPEEDR_HIGHSPEED,
                         GPIO_PUPDR_NONE, 0);

    /* PA9 : USART1 TX */
    GPIO_init_pin_altfct(GPIOA, 9,
                         GPIO_OTYPER_PUSHPULL,
                         GPIO_OSPEEDR_HIGHSPEED,
                         GPIO_PUPDR_NONE, 7);

    /* PA10 : USART1 RX */
    GPIO_init_pin_altfct(GPIOA, 10,
                         GPIO_OTYPER_PUSHPULL,
                         GPIO_OSPEEDR_HIGHSPEED,
                         GPIO_PUPDR_NONE, 7);

    /* PA11 : CAN1 TX */
    GPIO_init_pin_altfct(GPIOA, 11,
                         GPIO_OTYPER_PUSHPULL,
                         GPIO_OSPEEDR_HIGHSPEED,
                         GPIO_PUPDR_NONE, 9);

    /* PA12 : CAN1 RX */
    GPIO_init_pin_altfct(GPIOA, 12,
                         GPIO_OTYPER_PUSHPULL,
                         GPIO_OSPEEDR_HIGHSPEED,
                         GPIO_PUPDR_NONE, 9);

    /* PA13 JTMS-SWDIO : already configured */
    /* PA14 JTMS-SWDCLK : already configured */

    /* PA15 : ALARM */
    GPIO_init_pin_output(GPIOA, 15,
                         GPIO_OTYPER_PUSHPULL,
                         GPIO_OSPEEDR_LOWSPEED,
                         GPIO_PUPDR_NONE);

    /* PB0 : Unused */
    GPIO_init_pin_unused(GPIOB, 0);

    /* PB1 : AGRDY, ACC & GYR Ready */
    GPIO_init_pin_input(GPIOB, 1,
                        GPIO_PUPDR_PULLDOWN);

    /* PB2 MAGRDY, MAG Ready*/
    GPIO_init_pin_input(GPIOB, 2,
                        GPIO_PUPDR_PULLDOWN);

    /* PB3 : Unused */
    GPIO_init_pin_unused(GPIOB, 3);

    /* PB4 : Unused */
    GPIO_init_pin_unused(GPIOB, 4);

    /* PB5 : Unused */
    GPIO_init_pin_unused(GPIOB, 5);

    /* PB6 : Unused */
    GPIO_init_pin_unused(GPIOB, 6);

    /* PB7 : I2C1 SDA */
    GPIO_init_pin_altfct(GPIOB, 7,
                         GPIO_OTYPER_OPENDRAIN,
                         GPIO_OSPEEDR_HIGHSPEED,
                         GPIO_PUPDR_NONE, 4);

    /* PB8 : I2C1 SCL */
    GPIO_init_pin_altfct(GPIOB, 8,
                         GPIO_OTYPER_OPENDRAIN,
                         GPIO_OSPEEDR_HIGHSPEED,
                         GPIO_PUPDR_NONE, 4);

    /* PB9 : Unused */
    GPIO_init_pin_unused(GPIOB, 9);

    /* PB10 : Unused */
    GPIO_init_pin_unused(GPIOB, 10);

    /* PB11 : Unused */
    GPIO_init_pin_unused(GPIOB, 11);

    /* PB12 : Unused */
    GPIO_init_pin_unused(GPIOB, 12);

    /* PB13 : Unused */
    GPIO_init_pin_unused(GPIOB, 13);

    /* PB14 : Unused */
    GPIO_init_pin_unused(GPIOB, 14);

    /* PB15 : Unused */
    GPIO_init_pin_unused(GPIOB, 15);

    /*
     * PORT c:
     */
    /* PC0 : Unused */
    GPIO_init_pin_unused(GPIOC, 0);

    /* PC1 : Unused */
    GPIO_init_pin_unused(GPIOC, 1);

    /* PC2 : Unused */
    GPIO_init_pin_unused(GPIOC, 2);

    /* PC3 : Unused */
    GPIO_init_pin_unused(GPIOC, 3);

    /* PC4 : Unused */
    GPIO_init_pin_unused(GPIOC, 4);

    /* PC5 : Unused */
    GPIO_init_pin_unused(GPIOC, 5);

    /* PC6 : Unused */
    GPIO_init_pin_unused(GPIOC, 6);

    /* PC7 : Unused */
    GPIO_init_pin_unused(GPIOC, 7);

    /* PC8 : Auxiliary output for debugging, connected to LED */
    GPIO_init_pin_output(GPIOC, 8,
                         GPIO_OTYPER_PUSHPULL,
                         GPIO_OSPEEDR_HIGHSPEED,
                         GPIO_PUPDR_NONE);

    /* PC9 : Unused */
    GPIO_init_pin_unused(GPIOC, 9);

    /* PC10 : Unused */
    GPIO_init_pin_unused(GPIOC, 10);

    /* PC11 : Unused */
    GPIO_init_pin_unused(GPIOC, 11);

    /* PC12 : Unused */
    GPIO_init_pin_unused(GPIOC, 12);

    /* PC13 : Blue Button */
    GPIO_init_pin_input(GPIOC, 13,
                        GPIO_PUPDR_NONE);

    /* PC14 : Quartz, don't touch */

    /* PC15 : Quartz, don't touch */

    /* USART 1 */
    USART_set_baudrate(usart1, 38300);
    USART_set_parity_none(usart1);
    USART_set_stop_bits(usart1, twoStopBits);
    USART_set_buffer_mode(usart1, lineBuffered);
    USART_enable(usart1);
    NVIC_enable_IRQ(NVIC_NUM_IRQ_USART1);
    NVIC_SetPriority(NVIC_NUM_IRQ_USART1, 0xC);

    /* USART2 */
    USART_set_baudrate(usart2, 38400);
    USART_set_parity_none(usart2);
    USART_set_stop_bits(usart2, oneStopBit);
    USART_set_buffer_mode(usart2, lineBuffered);
    USART_enable(usart2);
    NVIC_enable_IRQ(NVIC_NUM_IRQ_USART2);
    NVIC_SetPriority(NVIC_NUM_IRQ_USART2, 0xC);

    /* TIM6 and ADC has to start conversions when queues are created  */
    /*so TIM6 and ADC configuration is done at the begining of motor task */
    /* ADC CKMode can be done now */
    ADC_init_all();

    /* I2C1 */
    NVIC_enable_IRQ(NVIC_NUM_IRQ_I2C1_EV);
    NVIC_enable_IRQ(NVIC_NUM_IRQ_I2C1_ER);
    NVIC_SetPriority(NVIC_NUM_IRQ_I2C1_EV, 0xB);
    NVIC_SetPriority(NVIC_NUM_IRQ_I2C1_ER, 0xC);

    /* TODO : Réglages de priorités */
    NVIC_enable_IRQ(NVIC_NUM_IRQ_ADC);
    NVIC_SetPriority(NVIC_NUM_IRQ_ADC, 0xE);

    /* Objects creations and initialisations */
    init_taskMotor();
    init_taskDialog();
    init_taskMEMs();
    init_taskAutoPilot();

    void taskMEMs_Claude(void *);

    xTaskCreate(taskMEMs, "LSM9DS1", configMINIMAL_STACK_SIZE + 400, NULL, 1, NULL);
    xTaskCreate(taskAutoPilot, "AutoPilot", configMINIMAL_STACK_SIZE + 400, NULL, 3, NULL);
    xTaskCreate(taskMotor, "Motor", configMINIMAL_STACK_SIZE + 400, NULL, 2, NULL);
    xTaskCreate(taskDialogIn, "DialogIn", configMINIMAL_STACK_SIZE + 400, NULL, 2, NULL);
    xTaskCreate(taskBlink, "Blink", configMINIMAL_STACK_SIZE + 200, NULL, 2, NULL);
    xTaskCreate(taskEssais, "Essais", configMINIMAL_STACK_SIZE + 200, NULL, 2, NULL);

    vTaskStartScheduler();

    for (;;)
    {
        ;
    }
}
