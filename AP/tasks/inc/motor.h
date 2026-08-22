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
 * @file Module "motor"
 * Ce module assure le contrôle du moteur.
 *  - Commande de l'embrayage et allumage de la LED verte de la carte Nucleo
 *  - Mise en marche et arrêt du moteur dans un sens ou l'autre
 *  - Détection de défaut ou de butée
 *  - Gestion de la durée de fonctionnement
 *  - Estimation de l'effort et ajustement de la durée de fonctionnement
 *  - Communication avec la tâche principale
 */

#include "FreeRTOS.h"
#include "queue.h"

#include "stm32l4xx_ll_gpio.h"
#include <stm32l452xx.h>

#include "util.h"

int Motor_task_init();
void Motor_task();

/*
 * @brief Send command to engage the actuator to motor task.
 * Upon reception of this command the motor task engages the clutch
 * and waits for steering angles from autopilot task.
 * @param void
 * @return void
 */
void Motor_msg_engage_actuator();

/*
 *
 * @brief Send command to let out the clutch to motor task
 * Upon reception of this command the motor task disengages the actuator
 * and stops motor.
 * @param void
 * @return void
 */
void Motor_msg_disengage_actuator();

/*
 * @brief Send steering angle to motor task
 * Upon reception of this command if the angle
 * is significantly different from angle the motor task
 * steers the helm to the given angle.
 * The command has no effect if the Cactuator is disengaged.
 * @param angle angle to steer is in radians
 * @return void
 */
void Motor_msg_set_helm_angle(float angle);

/**
 * @brief send the order move for a time
 * @param time to move in seconds  counterclockwise if negative, clockwise if positive
 * Used to move helm when not in auto mode.
 * Task motor send a message when move done.
 * Previous move order for time is discarded if motor was running same direction.
 * Move order is ignored if if motor is running oposite direction.
 * Used for moving continuously with repeated pushes on button.
 * @return none
 */
void Motor_msg_move_time(float time);

/*
 * @brief Set the conversion factor between helm angle and time
 * @param cvt Conversion factor angle (radians) to time (seconds)
  * cvt has to be positive. in a future version it may be used  revert
 * the direction of the motor instead of swaping motor wires.
 * @return void
 */
void Motor_msg_set_cvt_angle_time(float cvt);

/*
 * @brief Set the delta of gain of starboard and port
 * @param ddg delta of gain : 0 no difference of gain
 * The tiller move more in one direction than in the other
 * move is multiplied py (1 + ddg) when moving starboard
 * and by (1 - ddg) when moving to port.
 * ddg can be positive or negative.
 * @return void
 */
void Motor_msg_set_delta_dir_gain(float ddg);

/*
 * @brief Set the high pass filter coefficient
 * @param cvt Coefficient between 0 and 1 for the high pass filter
 * Usage is to be defined later.
 * @return void
 */
void Motor_msg_set_hpf_coeff(float cvt);

/*
 * @brief Set the threshold of angle to move the motor
 * @param thr threshold in radians
 * Motor is not moved if the estimated angle to move is below this threshold.
 * This is used to avoid moving the motor for very small angles.
 * Starting the motor consumes a lot of energy and is less efficient.
 * @return void
 */
void Motor_msg_set_threshold(float cvt);

/*
 @brief Asks motor task to display motor configuration
 @param none
 @return none
 */
void Motor_msg_display_config(void);

/*
 @brief Asks motor task to display motor status
 Prints status bits and some values
 @param none
 @return none
 */
void Motor_msg_display_status(void);


/*
 * @brief Stop motor in case pf panic.
 * This function stops the motor by resetting PA4, PA6 and PA7
 * which are connected to PWM, INA and INB of the motor driver.
 * It doesn't disengage the clutch.
 * It is meant to be called by fault exceptions handlers.
 * It is INLINE in order to use no function call nor stack.
 */
/* TODO test, make sure it is inlined, maybe use macro */
__attribute__((always_inline)) inline void MOTOR_stopPanic(void)
{
    /* Reset PWN, INA and INB */
    // LL_GPIO_ResetOutputPin(GPIOA,
    // LL_GPIO_PIN_4 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
    /* Very low level : register acces */
    GPIOA->BRR = LL_GPIO_PIN_4 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
}

