/**
 * \Brief Functions that control the motor of the autopilot
 * \date november 10 2020
 * \author Emmanuel Gautier
 *
 * Ces fonction permettent le contrôle de l'actuateur, elles assurent:
 *   - la commande d'embrayage ;
 *   - la commande du moteur et le sens pendant un temps imparti ;
 *   - la mesure du courant ;
 *   - L'arrêt du moteur en cas de surcharge, de court-cicuit ou de détection de buttée
 *   - Une évaluation de la position de la barre ;
 *
 */

#include <math.h>
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "semphr.h"
#include "queue.h"
#if 0
#include <projdefs.h>
#include <stddef.h>
#include <stdlib.h>
#include <sys/stdint.h>
#endif
#include "task.h"
#include "timers.h"
#include "util.h"
#include "gpio.h"
#include "usart.h"
#include "nauteff.h"
#include "motor.h"

QueueHandle_t msgQueueMotor;

void LetInTheClutch(void);
void letOutTheClutch(void);
void runMotorToPort(void);
void stopMotor(void);
void runMotorToStarboard(void);

static void MotorTimerCallback(TimerHandle_t xTimer);

/**
 *  \brief lets in the clutch
 *  This function sends the command to let in the clutch by setting B3.
 *  It is called by taskMotor.
 *  On Nucleo card a green LED is connected to B3 and lights when clutch is in
 *  \param void
 *  \return void
 */
void LetInTheClutch(void)
{
	// GPIO B, broche No 3 à mettre à 1.
	// Sur la carte Nucleo il y a une Led verte reliée à cette broche.
	GPIOB->ODR |= (0x1 << 3);
	// GPIOB->BSRR = (0x1 << 3);
}

/**
 *  \brief lets out the clutch	TimerHandle_t motorTimer = xTimerCreate("MotorTimer", pdMS_TO_TICKS(10),
	pdTRUE, 0, MotorTimerCallback);
 *  This function sends the command to let out the clutch by resetting B3.
 *  It is called by taskMotor.
 *  \param void
 *  \return void
 */
void letOutTheClutch(void)
{
	// GPIO B, broche No 3 à mettre à 0.
	GPIOB->ODR &= ~(0x1 << 3);
	// GPIOB->BSRR = (0x1 << (3+16));
}

void runMotorToPort(void)
{
	// Les commandes du moteur sont sur le port GPIO B
	// B0 vers entrée PWM doit être au niveau haut
	// B4 vers INA doit être au niveau bas
	// B5 vers INB doit être au niveau haut
	GPIOB->BSRR = (0x1 << 0) | (0x1 << 5) | ((0x1 << 4) << 16);
}

/**
 * \brief Stops the motor
 * this function stops the motor by resetting B0, B4 and B5
 * \param void
 * \return void
 */
void stopMotor(void)
{
	// GPIOB->ODR &= ~(0x1 << 0);
	GPIOB->BSRR = ((0x1 << 0) << 16) | (0x1 << 4) << 16 | (0x1 << 5) << 16;
}

void runMotorToStarboard(void)
{
	GPIOB->BSRR = (0x1 << 0) | (0x1 << 4) | ((0x1 << 5) << 16);
}

/**
 * \brief Callback of timer and sends requests to taskMotor
 *
 * This callback function sends MSG_MOT_MOVING_CONTROL periodicaly
 * to taskMotor when motor is running. When taskMotor receive this message
 * it measures current compute an estimated position and checks current.
 *
 * \date November 10 2024
 */


typedef enum {
    MSG_MOTOR_CONTROL = 0,
} MessageMotorType_t;


void MotorTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

	MessageCore_t commande;
	commande.category = MSG_MOTOR;
    commande.type = MSG_MOTOR_CONTROL;

	static char message[100];
	// sprintf (message, "###### >>> Appel MotorTimerCallback\n");
	// USART_write (usart1, message, strlen(message), 0U);

	xQueueSendToBack (msgQueueMotor, &commande, 0U);
}

/**
 * \brief Motor control task
 */

void taskMotor(void *parameters)
{
	(void)parameters; /* parameters ignored, instr to avoid warning */

	size_t ret;
	MessageCore_t commande;

	// char enMarche = 0;
	// float MoveToDo = 0.F;
	// vuint32_t delay = 1000;
	// vuint32_t   tension;
	static char message[100];

	// sprintf (message, "@@@@@@   taskMotor Départ\n");
	// USART_write(usart1, message, strlen(message), 0U);

	msgQueueMotor = xQueueCreate (10, sizeof (MessageCore_t));

    /* Timer that calls periodically MotorTimerCallback wich sends a message */
	TimerHandle_t motorTimer = xTimerCreate("MotorTimer",
	                                        pdMS_TO_TICKS(100),
											pdTRUE,
											0,
											MotorTimerCallback);
    ret = xTimerStart(motorTimer, 0);

	for (;;)
	{
        // vTaskDelay(pdMS_TO_TICKS(200));
    	sprintf (message, "Allo ?\n");
	    USART_write(usart1, message, strlen(message), 0U);

        commande.category = -1;
        commande.type = -1;
		ret = xQueueReceive(msgQueueMotor,
		                    &commande,
		                    1000);

    	// sprintf (message, "Motor retour xMessageBufferReceive %x, %d\n", ret, (int)commande.category);
	    // USART_write(usart1, message, strlen(message), 0U);
	}
}

#if 0

		if (ret != (size_t)0)
		{
			switch (commande.msgCode)
			{

			case MSG_MOT_EMBRAYE:
				LetInTheClutch();
				break;

			case MSG_MOT_DEBRAYE:
				stopMotor();
				letOutTheClutch();
				xTimerStop(motorTimer, pdMS_TO_TICKS(10));
				delay = 1000;
				break;

			case MSG_MOT_STARBOARD:
				runMotorToStarboard();
				// enMarche = 1;
				xTimerStart(motorTimer, pdMS_TO_TICKS(10));
				break;

			case MSG_MOT_PORT:
				runMotorToPort();
				// enMarche = 1;
				xTimerStart(motorTimer, pdMS_TO_TICKS(10));
				break;

			case MSG_MOT_STOP:
				stopMotor();
				// enMarche = 0;
				delay = 1000;
				xTimerStop(motorTimer, pdMS_TO_TICKS(10));
				break;

			case MSG_MOT_MOVING_CONTROL:
				// Mesure du courant, evaluation de l'effort,
				// estimation du déplacement et calcul du temps restant
				// Si courant trop fort abandon et alarme

				if (enMarche)
					{;}
				if (MoveToDo)
					{;}
#if 0
				ADC2->CR |= 0x01 << 2;
				vTaskDelay(1);
				tension = ADC2->DR;
				if (enMarche) {
				    MoveToDo -= 1.F;
				    if (MoveToDo < 0.F) {
				    	enMarche = (char)0;
				    	stopMotor();
						delay = 1000;
						xTimerStop(motorTimer, pdMS_TO_TICKS(10));
				    }
				}
#endif
				message[0] = '\0';
				strcat(message, "MoveToDo=");
				// int tmp_int = ((int)MoveToDo*10.F);
				// inttoa(cnbre, sizeof(cnbre) - 1, tmp_int);
				// strcat(message, cnbre);
				// strcat(message, "\n");
				// dbgmsg(message);
				break;

			case MSG_MOT_MOVE_STARBOARD:
			case MSG_MOT_MOVE_PORT:
				MoveToDo = fabs(commande.msgMotorMove.angle);
				enMarche = 'M';
				if (commande.msgMotorMove.angle > 0.F)
				{
					runMotorToPort();
				}
				else
				{
					runMotorToStarboard();
				}
				xTimerStart(motorTimer, pdMS_TO_TICKS(10));

				message[0] = '\0';
				strcat(message, "Angle à tourner");
				// inttoa(cnbre, sizeof(cnbre) - 1, (int)commande.msgMotorMove.angle);
				strcat(message, cnbre);
				strcat(message, "\n");
				// dbgmsg(message);
				// printf ("Hello World !");
				break;

			case MSG_MOT_DMD_EFFORT:
#if 0			
				//dbgmsg("--> Mesure\n");
				// Mesure de la tension
				ADC2->CR |= 0x01 << 2;
				vTaskDelay(1);
				tension = ADC2->DR;

				message[0] = '\0';
				strcat(message, "i=");
				inttoa(cnbre, sizeof(cnbre) - 1, tension);
				strcat(message, cnbre);
				strcat(message, "\n");
				//write_USART(USART1, message, strlen(message));
				dbgmsg(message);
#endif
				break;
			}
		}
	}
}
#endif
