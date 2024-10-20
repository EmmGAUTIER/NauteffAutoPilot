#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include <message_buffer.h>
#include <semphr.h>

#include "util.h"
#include "usart.h"
#include "bno055.h"

#include "task.h"
#include "timers.h"
#include "util.h"

#define QUEUE_LENGTH 10
#define ITEM_SIZE sizeof(uint32_t)

static QueueHandle_t xQueueMEMs = 0;

//static StaticQueue_t xStaticQueue;
//static uint8_t ucQueueStorageArea[ QUEUE_LENGTH * ITEM_SIZE ];
static TimerHandle_t timerMEMs;
//static StaticTimer_t xStaticTimer;

/*
 * 
 */

void timerMEMsCallback(TimerHandle_t xTimer)
{
	(void)xTimer;
	static char message[100];
    static uint32_t command = 0x01;
	xQueueSend(xQueueMEMs, (const void*)&command, (TickType_t)0);

	// sprintf (message, "====> Appel timerMEMsCallback\n");
	// USART_write(usart1, message, strlen(message), 0U);
}

/**
 * \brief MEMMs task
 */

void taskMEMs(void *parameters)
{
	(void)parameters;
	char data[10];
	int ret;
	uint32_t command;
	static char message[100];
	int compteur = 0;

    /* Queue creation*/
    xQueueMEMs = xQueueCreate(QUEUE_LENGTH,
                                    ITEM_SIZE);

	bno055->usart = usart5;
	bno055->errno = 421;

    /* Sélection de la page 0 */
	char cmd_page0[] = {0xAA, 0x00, 0x07, 0x01, 0x00};
    USART_write (usart5, (const void*)cmd_page0, sizeof(cmd_page0), 1000U);

    /* Reset, mise à un du bit 5 registre 0x3F avec delai 600ms */
	char cmd_reinit[] = {0xAA, 0x00, 0x3F, 0x01, 0x1<<5};
    USART_write (usart5, (const void*)cmd_reinit, sizeof(cmd_reinit), 1000U);
	vTaskDelay(pdMS_TO_TICKS(600)); /* wait 600ms cf. sect. 3.3, table 3.6*/

    /* Remise à zéro du bit 5 registre 0x3F avec delai 600ms */
	char cmd_restrt[] = {0xAA, 0x00, 0x3F, 0x01, 0x00};
    USART_write (usart5, (const void*)cmd_restrt, sizeof(cmd_restrt), 1000U);
	vTaskDelay(pdMS_TO_TICKS(100)); 

    /* Mode NDOF */
	char cmd_NDOF[] = {0xAA, 0x00, 0x3D, 0x01, 0x0C};
    USART_write (usart5, (const void*)cmd_NDOF, sizeof(cmd_NDOF), 1000U);
	vTaskDelay(pdMS_TO_TICKS(100)); 

    char cmd_units = {0xAA, 0x00, 0x3B, 0x01, 0x06};
    USART_write (usart5, (const void*)cmd_units, sizeof(cmd_units), 1000U);
	vTaskDelay(pdMS_TO_TICKS(100)); 

#if 0
	/* Réinitialisation du capteur */
	//data[0] = 0x1<<5;
	// ret = USART_write(usart5, data, )
	// ret = BNO055_write(bno055, 0x3F, data, 1);
	// if (ret != 1) {
	    // sprintf(message, "Retour BNO055_write réinit. :  %d errno = %d\n", ret, bno055->errno);
	    // USART_write(usart1, message, strlen(message), 0U);
	// }

	//vTaskDelay(pdMS_TO_TICKS(600)); /* wait 600ms cf. sect. 3.3, table 3.6*/

	/* Config Mode  */
	//char cmd_reinit[] = {0xAA, 0x00, 0x3F, 0x01, 0x1<<5};
    //USART_write (usart5, (const void*)cmd_reinit, sizeof(cmd_reinit), 1000U);
	//vTaskDelay(pdMS_TO_TICKS(600)); /* wait 600ms cf. sect. 3.3, table 3.6*/

	//ret = USART_read (usart5, data, 3, 1000U);
	//sprintf (message, "------------ 1 >>>>>>  Réponse %d : %x %x\n", ret, data[0], data[1]);
	//USART_write(usart1, message, strlen(message), 0U);

	//vTaskDelay(pdMS_TO_TICKS(600)); /* wait 600ms cf. sect. 3.3, table 3.6*/

	//char cmd_mode [] = {0xAA, 0x00, 0x3D, 0x01, 0x0C};
    //ret = USART_write (usart5, (const void*)cmd_mode, sizeof(cmd_reinit), 1000U);
	//sprintf (message, "------------ 10 >>>>>>  Réponse %d\n", ret);
	//USART_write(usart1, message, strlen(message), 0U);

    //data[0] = data[1] = 0xFF;
	//ret = USART_read (usart5, data, 2, 1000U);
	//sprintf (message, "------------ 11 >>>>>>  Réponse %d : %x %x\n", ret, data[0], data[1]);
	//USART_write(usart1, message, strlen(message), 0U);


	//ret = BNO055_write(bno055, 0x3D, data, 1);
	//if (ret !=1) {
	    //sprintf(message, "Retour BNO055_write cfg mode: %d errno = %d\n", ret, bno055->errno);
	    //USART_write(usart1, message, strlen(message), 0U);
	//}

	//vTaskDelay(pdMS_TO_TICKS(600)); /* wait 600ms cf. sect. 3.3, table 3.6*/

	/* Sélection des unités :
	 * accélération en m/s2*,
	 * Gyrometer and EULER angles in radians
	 */
	data[0] = 0x06;
	ret = BNO055_write(bno055, 0x3B, data, 1);
	if (ret != 1) {
	    sprintf(message, "Retour BNO055_write unités : %d errno = %d\n", ret, bno055->errno);
	    USART_write(usart1, message, strlen(message), 0U);
	}

	/* Mode NDOF : Nine degrees of Fredom and data fusion */
	data[0] = 0x0C;
	ret = BNO055_write(bno055, 0x3D, data, 1);
	if (ret != 1) {
	    sprintf(message, "Retour BNO055_write NDOF : %d errno = %d\n", ret, bno055->errno);
	    USART_write(usart1, message, strlen(message), 0U);
	}

	data[0] = 0xFF;
	ret = BNO055_read(bno055, 0x3B, data, 1);
	if (ret != 1) {
	    // sprintf(message, "Retour BNO055_read : %d errno = %d valeur = %x\n", ret, (unsigned int)bno055->errno, (unsigned int)data[0]);
	    // USART_write(usart1, message, strlen(message), 0U);
	}
#endif

    /* Start timer */
	timerMEMs =  xTimerCreate("MEMs",
                              //pdMS_TO_TICKS(20),// 20ms ? /* Every 200 ms : 5 Hz*/
							  100,
                              pdTRUE, /* Auto reload (repeat indefinitely) */
                              (void*)0,
                              timerMEMsCallback);

	sprintf(message, "Retour xTimerCreate : %x\n", (uint32_t)timerMEMs);
	USART_write(usart1, message, strlen(message), 0U);


    ret = xTimerStart(timerMEMs, 0);
	sprintf(message, "Retour xTimerStart : %x\n", ret);
	USART_write(usart1, message, strlen(message), 0U);
    
	for (;;)
	{
		union
		{
			int16_t vect[4];
			char buff[8];
		} values;
		float norme;
        
        ret = xQueueReceive(xQueueMEMs, &command, pdMS_TO_TICKS(5000));
		//sprintf(message, "Retour xQueueReceive : %x\n", ret);
    	//USART_write(usart1, message, strlen(message), 0U);

		ret = -421;
		ret = BNO055_read(bno055, 0x1A, &values, 6);

		if (ret == 6)
		{
			norme = sqrt((float) (values.vect[0] * values.vect[0] + values.vect[1]*values.vect[1]  + values.vect[2]*values.vect[2] ));
			sprintf(message, "ATTITUDE %f %f %f %d\n", (float)(values.vect[0]/900.), (float)(values.vect[1]/900.), (float)(values.vect[2]/900.));
			USART_write(usart1, message, strlen(message), 0L);

		}
		else {
			sprintf(message, "  ------> tour = %d  CR = %d  errno =  %5d  / %x\n",
				compteur, ret, bno055->errno, (unsigned int)bno055->errno_EE);
			USART_write(usart1, message, strlen(message), 0L);

		}

		//vTaskDelay(pdMS_TO_TICKS(200));
		compteur++;
	}
}

#if 0
void taskMEMs_old(void *parameters)
{
	int ret;
	//static char dmd_id[] = {0xAA, 0x01, 0x00, 0x04};
	static char reponse[20];
	static char message[100];
	static char data[10];

    USART_flush_rx_buffer(usart5);
    USART_flush_rx_buffer(usart5);

	/* Pour vérification uniquement, à supprimer ultérieurement */
	ret = BNO055_read(&bno055_Handle, 0x00, reponse, 6);
	/* Envoi du CR au raspberry pi par usart 1 */
	if (ret > 0)
	{
		sprintf(message, "Réponse BNO055 retour=%d, %x %x %x %x %x %x\n", ret,
				reponse[0], reponse[1], reponse[2], reponse[3], reponse[4], reponse[5]);
	}
	else
	{
		sprintf(message, "Réponse BNO055 retour=%d,  errno = %d / 0x%x\n", bno055_Handle.errno, bno055_Handle.errno);
	}
	USART_write(usart1, message, strlen(message), 0L);
	/* Fin vérificatoin */
#if 0
	ret = BNO055_init(&bno055_Handle, usart5);
	sprintf(message, "Initialisation BNO055 retour = %d,  errno = %d / 0x%x\n", bno055_Handle.errno, bno055_Handle.errno);
	vTaskDelay(pdMS_TO_TICKS(20));
#endif
	BNO055_init(&bno055_Handle, usart5);

    data[0] = 0x0C;
    ret = BNO055_write(usart5, 0x3D, data, 1);
	sprintf (message, "BNO055_write return : %d, errno = %d\n", ret, bno055->errno);
	USART_write(usart1, message, strlen(message), 0L);
	vTaskDelay(pdMS_TO_TICKS(1000));
    
	for (;;)
	{
		union
		{
			char buffer[6];
			int16_t values[3]
		} data;

		ret = BNO055_read(&bno055_Handle, 0x20, &data, 6);
		sprintf(message, "  --> CR = %d  values = () %5d  %5d  %5d)\n",
		        ret,
				data.values[0], data.values[1], data.values[2]);
		USART_write(usart1, message, strlen(message), 0L);

		vTaskDelay(pdMS_TO_TICKS(500));
	}
}
#endif
