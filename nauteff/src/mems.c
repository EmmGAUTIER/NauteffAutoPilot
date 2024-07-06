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

/**
 * \brief MEMMs task
 */

void taskMEMs(void *parameters)
{
	(void)parameters;
	char data[10];
	int ret;
	static char message[100];

	bno055->usart = usart5;
	bno055->errno = 421;

	/* Réinitialisation du capteur */
	data[0] = 0x00;
	ret = BNO055_write(bno055, 0x3F, data, 1);
	sprintf(message, "Retour BNO055_write : %d errno = %d\n", ret, bno055->errno);
	USART_write(usart1, message, strlen(message), 0U);

	vTaskDelay(pdMS_TO_TICKS(600)); /* wait 600ms cf. sect. 3.3, table 3.6*/

	/* Config Mode  */
	data[0] = 0x00;
	ret = BNO055_write(bno055, 0x3D, data, 1);
	sprintf(message, "Retour BNO055_write : %d errno = %d\n", ret, bno055->errno);
	USART_write(usart1, message, strlen(message), 0U);

	/* Sélection des unités */
	data[0] = 0x06;
	ret = BNO055_write(bno055, 0x3B, data, 1);
	sprintf(message, "Retour BNO055_write : %d errno = %d\n", ret, bno055->errno);
	USART_write(usart1, message, strlen(message), 0U);

	/* Mode NDOF */
	data[0] = 0x0C;
	ret = BNO055_write(bno055, 0x3D, data, 1);
	sprintf(message, "Retour BNO055_write : %d errno = %d\n", ret, bno055->errno);
	USART_write(usart1, message, strlen(message), 0U);

	data[0] = 0xFF;
	ret = BNO055_read(bno055, 0x3B, data, 1);
	sprintf(message, "Retour BNO055_read : %d errno = %d valeur = %x\n", ret, (unsigned int)bno055->errno, (unsigned int)data[0]);
	USART_write(usart1, message, strlen(message), 0U);

	for (;;)
	{
		union
		{
			int16_t vect[4];
			char buff[8];
		} values;
		//float norme;

		ret = BNO055_read(bno055, 0x1A, &values, 6);

		if (ret == 6)
		{
			//norme = sqrt((float) (values.vect[0] * values.vect[0] + values.vect[1]*values.vect[1]  + values.vect[2]*values.vect[2] ));
			sprintf(message, "HDG/M  %d \n", (int)(values.vect[0]/15.70));
					//ret,
                    //values.vect[0], values.vect[1], values.vect[2]);
					//values.vect[0]/(3.14159*5), (float)values.vect[1], (float)values.vect[2]/*, norme*/);

		}
		else {
			sprintf(message, "  --> CR = %d  errno =  %5d  %5d / %x\n",
					ret,bno055->errno, bno055->errno);
		}
		USART_write(usart1, message, strlen(message), 0L);

		vTaskDelay(pdMS_TO_TICKS(200));
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