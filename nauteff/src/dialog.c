#include "stdio.h"
#include "string.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stream_buffer.h"

#include "util.h"
#include "gpio.h"
#include "usart.h"
#include "dialog.h"

int tour = 0;

void __attribute__((noreturn)) taskDialog(void *args __attribute__((unused)))
{
    static char buff[100];
    //static char message[150];
    //int sr;
    for (;;)
    {
#if 0
#endif

#if 1
        
        sprintf(buff, "usart1 / Tour : %d\n", tour);
        USART_write(usart1, buff, strlen(buff), 0U);
        //sr = USART_read(usart5, message, sizeof (message)-1, 1000);
        //message[sr >= 0 ? sr:0] = '\0';
        //sprintf (buff, "  Réponse %d \"%s\"\n", sr, message);
        //USART_write(usart5, buff, strlen(buff), 0U);
        vTaskDelay(pdMS_TO_TICKS(500));

#endif
        tour++;
    }
}
