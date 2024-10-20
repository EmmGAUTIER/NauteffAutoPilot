#include "stdio.h"
#include "string.h"
#include <ctype.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stream_buffer.h"

#include "util.h"
#include "gpio.h"
#include "usart.h"
#include "dialog.h"

int tour = 0;

typedef struct {
    int code;
}Command;

inline void skip_blanks(const char* str, int *idx)
{
    int deb = *idx;
    while (isblank(str[deb]))
    {
        deb++;
    }
    *idx=deb;
}

inline int get_word(const char* str, int *idx)
{
    (void) str;
    (void) idx;
     return 0;
    }

#if 0
int dialogDecode(const char* str, Command &command)
{
    cmdCode=0;
    int idx=0; /* Index char of command */

    /* Skip blanks, white spaces and tabs */
    skip_blanks(str, &idx);


    return ;
}
#endif

void fct()
{
    return;
}

void __attribute__((noreturn)) taskDialog(void *args __attribute__((unused)))
{
    static char buff[120];
    static char message[80];
    int ret;
    //int sr;
    for (;;)
    {
#if 1
    
#endif
    message[0] = '\0';
    ret = USART_read(usart1, message, 80, 1000);
    if (ret>0) {
        int idxcar = 0;
        int itemcnt;
        int capdeg;
        message [ret] = '\0';
        sprintf(buff, "###>> Reçu %d : %s\n", ret, message);
        USART_write(usart1, buff, strlen(buff), 0U);
    
        while (isblank(idxcar)) {
            idxcar++;
        }
        while (tolower)


        itemcnt = sscanf (message, "AP HEADING %d", &capdeg);
        if (itemcnt > 0) {
            sprintf(buff, "Barrer au cap %d\n", capdeg);
            USART_write(usart1, buff, strlen(buff), 0U);
        }

    }

#if 0
        
        sprintf(buff, "usart1 / Tour : %d\n", tour);
        USART_write(usart1, buff, strlen(buff), 0U);
        //sr = USART_read(usart5, message, sizeof (message)-1, 1000);
        //message[sr >= 0 ? sr:0] = '\0';
        //sprintf (buff, "  Réponse %d \"%s\"\n", sr, message);
        //USART_write(usart5, buff, strlen(buff), 0U);
        vTaskDelay(pdMS_TO_TICKS(500));

#endif
    //vTaskDelay(pdMS_TO_TICKS(500));
    tour++;
    }
}
