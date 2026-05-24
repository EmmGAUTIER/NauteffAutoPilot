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

/*****************************************************************************
 * Dialog task
 *
 * Reads from line buffered input
 *
 * turn port/starboard <angle>
 * mode idle|heading : set idle or heading mode
 *
 *
 *****************************************************************************/

#include "FreeRTOS.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "task.h"
#include "timers.h"

#include "printf.h"
#include "rlib.h"
#include "util.h"

#include "apmain.h"
#include "apdialog.h"
#include "autopilot.h"
#include "mems.h"
#include "motor.h"
#include "service.h"
#include <stm32l452xx.h>
#include <stm32l4xx_ll_gpio.h>

#define DBG_DIALOG_PRINT(X) (X)

#define YYINPUT svc_UART_getc(&svc_uart2, portMAX_DELAY)

#if 0
// #define YYINPUT entree()
int entree()
{
    int car = USART_getc(aux_usart1, portMAX_DELAY);
    svc_UART_Write(aux_usart1, &car, 1, portMAX_DELAY);
    return car;
}
#endif

/* Maximum length for a token (word or number) */
#define MAX_TOKEN_LEN 20

/* Maximum tokens per command line.
 According to the grammar, the maximum is 3 or 4 tokens. */
#define MAX_TOKENS 4

/* Enumeration of token types */
typedef enum
{
    TOKEN_UNKNOWN = 0, /* Unrecognized token */
    TOKEN_AP,
    TOKEN_CALIBRATE,
    TOKEN_CONFIG,
    TOKEN_DISPLAY,
    TOKEN_EOL, /* End-of-line marker */
    TOKEN_GPS,
    TOKEN_HEADING,
    TOKEN_HWMS, /* High Water Marks, stack usage measure */
    TOKEN_IDLE,
    TOKEN_KD,
    TOKEN_KI,
    TOKEN_KP,
    TOKEN_MAG_VS_GYR,
    TOKEN_MEMS,
    TOKEN_MODE,
    TOKEN_MOTOR,
    TOKEN_MOTOR_CVT_ANGLE_TIME,
    TOKEN_MOTOR_HPF_COEFF,
    TOKEN_MOTOR_THRESHOLD,
    TOKEN_NUMBER, /* Represents a number token (with optional sign) */
    TOKEN_PORT,
    TOKEN_SET,
    TOKEN_STARBOARD,
    TOKEN_STATUS,
    TOKEN_TURN,
    TOKEN_WIND

} TokenType;

/* Structure to map keyword strings to token types.
 This lookup table is used to avoid multiple strcmp calls scattered in the
 code. */
typedef struct
{
    const char *keyword;
    TokenType type;
} TokenEntry;

/* Table of known keywords */
static const TokenEntry tokenTable[] =
        {
                { "calibrate", TOKEN_CALIBRATE },
                { "AP", TOKEN_AP },
                { "config", TOKEN_CONFIG },
                { "display", TOKEN_DISPLAY },
                { "GPS", TOKEN_GPS },
                { "heading", TOKEN_HEADING },
                { "hwms", TOKEN_HWMS},
                { "idle", TOKEN_IDLE },
                { "Kd", TOKEN_KD },
                { "Ki", TOKEN_KI },
                { "Kp", TOKEN_KP },
                { "mag_vs_gyr", TOKEN_MAG_VS_GYR },
                { "MEMS", TOKEN_MEMS },
                { "mode", TOKEN_MODE },
                { "motor", TOKEN_MOTOR },
                { "motor_angletime", TOKEN_MOTOR_CVT_ANGLE_TIME },
                { "motor_hpf_coeff", TOKEN_MOTOR_HPF_COEFF },
                { "motor_threshold", TOKEN_MOTOR_THRESHOLD },
                { "port", TOKEN_PORT },
                { "set", TOKEN_SET },
                { "starboard", TOKEN_STARBOARD },
                { "turn", TOKEN_TURN },
                { "wind", TOKEN_WIND },
                { NULL, TOKEN_UNKNOWN }
        };

QueueHandle_t msgQueueDialogIn;
QueueHandle_t msgQueueDialogOut;

int init_taskDialogIn()
{

    msgQueueDialogIn = xQueueCreate(10, sizeof(MsgDialog_t));
    msgQueueDialogOut = xQueueCreate(10, sizeof(MsgDialog_t));

    if ((msgQueueDialogIn == (QueueHandle_t) 0) ||
            (msgQueueDialogOut == (QueueHandle_t) 0))
    {
        return -1;
    }
    else
    {
        return 1;
    }
}

/*-----------------------------*/
/* Helper functions prototypes */
/*-----------------------------*/

int convert_float(const char *token, float *number)
{
    int i = 0;
    int sign = 1;
    float result = 0.0f;
    float fraction = 0.0f;
    float divisor = 10.0f;
    int has_digits = 0;

    if (!token || !number)
    {
        return 0;
    }

// Gérer le signe éventuel
    if (token[i] == '+' || token[i] == '-')
    {
        if (token[i] == '-')
        {
            sign = -1;
        }

        i++;
    }

// Partie entière
    while (token[i] != '\0' && rlib_isdigit((unsigned char) token[i]))
    {
        result = result * 10.0f + (token[i] - '0');
        i++;
        has_digits = 1;
    }

// Partie fractionnaire
    if (token[i] == '.')
    {
        i++;

        while (token[i] != '\0' && rlib_isdigit((unsigned char) token[i]))
        {
            fraction += (token[i] - '0') / divisor;
            divisor *= 10.0f;
            i++;
            has_digits = 1;
        }
    }

    if (!has_digits)
    {
        return 0; // Aucun chiffre trouvé
    }

    *number = sign * (result + fraction);
    return 1;
}

/**
 * convert_number:
 * Manually converts a string representing a signed number into an integer.
 * It processes the string character-by-character, using multiplications and
 * additions.
 *
 * Parameters:
 *   token - the string representing the number (with an optional '+' or '-'
 * sign)
 *
 * Returns:
 *   The integer value represented by the string.
 */
int convert_number(const char *token)
{
    int num = 0;
    int sign = 1;
    int i = 0;

    /* Check for an optional sign */
    if (token[i] == '+' || token[i] == '-')
    {
        if (token[i] == '-')
        {
            sign = -1;
        }

        i++;
    }

    /* Process each digit character until the end of the string */
    while (token[i] != '\0' && rlib_isdigit((unsigned char) token[i]))
    {
        /* Multiply current result by 10 and add the numeric value of the digit
         */
        num = num * 10 + (token[i] - '0');
        i++;
    }

    return sign * num;
}

/**
 * read_token:
 * Reads a token (a word or number) from standard input using USART_getc().
 * The token is stored in the provided buffer and is null-terminated.
 * This function stops reading when a space, tab, newline, or EOF is
 * encountered.
 *
 * Parameters:
 *   token - a char array where the token will be stored.
 *   max_len - maximum number of characters to store (including the terminating
 * null).
 *
 * Returns:
 *   The first character that terminated the token (could be '\n' or EOF),
 *   so that the caller knows whether the token ended because of end-of-line.
 */
int read_token(char token[], int max_len)
{
    int c;
    int pos = 0;

// Skip any leading spaces or tabs
    while ((c = YYINPUT) == ' ' || c == '\t')
        ; // do nothing

// If the first non-space char is newline or EOF, return it directly.
    if (c == '\n' || c == '\r')
    {
        token[0] = '\0';
        return c;
    }

// Read characters until a space, tab, newline, or EOF is found.
    do
    {
        if (pos < max_len - 1)
        {
            token[pos++] = (char) c;
        }

        c = YYINPUT;
    } while (c != ' ' && c != '\t' && c != '\n' && c != '\r');

    token[pos] = '\0';
    return c;
}

/**
 * lookup_token:
 * Compares a given token string to the known keywords table.
 * If a match is found, the corresponding TokenType is returned.
 * If the token represents a number, TOKEN_NUMBER is returned.
 * Otherwise, TOKEN_UNKNOWN is returned.
 *
 * Parameters:
 *   token - the token string to classify.
 *
 * Returns:
 *   The TokenType corresponding to the token.
 */
TokenType lookup_token(const char *token)
{
    int i;

    /* Check if the token is a number: optional sign and at least one digit.
     We assume that any token starting with a digit or a sign followed by a
     digit is a valid number token. */
    if (token[0] == '+' || token[0] == '-' ||
            rlib_isdigit((unsigned char) token[0]))
    {
        int pos = (token[0] == '+' || token[0] == '-') ? 1 : 0;

        if (token[pos] != '\0' && rlib_isdigit((unsigned char) token[pos]))
        {
            return TOKEN_NUMBER;
        }
    }

    /* Look up in the keyword table */
    for (i = 0; tokenTable[i].keyword != NULL; i++)
    {
        int j = 0;

        /* Compare the token and the keyword character by character,
         using only functions from ctype for case handling */
        while (tokenTable[i].keyword[j] != '\0' && token[j] != '\0')
        {
            if (rlib_tolower((unsigned char) token[j]) !=
                    rlib_tolower((unsigned char) tokenTable[i].keyword[j]))
                break;

            j++;
        }

        if (tokenTable[i].keyword[j] == '\0' && token[j] == '\0')
        {
            return tokenTable[i].type;
        }
    }

    return TOKEN_UNKNOWN;
}

/**
 * parse_command_line:
 * Parses a single command line (terminated by a newline) from standard input.
 * It tokenizes the line into at most MAX_TOKENS tokens and then processes the
 * command according to the grammar provided in the attached file.
 *
 * The function uses only local variables (no global variables, no dynamic
 * allocation) and relies on a table lookup for token classification.
 */
void parse_command_line(void)
{
    char tokens[MAX_TOKENS]
    [MAX_TOKEN_LEN]; /* table for tokens in the current command */
    TokenType tokenTypes[MAX_TOKENS]; /* corresponding token types */
    float numberValue; /* for storing converted numbers */
    int tokenCount = 0;
    int terminator;
    static char message[100];
    //char nbcar;
    MsgAutoPilot_t msgAutoPilot;
    MEMS_Msg_t msgMEMs;
    MsgMotor_t msgMotor;

    for (int i = 0; i < MAX_TOKENS; i++)
    {
        tokenTypes[i] = TOKEN_UNKNOWN; // Initialize token strings to empty
    }

    /* Read tokens until a newline is encountered.
     Each call to read_token returns the delimiter that terminated the token.
     */
    do
    {
        terminator = read_token(tokens[tokenCount], MAX_TOKEN_LEN);

        /* If the token string is empty, it means that the newline or EOF was
         encountered before any token was read; we skip processing in that
         case. */
        if (tokens[tokenCount][0] != '\0')
        {
            tokenTypes[tokenCount] = lookup_token(tokens[tokenCount]);
            tokenCount++;

            if (tokenCount >= MAX_TOKENS)
            {
                // We limit to MAX_TOKENS tokens per command
                break;
            }
        }
    } while (terminator != '\n' && terminator != '\r');

    /* Now process the command based on the tokens read.
     The grammar (as described in your file) specifies the following
     productions:

     1) "turn" PORT | STARBOARD NUMBER EOL          -> exec_turn(NUMBER)
     2) "mode" "idle" EOL          -> exec_idle()
     3) "mode" "heading" EOL       -> exec_heading()
     4) "mode" "heading" NUMBER EOL -> exec_heading_value(NUMBER)
     5) "move" "starboard" EOL     -> exec_move_stb()
     6) "move" "port" EOL          -> exec_move_port()

     We check the token types and the number of tokens to decide which command
     to execute.
     */

    switch (tokenTypes[0])
    {
    case TOKEN_TURN:
        if (((tokenTypes[1] == TOKEN_PORT) || tokenTypes[1] == TOKEN_STARBOARD)
                && (tokenTypes[2] == TOKEN_NUMBER))
        {
            int angle = convert_number(tokens[2]);
            msgAutoPilot.msgType = AP_MSG_TURN;
            msgAutoPilot.data.reqTurnAngle =
                    (tokenTypes[1] == TOKEN_PORT) ? -angle : angle;
            xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
        }
        else
        {
            return; // Syntax error
        }
        break;

        /* Mode idle | ( heading [number] ) */
        /* set mode idle or heading heading to steer optional */
    case TOKEN_MODE:
        switch (tokenTypes[1])
        {
        /* Mode idle */
        case TOKEN_IDLE:
            msgAutoPilot.msgType = AP_MSG_MODE_IDLE;
            xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
            break;

            /* mode heading, optional heading number */
        case TOKEN_HEADING:
            switch (tokenTypes[2])
            {
            case TOKEN_NUMBER:
                int heading = convert_number(tokens[2]);
                msgAutoPilot.msgType = AP_MSG_MODE_HEADING_DIR;
                msgAutoPilot.data.reqHeading = heading;
                xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
                break;

            case TOKEN_UNKNOWN:
                msgAutoPilot.msgType = AP_MSG_MODE_HEADING;
                xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
                break;

            default:
                return; // Syntax error
            }
            break;

        case TOKEN_GPS:
            break;

        case TOKEN_WIND:
            break;

        default:
            return; // Syntax error
        }

        /* calibration commands */
    case TOKEN_CALIBRATE:
        switch (tokenTypes[1])
        {
        /*calibrate MEMS */
        case TOKEN_MEMS:
            msgMEMs.msgType = MEMS_MSG_CALIBRATE;
            xQueueSend(msgQueueMEMs, &msgMEMs, 0);
            break;

        default:
            return; // Syntax error
        }

        /* Set parameters SET <param> <value> */
    case TOKEN_SET:
        if (tokenTypes[2] == TOKEN_NUMBER)
        {
            if (convert_float(tokens[2], &numberValue))
            {
                // Process the parameter setting based on tokenTypes[1]
                // For example, if tokenTypes[1] is TOKEN_PROPORTIONAL, set
                // the proportional coefficient This part depends on how you
                // want to structure your parameter setting commands
                switch (tokenTypes[1])
                {
                case TOKEN_KP:
                    msgAutoPilot.msgType = AP_MSG_PARAM;
                    msgAutoPilot.data.coefficient.param_number =
                            AP_PARAM_KP;
                    msgAutoPilot.data.coefficient.param_value = numberValue;
                    xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
                    break;
                case TOKEN_KI:
                    msgAutoPilot.msgType = AP_MSG_PARAM;
                    msgAutoPilot.data.coefficient.param_number =
                            AP_PARAM_KI;
                    msgAutoPilot.data.coefficient.param_value = numberValue;
                    xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
                    break;

                case TOKEN_KD:
                    msgAutoPilot.msgType = AP_MSG_PARAM;
                    msgAutoPilot.data.coefficient.param_number =
                            AP_PARAM_KD;
                    msgAutoPilot.data.coefficient.param_value = numberValue;
                    xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
                    break;

                case TOKEN_MOTOR_CVT_ANGLE_TIME:
                    msgMotor.msgType = MSG_MOTOR_SET_CVT_ANGLE_TIME;
                    msgMotor.data.cvtAngleTime = numberValue;
                    xQueueSend(msgQueueMotor, &msgMotor, 0);
                    break;

                case TOKEN_MAG_VS_GYR:
                    msgMEMs.msgType = MEMS_MSG_SET_MAG_VS_GYR;
                    msgMEMs.data.mag_vs_gyr = numberValue;
                    xQueueSend(msgQueueMEMs, &msgMEMs, 0);
                    break;

                case TOKEN_MOTOR_THRESHOLD:
                    msgMotor.msgType = MSG_MOTOR_SET_THRESHOLD;
                    msgMotor.data.threshold = numberValue;
                    xQueueSend(msgQueueMotor, &msgMotor, 0);
                    break;

                case TOKEN_MOTOR_HPF_COEFF:
                    msgMotor.msgType = MSG_MOTOR_SET_HPF_COEF;
                    msgMotor.data.hpf_coeff = numberValue;
                    xQueueSend(msgQueueMotor, &msgMotor, 0);
                    break;

                default:
                    return; // Syntax error: unrecognized parameter
                    break;
                }
            }
            else
            {
                return; // Syntax error in number format
            }
        }
        break;

        /* display configurations or statuses */
    case TOKEN_DISPLAY:
        switch (tokenTypes[1]) {
       
        case TOKEN_MEMS: /* display MEMS ... */

            switch (tokenTypes[2]) {

            case TOKEN_CONFIG: /* display MEMS config */

                msgMEMs.msgType = MEMS_MSG_DISPLAY_CONFIG;
                xQueueSend(msgQueueMEMs, &msgMEMs, 0);

                break; /* case TOKEN_CONFIG: */

            default:

                return; 

            }
            break;

        case TOKEN_HWMS: /* High Water MarkS : display stack usage of each task */

            /* task list is in taskHandles[] */
            for (int i = 0 ; i<tasksNumber ; i++)
            {
                /* each task has its own stack.*/
                /* High water mark tels the usage of the task's stack. */
                /* get task name and high water mark of task. */
                UBaseType_t hwm = uxTaskGetStackHighWaterMark(tasksHandles[i]);
                char* taskName = pcTaskGetName(tasksHandles[i]);
                int nbcar = snprintf(message, sizeof(message), "TASK HWMS %d  %16s : %u bytes\n", i, taskName, hwm * sizeof(StackType_t));
                svc_UART_Write(&svc_uart2, message, nbcar, 0U);
            }

            break; /* TOKEN_HWMS */
           
        case TOKEN_MOTOR: /* display motor ... */
            switch (tokenTypes[2])
            {
           
            case TOKEN_CONFIG: /* display motor config */

                snprintf (message, sizeof(message) - 1, "Motor config ?\n");
                svc_UART_Write(&svc_uart2, message, strlen(message), 0U);

                msgMotor.msgType = MSG_MOTOR_DISPLAY_CONFIG;
                xQueueSend(msgQueueMotor, &msgMotor, 0);
                break;

            default:
                return;
            }
            break;

            /* display AP ...  AP for autopilot */
        case TOKEN_AP:
            switch (tokenTypes[2])
            {

            case TOKEN_CONFIG: /* display AP config */

                msgAutoPilot.msgType = AP_MSG_DISPLAY_CONFIG;
                xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);

                break;

            default:
                return;
            }
            break;

        default:
            return;
        }

    default:
        return; // Syntax error: unrecognized command
    }

    return;
}

void taskDialogIn(void *args __attribute__((unused)))
{

    for (;;)
    {
        parse_command_line();
    }
}

#if 0
void __attribute__((noreturn)) taskDialogOut(void *args __attribute__((unused)))
{
    MsgDialog_t msg;
    int nb = 0;
    char message[50];
    int nbcar;

    for(;;)
    {
        nb++;

        if(xQueueReceive(msgQueueDialogOut, &msg, portMAX_DELAY) == pdPASS)
        {
            nbcar = snprintf(message, sizeof(message), "Message type %d\r\n", (int)msg.msgType);
            // Process the message
            // For now, just print the message type
            svc_UART_Write(&svc_uart2, message, nbcar, 0U);
        }
    }
}
#endif
