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

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "stream_buffer.h"

#include "util.h"
#include "printf.h"
#include "rlib.h"

#include <stm32l452xx.h>
#include <stm32l4xx_ll_gpio.h>
#include "service.h"
#include "apdialog.h"
#include "autopilot.h"
#include "motor.h"
#include "mems.h"

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
#define MAX_TOKEN_LEN 16

/* Maximum tokens per command line.
   According to the grammar, the maximum is 3 or 4 tokens. */
#define MAX_TOKENS 4

/* Enumeration of token types */
typedef enum
{
    TOKEN_UNKNOWN = 0, /* Unrecognized token */
    TOKEN_TURN,
    TOKEN_MODE,
    TOKEN_IDLE,
    TOKEN_HEADING,
    TOKEN_MOVE,
    TOKEN_STARBOARD,
    TOKEN_PORT,
    TOKEN_NUMBER, /* Represents a number token (with optional sign) */
    TOKEN_COEFFICIENT,
    TOKEN_INTEGRAL,
    TOKEN_DERIVATIVE,
    TOKEN_PROPORTIONAL,
    TOKEN_AHRS,
    TOKEN_CALIBRATE,
    TOKEN_DISPLAY,
    TOKEN_CONFIG,
    TOKEN_MEMS,
    TOKEN_MOTOR,
    TOKEN_AP,
    TOKEN_EOL /* End-of-line marker */
} TokenType;

/* Structure to map keyword strings to token types.
   This lookup table is used to avoid multiple strcmp calls scattered in the code. */
typedef struct
{
    const char *keyword;
    TokenType type;
} TokenEntry;

/* Table of known keywords */
static const TokenEntry tokenTable[] = {
    {"turn", TOKEN_TURN},
    {"mode", TOKEN_MODE},
    {"idle", TOKEN_IDLE},
    {"heading", TOKEN_HEADING},
    {"move", TOKEN_MOVE},
    {"starboard", TOKEN_STARBOARD},
    {"port", TOKEN_PORT},
    {"coefficient", TOKEN_COEFFICIENT},
    {"integral", TOKEN_INTEGRAL},
    {"derivative", TOKEN_DERIVATIVE},
    {"proportional", TOKEN_PROPORTIONAL},
    {"AHRS", TOKEN_AHRS},
    {"calibrate", TOKEN_CALIBRATE},
    {"display", TOKEN_DISPLAY},
    {"config", TOKEN_CONFIG},
    {"MEMS", TOKEN_MEMS},
    {"motor", TOKEN_MOTOR},
    {"AP", TOKEN_AP},
    {NULL, TOKEN_UNKNOWN}};

QueueHandle_t msgQueueDialogIn;
QueueHandle_t msgQueueDialogOut;

int init_taskDialogIn()
{

    msgQueueDialogIn = xQueueCreate(10, sizeof(MsgDialog_t));
    msgQueueDialogOut = xQueueCreate(10, sizeof(MsgDialog_t));

    if ((msgQueueDialogIn == (QueueHandle_t)0) || (msgQueueDialogOut == (QueueHandle_t)0))
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
    while (token[i] != '\0' && rlib_isdigit((unsigned char)token[i]))
    {
        result = result * 10.0f + (token[i] - '0');
        i++;
        has_digits = 1;
    }

    // Partie fractionnaire
    if (token[i] == '.')
    {
        i++;
        while (token[i] != '\0' && rlib_isdigit((unsigned char)token[i]))
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
 * It processes the string character-by-character, using multiplications and additions.
 *
 * Parameters:
 *   token - the string representing the number (with an optional '+' or '-' sign)
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
    while (token[i] != '\0' && rlib_isdigit((unsigned char)token[i]))
    {
        /* Multiply current result by 10 and add the numeric value of the digit */
        num = num * 10 + (token[i] - '0');
        i++;
    }
    return sign * num;
}

/**
 * read_token:
 * Reads a token (a word or number) from standard input using USART_getc().
 * The token is stored in the provided buffer and is null-terminated.
 * This function stops reading when a space, tab, newline, or EOF is encountered.
 *
 * Parameters:
 *   token - a char array where the token will be stored.
 *   max_len - maximum number of characters to store (including the terminating null).
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
            token[pos++] = (char)c;
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
       We assume that any token starting with a digit or a sign followed by a digit
       is a valid number token. */
    if (token[0] == '+' || token[0] == '-' || rlib_isdigit((unsigned char)token[0]))
    {
        int pos = (token[0] == '+' || token[0] == '-') ? 1 : 0;
        if (token[pos] != '\0' && rlib_isdigit((unsigned char)token[pos]))
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
            if (rlib_tolower((unsigned char)token[j]) != rlib_tolower((unsigned char)tokenTable[i].keyword[j]))
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
 * It tokenizes the line into at most MAX_TOKENS tokens and then processes the command
 * according to the grammar provided in the attached file.
 *
 * The function uses only local variables (no global variables, no dynamic allocation)
 * and relies on a table lookup for token classification.
 */
void parse_command_line(void)
{
    char tokens[MAX_TOKENS][MAX_TOKEN_LEN]; // table for tokens in the current command
    TokenType tokenTypes[MAX_TOKENS];       // corresponding token types
    int tokenCount = 0;
    int terminator;
    static char message[100];
    char nbcar;
    MsgAutoPilot_t msgAutoPilot;
    MEMS_Msg_t msgMEMs;
    MsgMotor_t msgMotor;

    for (int i = 0; i < MAX_TOKENS; i++)
    {
        tokenTypes[i] = TOKEN_UNKNOWN; // Initialize token strings to empty
    }

    /* Read tokens until a newline is encountered.
       Each call to read_token returns the delimiter that terminated the token. */
    do
    {
        terminator = read_token(tokens[tokenCount], MAX_TOKEN_LEN);
        /* If the token string is empty, it means that the newline or EOF was encountered
           before any token was read; we skip processing in that case. */
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
       The grammar (as described in your file) specifies the following productions:

       1) "turn" PORT | STARBOARD NUMBER EOL          -> exec_turn(NUMBER)
       2) "mode" "idle" EOL          -> exec_idle()
       3) "mode" "heading" EOL       -> exec_heading()
       4) "mode" "heading" NUMBER EOL -> exec_heading_value(NUMBER)
       5) "move" "starboard" EOL     -> exec_move_stb()
       6) "move" "port" EOL          -> exec_move_port()

       We check the token types and the number of tokens to decide which command to execute.
    */

    /* empty line */
    if (tokenCount == 0)
        return; // Empty line, nothing to process.

    /* turn port NNN */
    if (tokenCount == 3 && tokenTypes[0] == TOKEN_TURN && tokenTypes[1] == TOKEN_PORT && tokenTypes[2] == TOKEN_NUMBER)
    {
        int angle = convert_number(tokens[2]);
        DBG_DIALOG_PRINT((
            nbcar = snprintf(message, sizeof(message), "DIALOG Turn port %d\r\n", angle),
            svc_UART_Write(&svc_uart2, message, nbcar, 0U)));

        msgAutoPilot.msgType = AP_MSG_TURN;
        msgAutoPilot.data.reqTurnAngle = -angle;
        xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
    }
    /* turn starboard NNN */
    else if (tokenCount == 3 && tokenTypes[0] == TOKEN_TURN && tokenTypes[1] == TOKEN_STARBOARD && tokenTypes[2] == TOKEN_NUMBER)
    {
        int angle = convert_number(tokens[2]);
        DBG_DIALOG_PRINT((
            nbcar = snprintf(message, sizeof(message), "DIALOG Turn starboard %d\r\n", angle),
            svc_UART_Write(&svc_uart2, message, nbcar, 0U)));

        msgAutoPilot.msgType = AP_MSG_TURN;
        msgAutoPilot.data.reqTurnAngle = angle;
        xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
    }
    else if (tokenCount == 2 && tokenTypes[0] == TOKEN_MODE && tokenTypes[1] == TOKEN_IDLE)
    {
        /* Command: mode idle */
        DBG_DIALOG_PRINT((
            svc_UART_Write(&svc_uart2, "DIALOG mode idle\r\n", 11, 0U)));

        msgAutoPilot.msgType = AP_MSG_MODE_IDLE;
        xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
    }
    else if (tokenCount == 2 && tokenTypes[0] == TOKEN_MODE && tokenTypes[1] == TOKEN_HEADING)
    {
        /* Command: mode heading */

        DBG_DIALOG_PRINT((
            svc_UART_Write(&svc_uart2, "DIALOG mode heading\n", 14, 0U)));

        msgAutoPilot.msgType = AP_MSG_MODE_HEADING;
        xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
    }
    else if (tokenCount == 3 && tokenTypes[0] == TOKEN_MODE && tokenTypes[1] == TOKEN_HEADING && tokenTypes[2] == TOKEN_NUMBER)
    {
        /* Command: mode heading NUMBER */
        int heading = convert_number(tokens[2]);
        DBG_DIALOG_PRINT((
            nbcar = snprintf(message, sizeof(message), "DIALOG Mode heading %d\n", heading),
            svc_UART_Write(&svc_uart2, message, nbcar, 0U)));

        msgAutoPilot.msgType = AP_MSG_MODE_HEADING_DIR;
        msgAutoPilot.data.reqHeading = heading;
        xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
    }
    else if (tokenCount == 2 && tokenTypes[0] == TOKEN_CALIBRATE && tokenTypes[1] == TOKEN_AHRS)
    {
        /* Command: calibrate AHRS */
        DBG_DIALOG_PRINT((
            svc_UART_Write(&svc_uart2, "DIALOG calibrate AHRS\n", 22, 0U)));

        msgAutoPilot.msgType = AP_MSG_CALIBRATE_MEMS;
        xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
    }
    else if (tokenCount == 2 && tokenTypes[0] == TOKEN_MOVE && tokenTypes[1] == TOKEN_STARBOARD)
    {
        /* Command: move starboard */
        // svc_UART_Write(&svc_uart2, "DIALOG move starboard\n", 17, 0U);
    }
    else if (tokenCount == 2 && tokenTypes[0] == TOKEN_MOVE && tokenTypes[1] == TOKEN_PORT)
    {
        /* Command: move port */
        // svc_UART_Write(&svc_uart2, "DIALOG move port\n", 12, 0U);
    }
    else if (tokenCount == 3 && tokenTypes[0] == TOKEN_COEFFICIENT)
    {
        float coeff = 2.71F;
        int res;

        if (tokenTypes[1] == TOKEN_PROPORTIONAL && tokenTypes[2] == TOKEN_NUMBER)
        {
            res = convert_float(tokens[2], &coeff);
            (void)res;
            DBG_DIALOG_PRINT((
                nbcar = snprintf(message, sizeof(message), "DIALOG Coefficient proportional %f\n", coeff),
                svc_UART_Write(&svc_uart2, message, nbcar, 0U)));

            msgAutoPilot.msgType = AP_MSG_PARAM;
            msgAutoPilot.data.coefficient.param_number = AP_PARAM_PROPORTIONNAL;
            msgAutoPilot.data.coefficient.param_value = coeff;
            res = xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
        }
        else if (tokenTypes[1] == TOKEN_INTEGRAL && tokenTypes[2] == TOKEN_NUMBER)
        {
            res = convert_float(tokens[2], &coeff);
            (void)res;
            msgAutoPilot.msgType = AP_MSG_PARAM;
            msgAutoPilot.data.coefficient.param_number = AP_PARAM_INTEGRAL;
            msgAutoPilot.data.coefficient.param_value = coeff;
            xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
            DBG_DIALOG_PRINT((
                nbcar = snprintf(message, sizeof(message), "DIALOG Coefficient integral %f\n", coeff),
                svc_UART_Write(&svc_uart2, message, nbcar, 0U)));
        }
        else if (tokenTypes[1] == TOKEN_DERIVATIVE && tokenTypes[2] == TOKEN_NUMBER)
        {
            res = convert_float(tokens[2], &coeff);
            (void)res;
            msgAutoPilot.msgType = AP_MSG_PARAM;
            msgAutoPilot.data.coefficient.param_number = AP_PARAM_DERIVATIVE;
            msgAutoPilot.data.coefficient.param_value = coeff;
            xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
            DBG_DIALOG_PRINT((
                nbcar = snprintf(message, sizeof(message), "DIALOG Coefficient derivative %f\n", coeff),
                svc_UART_Write(&svc_uart2, message, nbcar, 0U)));
        }
    }
    else if (tokenCount == 3 && tokenTypes[0] == TOKEN_DISPLAY && tokenTypes[1] == TOKEN_CONFIG)
    {
        // Command: display
        switch (tokenTypes[2])
        {
        case TOKEN_MEMS:
            DBG_DIALOG_PRINT((svc_UART_Write(&svc_uart2, "DIALOG display config MEMS\n", 27, 0U)));
            msgMEMs.msgType = MEMS_MSG_DISPLAY_CONFIG;
            xQueueSend(msgQueueMEMs, &msgMEMs, 0);
            break;

        case TOKEN_MOTOR:
            DBG_DIALOG_PRINT((svc_UART_Write(&svc_uart2, "DIALOG display config MOTOR\n", 28, 0U)));
            msgMotor.msgType = MSG_MOTOR_DISPLAY_CONFIG;
            xQueueSend(msgQueueMotor, &msgMotor, 0);
            break;

        case TOKEN_AP:
            DBG_DIALOG_PRINT((svc_UART_Write(&svc_uart2, "DIALOG display config AP\n", 25, 0U)));
            msgAutoPilot.msgType = AP_MSG_DISPLAY_CONFIG;
            xQueueSend(msgQueueAutoPilot, &msgAutoPilot, 0);
            break;

        default:
            break;
        }
    }

    else
    {
        DBG_DIALOG_PRINT((
            nbcar = snprintf(message, sizeof(message), "DIALOG Syntax error ! %d tokens\r\n", tokenCount),
            svc_UART_Write(&svc_uart2, message, nbcar, 0U)));
    }
    return;
}

// void __attribute__((noreturn)) task_Dialog(void *args __attribute__((unused)))
void taskDialogIn(void *args __attribute__((unused)))
{
    //int nb = 0; // pour mise au point

    for (;;)
    {
        // nb++;
        parse_command_line();
        // vTaskDelay(pdMS_TO_TICKS(100));
    }
}

#if 0
void __attribute__((noreturn)) taskDialogOut(void *args __attribute__((unused)))
{
    MsgDialog_t msg;
    int nb = 0;
    char message[50];
    int nbcar;

    for (;;)
    {
        nb++;
        if (xQueueReceive(msgQueueDialogOut, &msg, portMAX_DELAY) == pdPASS)
        {
            nbcar = snprintf(message, sizeof(message), "Message type %d\r\n", (int)msg.msgType);
            // Process the message
            // For now, just print the message type
            svc_UART_Write(&svc_uart2, message, nbcar, 0U);
        }
    }
}
#endif