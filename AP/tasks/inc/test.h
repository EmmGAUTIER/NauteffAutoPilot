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

#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "queue.h"

typedef enum {
    TEST_MSG_TICK,
    TEST_MSG_START,
    TEST_MSG_STOP,
} Test_msg_type_t;

typedef struct {
    Test_msg_type_t msgType;
    union {
        int test_number; /* number of test */
    } data;
} Test_msg_t;

void Test_task_init();
void Test_task(void *param);
void Test_msg_start(int);
void Test_msg_stop(void);

// Type pour les pointeurs de fonction
typedef void (*func_ptr_t)(void);
typedef void (*func_ptr_int_t)(int);
typedef void (*func_ptr_float_t)(float);

// Type pour les arguments
typedef enum { ARG_NONE, ARG_INT, ARG_FLOAT } ArgType;

// Structure pour un argument
typedef struct {
    ArgType type;
    union {
        int i;
        float f;
    } value;
} Argument;

// Structure pour un appel de fonction
typedef struct {
    const char *name;       /* Function name or description */
    int delay;              /* Delay before calling the function, in milliseconds */
    void *func_ptr;         /* function pointer (cast to void*) */
    ArgType arg_type;       /* Argument type (ARG_NONE, ARG_INT, ARG_FLOAT) */
    Argument arg;           /* Argument if any */
} Test_call_t;
