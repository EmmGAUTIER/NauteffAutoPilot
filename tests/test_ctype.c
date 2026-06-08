#include <stdio.h>
#include <ctype.h>
#include "rlib.h"

#define TEST_RANGE_START -1
#define TEST_RANGE_END 256

static int test_function(const char* fname, int (*std_func)(int), int (*rlib_func)(int)) {
    int errors = 0;
    for (int i = TEST_RANGE_START; i <= TEST_RANGE_END; i++) {
        int std_result = std_func(i) != 0;
        int rlib_result = rlib_func(i) != 0;
        
        if (std_result != rlib_result) {
            printf("Mismatch for %s with input %d (0x%02x): std=%d, rlib=%d\n",
                   fname, i, i, std_result, rlib_result);
            errors++;
        }
    }
    return errors;
}

int main(void) {
    int total_errors = 0;
    
    struct {
        const char* name;
        int (*std_func)(int);
        int (*rlib_func)(int);
    } tests[] = {
        {"isalnum", isalnum, rlib_isalnum},
        {"isalpha", isalpha, rlib_isalpha},
        {"isascii", isascii, rlib_isascii},
        {"isblank", isblank, rlib_isblank},
        {"iscntrl", iscntrl, rlib_iscntrl},
        {"isdigit", isdigit, rlib_isdigit},
        {"isgraph", isgraph, rlib_isgraph},
        {"islower", islower, rlib_islower},
        {"isprint", isprint, rlib_isprint},
        {"ispunct", ispunct, rlib_ispunct},
        {"isspace", isspace, rlib_isspace},
        {"isupper", isupper, rlib_isupper},
        {"isxdigit", isxdigit, rlib_isxdigit},
        {NULL, NULL, NULL}
    };

    printf("Starting ctype functions comparison tests...\n\n");

    for (int i = 0; tests[i].name != NULL; i++) {
        printf("Testing %s()...\n", tests[i].name);
        int errors = test_function(tests[i].name, tests[i].std_func, tests[i].rlib_func);
        if (errors == 0) {
            printf("%s: OK\n", tests[i].name);
        } else {
            printf("%s: %d differences found\n", tests[i].name, errors);
        }
        total_errors += errors;
        printf("\n");
    }

    printf("Test complete. Total differences found: %d\n", total_errors);
    return total_errors != 0;
}
