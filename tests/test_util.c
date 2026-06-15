/*
 * test_util.c
 *
 *  Created on: 12 juin 2026
 *      Author: manu
 */

#include <stdio.h>
#include <assert.h>

#include "util.h"

int test_util()
{
    int err_counter = 0;
    unsigned b1, b2; /* unsigned integers as bit fields to modify */
    unsigned ms, mr; /* mask bits */
    unsigned br;     /* result*/

    printf ("\nTest de la fonction setbits()\n");
    b1 = 0x5555;
    b2 = b1;
    ms = 0x0F0F;
    setbits(&b2, ms);
    assert (b2 == (b1|ms));

    printf ("\nTest de la fonction setresetbits()\n");
    b1 = 0x5555;
    b2 = b1;
    ms = 0x0F0F;
    mr = 0xF000;
    setresetbits (&b2, ms, mr);
    assert (b2 == ((b1&mr)|ms));

    printf ("\nTest de la fonction clearbits()\n");
    b1 = 0x5555;
    b2 = b1;
    mr = 0x0F00;
    clearbits (&b2, ms);
    assert (b2 == (b1|ms));

    return err_counter;
}


