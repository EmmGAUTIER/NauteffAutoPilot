#/usr/bin/bash
#
# Flash the NauteffAutoPilot.
#
#  command sent to openocd to reset and halt the STM32
#

cppcheck --enable=all --inconclusive --force --max-ctu-depth=10 -j 4 ../AP
