#/usr/bin/bash
#
# Flash the NauteffAutoPilot.
#
#  command sent to openocd to reset and halt the STM32
#
    (
    echo "reset halt ";\
    echo "exit ";\
    ) | nc ${NAUTEFF_OPENOCD_HOST} 4444
