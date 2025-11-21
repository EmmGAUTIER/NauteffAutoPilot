#/usr/bin/bash
#
# Flash the NauteffAutoPilot.
#
#  command sent to openocd to reset and halt the STM32
#

if [ -n NAUTEFF_AP_PATH ]
then
    NAUTEFF_AP_PATH="."
fi

cppcheck --enable=all --inconclusive --force --max-ctu-depth=10 \
	-I${NAUTEFF_AP_PATH}/AP/tasks/inc \
	-I${NAUTEFF_AP_PATH}/AP/geom/inc \
	-I${NAUTEFF_AP_PATH}/AP/compute/inc \
       	${NAUTEFF_AP_PATH}/AP
