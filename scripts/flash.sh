#/usr/bin/bash
#
# Flash the NauteffAutoPilot.
#
# Flash is done with openocd. openocd may run on a remote machine.
# The commands are sent to openocd by nc (wich stands for netcat)
# On the remote machine openocd has to run as a service.
# 
#

if [ -n "$NAUTEFF_AP_DIR_REMOTE" ]
then
    echo "Copie sur machine distante :"
    cp build/NauteffAutoPilot.elf $NAUTEFF_AP_DIR_REMOTE/build/NauteffAutoPilot.elf 
fi

if [ $NAUTEFF_AP_HOST_OPENOCD ]
then
    export HOST_OPENOCD=${NAUTEFF_AP_HOST_OPENOCD}
else
    export HOST_OPENOCD="localhost"
fi

#echo "Dir. distant : " ${NAUTEFF_AP_DIR_REMOTE}
#echo "Hôte openocd : " ${HOST_OPENOCD}


    (
    echo "reset halt ";\
    echo "flash write_image erase build/NauteffAutoPilot.elf";\
    echo "verify_image build/NauteffAutoPilot.elf";\
    echo "reset run ";\
    echo "exit ";\
    ) | nc ${HOST_OPENOCD} 4444
