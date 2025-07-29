#/usr/bin/bash
#
# Flash the NauteffAutoPilot.
#
# Flash is done with openocd. openocd may run on a remote machine.
# The commands are sent to openocd by nc (wich stands for netcat)
# On the remote machine openocd has to run as a service.
# 
#

echo "Copie :"
pwd
cp NauteffAutoPilot.elf $NAUTEFF_AP_DIR_REMOTE/build/NauteffAutoPilot.elf 


    (
    echo "reset halt ";\
    echo "flash write_image erase build/NauteffAutoPilot.elf";\
    echo "verify_image build/NauteffAutoPilot.elf";\
    echo "reset run ";\
    echo "exit ";\
    ) | nc ${NAUTEFF_OPENOCD_HOST} 4444
