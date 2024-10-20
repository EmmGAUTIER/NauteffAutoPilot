#!/bin/sh
##########################################
# Nauteff Autopilot                      #
# compilation Chargement débuggage       #
##########################################

TARGET=bin/NauteffAutoPilot.elf

#----- Compilation ------

echo "--> make "
make || exit

#----- Montage disque distant -----

# Montage du disque distant
# La carte de dev est sur un Raspberrry pi.
# Celui-ci a des resources réduites et est embarqué à bord d'un bateau.
echo "--> mount"
mount_rpi=`mount |grep rpi`
if  [  "$mount_rpi" = "" ]
then
    echo "----- Montage du dsique distant -----"
    sshfs emmanuel@192.168.0.102:workspace ~/workspace/rpi/
fi

#----- Copie du fichier -----
echo "--> Copie"
cp ${TARGET} ${HOME}/workspace/rpi/NauteffAutoPilot/bin || exit 1

#----- Démarrage de openocd -----

echo "--> openocd, start"
# nc -z permet de tester le port et donc de vérifier
# si le serveur openocd est actif.
nc -z 192.168.0.102 4444
if [ $? !=  0 ]
then
    ssh emmanuel@192.168.0.102 " ~/workspace/NauteffAutoPilot/srv &"
    sleep 3 #wait for starting of openocd before sending requests
fi

#----- Chargement du code dans le STM32 -----

echo "--> Chargement"
(
echo "reset halt ";\
echo "flash write_image erase ./bin/NauteffAutoPilot.elf";\
echo "verify_image ./bin/NauteffAutoPilot.elf";\
echo "reset run ";\
echo "exit ";\
) | nc 192.168.0.102 4444

#----- Déboggage -----

echo "--> ddd gdb"

(echo "reset halt" ; echo "exit"  ) | telnet 192.168.0.102 4444
#gdb-multiarch --eval-command="target remote localhost:3333" Debug/NauteffAutoPilot.elf
set -x
ddd --dbx --debugger "gdb-multiarch --eval-command='target remote 192.168.0.102:3333' ${TARGET}"

exit 0

