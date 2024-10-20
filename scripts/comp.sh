#!/bin/sh
##########################################
# Nauteff Autopilot compilation Commands #
##########################################

CC="arm-none-eabi-gcc"

#OPTDEB=-Os
OPTDEB=-g3

TARGET_DIR="bin"
LKR_SCRIPT="NauteffAutoPilot.ld"
TARGET_ELF=${TARGET_DIR}"NauteffAutoPilot.elf"

CFLAGS=""

# Options pour les avertissement
CFLAGS=${CFLAGS}" -Wall"
CFLAGS=${CFLAGS}" -Wextra"

# Option nécessaire depuis une nouvelle version de GCC
# TODO Rechercher la cause
#CFLAGS=${CFLAGS}" -Wno-clobbered"
CFLAGS=${CFLAGS}" -Wno-attributes"

# Options Débogage ou optimisation
CFLAGS=${CFLAGS}" "${OPTDEB}

# Option pour préciser la cible et le FPU
CFLAGS=${CFLAGS}" -fno-math-errno"
CFLAGS=${CFLAGS}" -mcpu=cortex-m4"
CFLAGS=${CFLAGS}" -mthumb"
CFLAGS=${CFLAGS}" -mfloat-abi=hard"
CFLAGS=${CFLAGS}" -fsingle-precision-constant"
CFLAGS=${CFLAGS}" -mfpu=fpv4-sp-d16"
CFLAGS=${CFLAGS}" -specs=nosys.specs"
CFLAGS=${CFLAGS}" -nostartfiles "

# définition de la version du C
CFLAGS=${CFLAGS}" -std=c11"

# Chemin vers les fichirs d'entête
CFLAGS=${CFLAGS}" -Inauteff/inc"
CFLAGS=${CFLAGS}" -Isys/inc"
CFLAGS=${CFLAGS}" -Ifreertos/inc"


# Librairies
CFLAGS=${CFLAGS}" -lm -lg -lc"

# definition mémoire pour éditeur de liens
CFLAGS=${CFLAGS}" -T${LKR_SCRIPT}"

# Sortie de la carte mémoire
CFLAGS=${CFLAGS}" -Xlinker -Map -Xlinker "${TARGET_DIR}"NauteffAutoPilot.map"

echo "CFLAGS : "  $CFLAGS
#exit

SRC_FILES=""
SRC_FILES=${SRC_FILES}" sys/src/*.c"
SRC_FILES=${SRC_FILES}" freertos/src/*.c"
SRC_FILES=${SRC_FILES}" nauteff/src/*.c"


rm ${TARGET_DIR}/obj/*.o

for file in $SRC_FILES
do
    echo "Compiling " $file "   --> " ${TARGET_DIR}/obj/`basename $file ".c"`.o
    $CC -c $file -o ${TARGET_DIR}/obj/`basename $file ".c"`.o  ${CFLAGS}  || exit 1
done


echo linking

$CC ${TARGET_DIR}/obj/*.o -o./${TARGET_DIR}/NauteffAutoPilot.elf ${CFLAGS}  || exit 1

echo "----- Taille -----"
size ./${TARGET_DIR}/NauteffAutoPilot.elf


mount_rpi=`mount |grep rpi`
if  [  "$mount_rpi" = "" ]
then
    echo "----- Montage du dsique distant -----"
    sshfs emmanuel@192.168.0.102:workspace ~/workspace/rpi/
fi

echo "    ----- Copie -----"
cp ./${TARGET_DIR}/NauteffAutoPilot.elf ${HOME}/workspace/rpi/NauteffAutoPilot/bin || exit 1

echo "    ----- Chargement -----"
# nc -z permet de tester le port et donc de vérifier
# si le serveur openocd est actif.
nc -z 192.168.0.102 4444
if [ $? !=  0 ]
then
    ssh emmanuel@192.168.0.102 " ~/workspace/NauteffAutoPilot/srv &"
    sleep 3 /*wait for starting of openocd before sending requests */
fi
    (
    echo "reset halt ";\
    echo "flash write_image erase ./bin/NauteffAutoPilot.elf";\
    echo "verify_image ./bin/NauteffAutoPilot.elf";\
    echo "reset run ";\
    echo "exit ";\
    ) | nc 192.168.0.102 4444

exit 0

