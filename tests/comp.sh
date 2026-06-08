
CC=gcc
CFLAGS="-g -lm"
COMPUTE_DIR="../AP/compute/src/"
FILES="main.c      \
       aux_fcts.c   \
      test_geom.c \
      ${COMPUTE_DIR}/geom.c"

C_INCLUDES=" \
-I../Core/Inc \
-I../Drivers/STM32L4xx_HAL_Driver/Inc \
-I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy \
-I../Middlewares/Third_Party/FreeRTOS/Source/include \
-I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
-I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-I../Drivers/CMSIS/Device/ST/STM32L4xx/Include \
-I../Drivers/CMSIS/Include \
-I../AP/tasks/inc \
-I../AP/aux/inc \
-I../AP/compute/inc"

set -x
${CC} ${CFLAGS} $FILES $C_INCLUDES -o tst_ap -lm
