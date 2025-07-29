#!/bin/sh
set -v
(echo "reset halt" ; echo "exit"  ) | telnet 192.168.0.102 4444
#gdb-multiarch --eval-command="target remote 192.168.0.102:3333" Debug/NauteffAutoPilot.elf
ddd --dbx --debugger "gdb-multiarch --eval-command='target remote 192.168.0.102:3333' build/NauteffAutoPilot.elf"
