#!/bin/bash

# DIALOG=${DIALOG=dialog}
DIALOG=dialog

SCRIPT_DIR=$(dirname "$(realpath "$0")")
export NAUTEFFAUTOPILOT_PATH=`dirname $SCRIPT_DIR`
cd  $NAUTEFFAUTOPILOT_PATH
export TARGET_DIR=$NAUTEFFAUTOPILOT_PATH/build

main_menu() {
    while true; do
        CHOICE=$($DIALOG --clear --backtitle "Menu Principal" \
            --title "Menu" \
            --menu "Sélectionnez une option :" 15 50 5 \
            1 "Compilation" \
            2 "Compilation totale" \
            3 "Carte mémoire" \
            4 "Débogage" \
            5 "Documentation Auto doxygen" \
            6 "Documentation latex" \
            t "tests" \
            z "Fin" \
            2>&1 >/dev/tty)

        case $CHOICE in
            1)  echo "Compilation :"
                cd ${NAUTEFFAUTOPILOT_PATH}
                cmake --build build
                read -p "Appuyez sur <Return> pour continuer" rep
                ;;

            2)  echo "Compilation Totale"
                cd ${NAUTEFFAUTOPILOT_PATH}
                cmake --build build --target clean
                cmake --build build
                read -p "Appuyez sur <Return> pour continuer" rep
                ;;

            3)  echo "Carte mémoire"
                ${NAUTEFFAUTOPILOT_PATH}/scripts/carte.py ${NAUTEFFAUTOPILOT_PATH}/build/NauteffAutoPilot.elf | less
                read -p "Appuyez sur <Return> pour continuer" rep
                ;;

            4) ${NAUTEFFAUTOPILOT_PATH}/scripts/debug.sh
                ;;

            5)  echo "Génération de documentation (doxygen)"
                ${NAUTEFFAUTOPILOT_PATH}/scripts/mk_doxy.sh
                read -p "Appuyez sur <Return> pour continuer" rep
                ;;

            6)  echo "Génération de documentation (latex)"
                ${NAUTEFFAUTOPILOT_PATH}/scripts/mk_latex.sh
                read -p "Appuyez sur <Return> pour continuer" rep
                ;;

            t) test_menu
		read -p "Appuyez sur une touche pour continuer" rep
                ;;

            z) clear
               exit 0
                ;;

            *) break ;;
        esac
    done
}

test_menu() {
    while true; do
        CHOICE=$($DIALOG --clear --backtitle "Menu Tests" \
            --title "Tests" \
            --menu "Sélectionnez un test :" 12 40 3 \
            1 "NMEA" \
            2 "Conversions" \
            3 "geométrie" \
            4 "Retour" \
            2>&1 >/dev/tty)

        case $CHOICE in
            1)  echo "Test NMEA 0183"
                ${NAUTEFFAUTOPILOT_PATH}/scripts/test_nmea.sh
                read -p "Fin de test. Appuyez sur <Return> pour continuer" rep
                ;;

            2)  echo "Test conversions"
                ${NAUTEFFAUTOPILOT_PATH}/scripts/test_any.sh cvt
                read -p "Fin de test. Appuyez sur <Return> pour continuer" rep
                ;;

            3)  echo "Test Géométrie"
                ${NAUTEFFAUTOPILOT_PATH}/scripts/test_any.sh geom
                read -p "Fin de test. Appuyez sur <Return> pour continuer" rep
                ;;
            4) break ;;
            *) break ;;
        esac
    done
}

main_menu
clear

