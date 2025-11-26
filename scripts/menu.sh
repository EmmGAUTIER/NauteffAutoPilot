#!/bin/bash

##############################################################################
# Main menu for scripts
##############################################################################

# The hostname of the openocd server may be set in OPENOCD_HOSTNAME 
# if undefined localhost is used
# menu.sh sets :
# NAUTEFF_AP_PATH wich points to directory of preject,
# NAUTEFF_AP_BUILD_DIR wich point to build,
# Determine the path to project, it the parent directory of menu.sh
SCRIPT_DIR=$(dirname "$(realpath "$0")")
export NAUTEFF_AP_PATH=`dirname $SCRIPT_DIR`
export NAUTEFF_AP_BUILD_DIR=$NAUTEFF_AP_PATH/build

cd  $NAUTEFF_AP_PATH

main_menu() {
    while true; do
        CHOICE=$(dialog --clear --backtitle "Menu Principal" \
            --title "Menu" \
            --menu "Sélectionnez une option :" 18 50 5 \
            1 "Compilation" \
            2 "Compilation totale" \
            3 "Carte mémoire" \
            4 "Débogage" \
            5 "Documentation Auto doxygen" \
            6 "Documentation latex" \
            t "tests" \
	    f "Format code" \
            z "Fin" \
            2>&1 >/dev/tty)

        case $CHOICE in
            1)  echo "Compilation :"
                cd ${NAUTEFF_AP_PATH}
                make -j -f Makefile.user
                read -p "Appuyez sur <Return> pour continuer" rep
                ;;

            2)  echo "Compilation Totale clean and complete build"
                cd ${NAUTEFF_AP_PATH}
                make -j -f Makefile.user clean
                make -j -f Makefile.user
                read -p "Appuyez sur <Return> pour continuer" rep
                ;;

            3)  echo "Carte mémoire"
                ${NAUTEFF_AP_PATH}/scripts/carte.py ${NAUTEFF_AP_PATH}/build/NauteffAutoPilot.elf | less
                read -p "Appuyez sur <Return> pour continuer" rep
                ;;

            4) ${NAUTEFF_AP_PATH}/scripts/debug.sh
                ;;

            5)  echo "Génération de documentation (doxygen)"
		pwd
		read
                ${NAUTEFF_AP_PATH}/scripts/mk_doxy.sh
                read -p "Appuyez sur <Return> pour continuer" rep
                ;;

            6)  echo "Génération de documentation (latex)"
                ${NAUTEFF_AP_PATH}/scripts/mk_latex.sh
                read -p "Appuyez sur <Return> pour continuer" rep
                ;;

            f)  echo "Formatage des fichiers sources"
                ${NAUTEFF_AP_PATH}/scripts/format.sh
		read -p "Appuyez sur une touche pour continuer" rep
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
                ${NAUTEFF_AP_PATH}/scripts/test_nmea.sh
                read -p "Fin de test. Appuyez sur <Return> pour continuer" rep
                ;;

            2)  echo "Test conversions"
                ${NAUTEFF_AP_PATH}/scripts/test_any.sh cvt
                read -p "Fin de test. Appuyez sur <Return> pour continuer" rep
                ;;

            3)  echo "Test Géométrie"
                ${NAUTEFF_AP_PATH}/scripts/test_any.sh geom
                read -p "Fin de test. Appuyez sur <Return> pour continuer" rep
                ;;
            4) break ;;
            *) break ;;
        esac
    done
}

main_menu
clear

