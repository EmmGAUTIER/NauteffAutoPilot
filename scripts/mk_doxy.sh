#!/bin/bash

echo "----->" $NAUTEFF_AP_PATH
cd $NAUTEFF_AP_PATH

# Nom du fichier de configuration Doxygen
DOXYFILE="doc/doxygen/doxyfile"
OUTPUT_DIR="doc/doxygen"

# Génère la documentation avec Doxygen
echo "Génération de la documentation Doxygen..."
doxygen "$DOXYFILE"


