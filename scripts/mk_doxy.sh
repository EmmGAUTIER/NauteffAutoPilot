#!/bin/bash
# cd $NAUTEFFAUTOPILOT_PATH
pwd

# Nom du fichier de configuration Doxygen
DOXYFILE="doc/doxyfile"
OUTPUT_DIR="doc/doxygen"

# Génère la documentation avec Doxygen
echo "Génération de la documentation Doxygen..."
doxygen "$DOXYFILE"


