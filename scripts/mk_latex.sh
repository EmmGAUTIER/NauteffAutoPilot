#!/bin/bash
##############################################################################
# Latex compilation                                                          #
#----------------------------------------------------------------------------#
#                                                                            # 
##############################################################################

Fichiers="Nauteff-Conception_Logiciel \
Nauteff-Glossaire \
Nauteff-OutilsDéveloppement \
Nauteff-ReflexionsPreliminaires \
Nauteff-Specification \
procedurePID"


if [ "$NAUTEFFAUTOPILOT_PATH" != "" ]
then
    cd $NAUTEFFAUTOPILOT_PATH
fi

#export TEXINPUTS=".:doc/latex/src://"

cd ./doc/latex

for fic in $Fichiers
do
    #pdflatex -I=./doc/latex/src/ -halt-on-error  -output-directory=./doc/latex/build  ./doc/latex/src/$fic
    pdflatex -I=./doc/latex/ -halt-on-error  -output-directory=../build  ${fic}.tex
    cp ../build/${fic}.pdf ../pdf
done

