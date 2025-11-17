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
procedurePID \
essai"


if [ "$NAUTEFFAUTOPILOT_PATH" != "" ]
then
    cd $NAUTEFFAUTOPILOT_PATH
fi

#export TEXINPUTS=".:doc/latex/src://"

cd $NAUTEFFAUTOPILOT_PATH/doc/latex

for fic in $Fichiers
do
    if  [ ../pdf/${fic}.pdf -ot ${fic}.tex ] || [ ../pdf/${fic}.pdf -ot ${fic}.tex ]
    then
        pdflatex  -aux-directory=../build -halt-on-error  -output-directory=../build  ${fic}.tex
        bibtex ../build/${fic}
        pdflatex  -aux-directory=../build -halt-on-error  -output-directory=../build  ${fic}.tex
        pdflatex  -aux-directory=../build -halt-on-error  -output-directory=../build  ${fic}.tex
        cp ../build/${fic}.pdf ../pdf
    fi
done

