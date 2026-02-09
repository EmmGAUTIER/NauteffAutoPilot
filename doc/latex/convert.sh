#/bin/sh
#
# Convertit les fichiers au format dia en fichier au format eps.
# Les graphes au format eps sont destiné à être inclus dans les doc. latex.
# Ces formats sont des formats vectoriels.
#

for file in *.dia
do
	# dia store the diagramms in dia files.
	# The CLI commande dia --export file.fmt converts to the format fmt
	echo
	echo  $file
	dia   --export   ${file%.dia}.eps $file 
done

