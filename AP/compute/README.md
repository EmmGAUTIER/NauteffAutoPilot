#Fonctions de calcul du pilote automatique

## De la géométrie 3D, des vecteurs et des matrices
Les fichiers geom.c et .h contient les sources pour manipuler les vecteurs,
Les fichiers matrix.c et matrix.h ceux pour les manipulations de matrices 3D
Les vecteurs 3D sont massivement utilisés pour les calculs.
Il servent à indiquer des directions et parfois une intensité.
Les fonctions permettent de les manipuler; le traitement le plus emblématique
est le produit vectoriel utilisé par exemple par la fusion des données de capteurs MEMS.

## Des quaternions
Appelés hypercomplexes par les mathématiciens, ils sont composés de quatre nombres.
Les orientations sont souvent représentées par les angles d'Euler : cap, roulis et tangage.
Nous les utilisons les angles d'Euler et les quaternions.
Les quaternions permettent de représenter les rotations dans l'espace à trois dimensions
avec l'axe et l'angle de rotation. Leur avantage est qu'ils sont simples à manipuler,
plus rapides et plus stables que les calculs avec les matrices et surtout qu'ils permettent
de s'affranchir du "blocage de cardan" possibles avec les angles d'Euler.

## Calculs d'orientation (AHRS)

Les calculs d'orientation utilisent les données d'un accéléromètre, d'un gyromètre
et d'un magnétomètre.
L'accéléromètre sert principalement à savoir "ou est le bas" pour corriger la gîte et le tangage.
Le magnétomètre sert à mesurer le champ magnétique, il est indispensable pour savoir ou est le nord.
Le gyromètre donne les mouvements de rotation.
La détermination de l'orientation à partir de ces données s'appelle la fusion des données.
Plusieurs algorithmes sont disponibles, leurs sources sont dans les fichiers
dont les nom commencent par ahrs.
Les données du magnétomètre sont bruitées, il est préférable de calculer l'orientation à partir
de l'orientation précédante et le gyromètre puis de racaler avec le magnétomètre et l'accéléromètre. 

### DT0058
Ce calcul est la retranscription de la note DT0058 de ST Electronics.
C'est une fusion de donnée simple qui utilise l'accélération et le champs magnétique seulement et
avec un trigonométrie simple. 

### Simple
Ce calcul utilise les produits vectoriels et intègre la donnée du gyromètre pour calculer le cap
Il utilise aussi les données du magnétomètre et de l'accéléromètre.

### Quat : à base de quaternions.
Pour expériementer les quaternions.
Calcule un quaternion d'orientation et à chaque nouvelles valeurs des capteurs calcule
un nouveau quaternion en intégrant les données du gyromètre, un nouveau quaternion d'orientation avec
le magnétomètre et l'accéléromètre puis il ajoute ces deux quaternions.


d'orient  
   

    
