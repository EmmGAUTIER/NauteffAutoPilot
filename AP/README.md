Ce répertoire contient les sources spécifiques au pilote automatique et des fonctions auxiliaires.

##Répertoire tasks
###Tâche mems
Elle gère les capteurs MEMS (LSM9DS1) et envoie les information d'orientation vers la tâche autopilot et vers UART2.
Elle lit les données brutes de des capteurs, applique les correction de biais et de gain,
calcule le cap, le roulis et le tangage et le quaternion d'orientation et les diffuse.   
Cette tâche permet aussi l'étalonnage des capteurs.

###Tâche autopilot
Elle recçoit les information d'orientation du navire les ordres du navigateur et les informations du moteur.
À réception de l'ordre de maintenir un cap, elle compare le cap demandé et la cap demandé et
envoie les commandes de barre à la tâche moteur. Elle implémente un régulateur PID.
   
###Tâche motor
Elle assure la commande et la surveillance du moteur. Elle reçoit ses ordres de la tâche autopilot.
Elle mesure le courant du moteur et l'arrête en cas de courant trop fort (moteur bloqué) et
évalue la position de la barre. Elle envoie des compte-rendus à la tâche autoipilot.

###tâche apmain
C'est la première tâche exécutée. Elle démarre les autres tâches.
 
###tâche service
C'est une tâche auxiliare assurant la gestion de l'UART.

###Tâche apdialog
Elle reçoit les ordres provenant de UART2, les décode et envoie les ordres aux autres tâches.

###Tâche blink
Cette tâche fait clignoter une LED. Elle est utilisée uniquement pour la mise au point.