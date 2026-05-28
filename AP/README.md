#Ce répertoire contient les sources spécifiques au pilote automatique et des fonctions auxiliaires.

## Répertoire tasks
Ce répertoire contient les sources des tâches.
Les tâches utilisent un boucle infinie qui lit les informations et ordres dans des messages
qui sont transférées dans des queues et traitent ces messages. La tâche Blink, très imple,
est un bon début de lecture pour comprendre ces mécanismes.

    

### Tâche blink
Cette tâche fait clignoter une LED. Elle est utilisée uniquement pour la mise au point.
  
### Tâche mems
Elle gère les capteurs MEMS (LSM9DS1) et envoie les information d'orientation vers la tâche autopilot et vers UART2.
Elle lit les données brutes de des capteurs, applique les correction de biais et de gain,
calcule le cap, le roulis et le tangage et le quaternion d'orientation et les diffuse.   
Cette tâche permet aussi l'étalonnage des capteurs.

### Tâche autopilot
Elle recçoit les information d'orientation du navire les ordres du navigateur et les informations du moteur.
À réception de l'ordre de maintenir un cap, elle compare le cap demandé et la cap demandé et
envoie les commandes de barre à la tâche moteur. Elle implémente un régulateur PID.
   
### Tâche motor
Elle assure la commande et la surveillance du moteur. Elle reçoit ses ordres de la tâche autopilot.
Elle mesure le courant du moteur et l'arrête en cas de courant trop fort (moteur bloqué) et
évalue la position de la barre. Elle envoie des compte-rendus à la tâche autopilot.


### Fonction apmain
C'est l'équivalent de la fonction main() du C. Elle initialise et crée les tâches puis elle démarre le scheduler.
 
### Tâche service
C'est une tâche auxiliare assurant la gestion de l'UART.

### Tâche apdialog
Elle reçoit les ordres provenant de UART2, les décode et envoie les ordres aux autres tâches.

