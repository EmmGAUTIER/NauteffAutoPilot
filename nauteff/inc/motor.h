/*
 * @file Module "motor"
 * Ce module assure le contrôle du moteur.
 *  - Commande de l'embrayage et allumage de la LED verte de la carte Nucleo
 *  - Mise en marche et arrêt du moteur dans un sens ou l'autre
 *  - Détection de défaut ou de butée
 *  - Gestion de la durée de fonctionnement
 *  - Estimation de l'effort et ajustement de la durée de fonctionnement
 *  - Communication avec la tâche principale
 */

//#include "stmregs.h"
//#include "nauteff.h"

extern MessageBufferHandle_t bufferMotor;

void taskMotor(void *);
