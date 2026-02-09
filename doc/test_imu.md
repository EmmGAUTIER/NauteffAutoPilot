## Valeurs de référence des accélérations

**Rappel :** L'accélération est positive vers le bas 


|   roll   | pitch   |   ax   |   ay   |   az   |
|----------|---------|--------|--------|--------|
|  0       | 0       |  0     |  0     |  9.81  |
| 30 port  | 0       |  0     | -4.905 |  8.496 |
| 30 stbd  | 0       |  0     |  4.905 |  8.496 |
|  0       | 30 down |  4.905 |  0     |  8.496 |
|  0       | 30 up   | -4.905 |  0     |  8.496 |

## Valeurs de référence des quaternions d'orientation

Objectif : tester les orientation et valeurs des quaternions.

|  cap   |   roll   | pitch   |   qw   |   qx   |   qy   |   qz    |
|--------|----------|---------|--------|--------|--------|---------|
| north  |  0       | 0       |  1     | +0     |  0     |  0      |
| east   |  0       | 0       |  0.707 | +0     |  0     |  0.707  |
| south  |  0       | 0       |  0     | +0     |  0     |  1      |
| west   |  0       | 0       |  0.707 | +0     |  0     | -0.707  |
|        |          |         |        |        |        |         |
| north  | 30 port  | 0       |  0.966 | -0.259 |  0     |  0      |
| east   | 30 port  | 0       |  0.683 | -0.183 |  0.183 |  0.683  |
| south  | 30 port  | 0       |  0     |  0     |  0.259 |  0.966  |
| west   | 30 port  | 0       |  0.683 | -0.183 | -0.183 | -0.683  |
|        |          |         |        |        |        |         |
| north  | 30 stbd  | 0       |  0.966 |  0.259 |  0     |  0      |
| east   | 30 stbd  | 0       |  0.683 |  0.183 | -0.183 |  0.683  |
| south  | 30 stbd  | 0       |  0     |  0     | -0.259 |  0.966  |
| west   | 30 stbd  | 0       |  0.683 |  0.183 |  0.183 | -0.683  |
|        |          |         |        |        |        |         |
| north  |  0       | 30 down |  0.966 |  0     |  0.259 |  0      |
| east   |  0       | 30 down |  0.683 |  0.183 |  0.183 |  0.683  |
| south  |  0       | 30 down |  0     |  0.259 |  0     |  0.966  |
| west   |  0       | 30 down |  0.683 | -0.183 |  0.183 | -0.683  |
|        |          |         |        |        |        |         |
| north  |  0       | 30 up   |  0.966 |  0     | -0.259 |  0      |
| east   |  0       | 30 up   |  0.683 | -0.183 | -0.183 |  0.683  |
| south  |  0       | 30 up   |  0     | -0.259 | -0     |  0.966  |
| west   |  0       | 30 up   |  0.683 |  0.183 | -0.183 | -0.683  |

