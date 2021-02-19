# Robot Control

Node de commande du robot: donne un objectif à atteindre à l'algorithme de mouvement à partir des informations de position du robot, des balles, et de la présence ou non d'une balle dans le catcher

#### Lancement du node:
```
ros2 run robot_control rbt_control
```

#### Entrées:
- Position du robot: topic /aruco_twist format Twist
- Position des balles: topic /balles_labels format PoseArray
- Présence d'une balle dans le catcher: topic /catcher_up format Bool

#### Sortie:
- Position de l'objectif à atteindre: topic /robot_objective format Point
	-X,Y sont les coordonnées 2D
	-Z sert de marqueur pour l'algorithme de contrôle: 1 pour déplacement, 2 pour arrêt (annule le déplacement et l'objectif)