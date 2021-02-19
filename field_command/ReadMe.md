Ce noeud prend en paramètre:

* La position de robot (localisation_aruco)
* L'orientation du robot (yaw_ctrl)
* La position du prochain objectif (robot_control)
* La position des joueurs (labelisation_balles)

Son but est de générer le champ de potentiel à appliquer sur le terrain.
Il retourne un message de type Twist donnant la vitesse et l'angle de consigne à donner au robot.
Il publie sur le topic /cmd_yaw.
