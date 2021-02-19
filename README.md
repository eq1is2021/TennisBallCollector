# Tennis Ball Collector

Ce dépôt propose une solution afin de récupérer les balles sur un terrain de tennis et les ramener dans des zones.

## Installation

#### Pour installer, cloner le git dans un workspace ROS (ici wk_ros_tennis):

```
cd src
git clone https://github.com/{User}/TennisBallCollector.git
```

Ensuite, compilation :
```
cd ..
colcon build --symlink-install
```

#### Modification du bashrc :

```
gedit ~/.bashrc
```
Et ajouter la ligne :
```
source /home/{User}/wk_ros_tennis/install/setup.bash
```

Puis relancer la console ou tapper :
```
bash
```

et enfin : 
```
. install/setup.bash
```


## Lancer la simulation

### Dépendances

Pour eloquent :
- ros-eloquent-vision-opencv

Pour foxy (peut-être nécesssaire) :
- ros-foxy-image-pipeline

Librairie python à installer: 
python3.6 -m pip install transforms3d

### Démarrer la simulation

#### Pour démarrer la simulation et tous les noeuds :
```
ros2 launch robot_control launcher.launch.py
```

#### Pour démarrer chaque noeud séparément
Pour démarrer la simulation (le court de tennis seul) : 
```
ros2 launch tennis_court tennis_court.launch.py
```

Pour démarrer la simulation avec le robot : 
```
ros2 launch tennis_court launcher.launch.py
```

Pour démarrer la detection de balles :
```
ros2 run detection_balles detection_balles
```

Pour démarrer la labelisation des balles:
(nécessite d'avoir démarrer le node de détection des balles)

```
ros2 run labelisation_balles labelisation
```

Pour démarrer la localisation des arucos :

```
ros2 run localisation_aruco viewer
```

Pour démarrer la détection des personnes :
```
ros2 run detection_joueurs detection_joueurs
```

Pour démarrer la préhension des balles (attrape les balles proches) :
```
ros2 run detection_balles_cage detection_balles_cage
```

Pour démarrer le control du robot :
```
ros2 run robot_control rbt_control
```

Pour démarrer la commande par champ de poteltiel du robot :
```
ros2 run field_command field_command
```

![gif](https://github.com/eq1is2021/TennisBallCollector/blob/master/gif1.gif)

## Groupe

### Membres

- Agathe (archetag)
- Alexandre (EdouardSaladier)
- Bertrand (Bentur)
- Kévin (affraike)
- Robin (RSCZ)


### Gestion de projet

###### Lien vers notre [Taiga](https://tree.taiga.io/project/0f719389-854b-4732-8624-3bbe33ae96a3-ramasseur-de-balles).

## Structure du dépôt

Ce dépôt doit être cloné dans le dossier `src` d'un workspace ROS 2.

### Package `tennis_court`

Le dossier `tennis_court` est un package ROS contenant le monde dans lequel le robot ramasseur de balle devra évoluer ainsi qu'un script permettant de faire apparaître des balles dans la simulation.
Ce package ne doit pas être modifié (sauf pour l'histoire de l'ombre).
Consulter le [README](tennis_court/README.md) du package pour plus d'informations.


### Documents

Le dossier `docs` contient tous les documents utiles au projet:
- Des [instructions pour utiliser Git](docs/GitWorkflow.md)
- Un [Mémo pour ROS 2 et Gazebo](docs/Memo_ROS2.pdf)
- Les [slides de la présentation Git](docs/GitPresentation.pdf)


### Rapports

Le dossier `reports` doit être rempli avec les rapports d'[objectifs](../reports/GoalsTemplate.md) et de [rétrospectives](../reports/DebriefTemplate.md) en suivant les deux templates mis à disposition. Ces deux rapports doivent être rédigés respectivement au début et à la fin de chaque sprint.
