# Objectifs du 19/02/2021

PO: Agathe Archet

## Fonctionnalités attendues

* Tests sur l'algo de sélection de balles - Alexandre et Bertrand
* Amélioration de l'algorithme haut-niveau - Kévin
* Amélioration de la commande bas niveau et dynamique du robot - Robin
* Correction beugs aruco / algorithme bas niveau - Robin et Agathe

## Tâches à réaliser

* Continuation de l'agorithme de commande : gestion de l'évitement des joueurs avec des champs de potentiels répulsifs
* Tests sur l'algorithme de sélection de balle : tests de différentes configurations et éventuel débuggage
* Reconstruction de l'algorithme bas-niveau : ajustement de l'interie du robot + filtre de la commande
* Se mettre d'accord sur les types de topics, les conventions et ordres de grandeurs en sortie d'algorithme


## Challenges techniques

* Effet file d'attente/dépendance entre les différents algorithmes à gérer ?
* Gérer l'évolution des noms de certaines fonctions entre les versions 3.2.0 et 4.2.0 d'OpenCV (pour les arucos) ou trouver une autre solution ?

