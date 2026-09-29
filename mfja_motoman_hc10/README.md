# MFJA ROS2 Yaskawa

Nous utilisons MotoROS2 (https://github.com/yaskawa-global/motoros2) pour utiliser les robots Yaskawa avec ROS2.

MotoROS2 est déjà installé sur les robots, selon la procédure d'installation du README.

Sept 2026, MotoROS2 version
Procédure d'installation du README du commit 41ac074a879bb9f75fd1f96618ce54b4899cfd1a


## Guide rapide

- [ ] MotoROS2 doit être installé sur les robots
- [ ] L'agent micro-ROS doit tourner sur une machine Linux
- [ ] La machine Linux doit avoir la bonne configuration réseau
- [ ] "MotoROS2 Client Interface Dependencies" doit être installé sur la machine Linux


## Configuration réseau

Les deux robots communiquent avec un agent micro-ROS. La machine (en 315 une VM) qui fait tourner l'agent micro-ROS doit être correctement configurée vis-à-vis du réseau pour pouvoir communiquer avec les robots.


## Installer MotoROS2 Client Interface Dependencies

Il faut suivre les instructions du paquet suivant pour installer les dépendances qui définissent les messages, services etc :

https://github.com/yaskawa-global/motoros2_client_interface_dependencies


## Contrôle du robot via ROS2

Le contrôle des robots via ROS2 se fait via le serveur d'action ROS2 "FollowJointTrajectory".

Attention, dû à des contraintes intrinsèques à micro-ROS, MotoROS2 impose une limite maximale au nombre de points (JointTrajectoryPoints) d'une trajectoire JointTrajectory envoyée à l'action FollowJointTrajectory : la limite actuelle est actuellement 200 points.


## Installer les paquets de description des robots

https://github.com/Yaskawa-Global/motoman_ros2_support_packages.git