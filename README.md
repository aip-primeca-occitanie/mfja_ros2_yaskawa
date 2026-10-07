# SAE GEII - Yaskawa ROS2

## Setup

### Réseau

Dans la VM, vérifier que le robot est joignable :

$ ping 10.100.12.133
$ ping 10.100.12.134

Si la connexion de l'agent microROS avec les robots semblent instable, ouvrir les paramètres réseau de la VM et désactiver l'ethernet "enp0s8".

### Lancer l'agent microROS

L'agent microROS communique avec les robots, qui connaissent l'IP de la machine sur laquelle il tourne.
L'agent doit tourner en permenance.

runmicrorosdocker

Cette commande est un alias pour :

docker run \
  -it \
  --rm \
  --net=host \
  --user=$(id -u):$(id -g) \
  microros/micro-ros-agent:jazzy \
    udp4 \
    --port 8888

Pour débugger et voir l'état des robots :

source ~/colcon_ws/install/setup.bash 
ros2 run mfja_motoman_hc10 debug_listener.py 

Pour rappel :
- IP du Yaskawa gauche : 10.100.12.134
- IP du Yaskawa droit : 10.100.12.133

Une fois que l'agent tourne, vérifiez que vous voyez les robots :

$ ros2 topic echo /yaskawa_LEFT/joint_states --once
$ ros2 topic echo /yaskawa_RIGHT/joint_states --once

### Recompiler les paquets des yaskawa (si nécessaire)

cd ~/colcon_ws
colcon build --mixin release --parallel-workers 4 --packages-select mfja_motoman_hc10 motoman_hc10_moveit_config
source install/setup.bash

## Simulation

### Lancer la simulation

Yaskawa gauche :

source ~/colcon_ws/install/setup.bash
ros2 launch motoman_hc10_moveit_config demo_sim.launch.py robot:=left

Yaskawa droit :

ros2 launch motoman_hc10_moveit_config demo_sim.launch.py robot:=right


## Executer une trajectoire

ros2 run mfja_motoman_hc10 ../trajectories/trajectory.csv --robot left
ros2 run mfja_motoman_hc10 ../trajectories/trajectory.csv --robot right

## Vrai hardware

### Créer une trajectoire de test

Si connecté au vrai robot, pour créer une trajectoire qui diffère d'un delta de la configuration actuelle :

cd ~/colcon_ws/
ros2 run mfja_motoman_hc10 create_trajectory_from_current.py --robot right --delta 0 0 0 0 0 1 --filename ~/colcon_ws/src/mfja_ros2_yaskawa/mfja_motoman_hc10/trajectories/trajectory.csv

Pour créer une trajectoire entre deux configurations q1 et q2 :

TODO
 
### Executer une trajectoire

La clé sur le teach pendant du robot doit être en mode "remote".

ros2 run mfja_motoman_hc10 execute_trajectory.py ../trajectories/trajectory.csv --robot left
ros2 run mfja_motoman_hc10 execute_trajectory.py ../trajectories/trajectory.csv --robot right

### Allumer/éteindre la puissance (servos)

La clé sur le teach pendant du robot doit être en mode "remote".

Robot gauche :

ros2 service call /yaskawa_LEFT/start_traj_mode motoros2_interfaces/srv/StartTrajMode {}
ros2 service call /yaskawa_LEFT/stop_traj_mode std_srvs/srv/Trigger {}

Robot droit :

ros2 service call /yaskawa_RIGHT/start_traj_mode motoros2_interfaces/srv/StartTrajMode {}
ros2 service call /yaskawa_RIGHT/stop_traj_mode std_srvs/srv/Trigger {}

## I/O link

Piste à étudier :
https://github.com/b-robotized/motoros2_hw_interfaces

