# MFJA ROS2 Yaskawa

## Setup

### Réseau

La configuration locale est détaillée sur le Moodle de la plateforme.

### Lancer l'agent microROS

L'agent microROS communique avec les robots, qui connaissent l'IP de la machine sur laquelle il tourne. L'agent doit tourner en permenance.

```
runmicrorosdocker
```

Cette commande est un alias pour lancer le docker:

```sh
alias runmicrorosdocker=docker run \
  -it \
  --rm \
  --net=host \
  --user=$(id -u):$(id -g) \
  microros/micro-ros-agent:jazzy \
    udp4 \
    --port 8888
```

### Debug

Pour débugger et voir l'état des robots :

```bash
source ~/colcon_ws/install/setup.bash 
ros2 run mfja_motoman_hc10 debug_listener.py 
```

Une fois que l'agent tourne, vérifiez que vous voyez les robots :

```sh
ros2 topic echo /yaskawa_LEFT/joint_states --once
```

```sh
ros2 topic echo /yaskawa_RIGHT/joint_states --once
```

### Recompiler les paquets des yaskawa (si nécessaire)

```sh
cd ~/colcon_ws
colcon build --mixin release --parallel-workers 4 --packages-select mfja_motoman_hc10 motoman_hc10_moveit_config
source install/setup.bash
```

### Emplacement des trajectoires

Une variable d'environnement peut être utilisée pour définir le chemin vers le dossier où les trajectoires sont stockées. Par exemple :

```sh
export TRAJ_PATH=~/colcon_ws/src/mfja_ros2_yaskawa/mfja_motoman_hc10/trajectories/traj_etudiants
```

## Simulation

### Lancer la simulation

Yaskawa gauche :

```sh
source ~/colcon_ws/install/setup.bash
ros2 launch motoman_hc10_moveit_config demo_sim.launch.py robot:=left
```

Yaskawa droit :

```sh
ros2 launch motoman_hc10_moveit_config demo_sim.launch.py robot:=right
```


## Executer une trajectoire

```sh
ros2 run mfja_motoman_hc10 execute_trajectory_sim.py $TRAJ_PATH/trajectory.csv --robot left
ros2 run mfja_motoman_hc10 execute_trajectory_sim.py $TRAJ_PATH/trajectory.csv --robot right
```

## Vrai hardware

### Visualiser le robot

```sh
ros2 launch motoman_hc10_moveit_config demo_real.launch.py robot:=right
ros2 launch motoman_hc10_moveit_config demo_real.launch.py robot:=left
```

### Créer une trajectoire de test

Si connecté au vrai robot, pour créer une trajectoire qui diffère d'un delta de la configuration actuelle :

```sh
ros2 run mfja_motoman_hc10 create_trajectory_from_current.py --robot right --delta 0 0 0 0 0 1 --filename $TRAJ_PATH/trajectory.csv
```

Pour créer une trajectoire entre deux configurations q1 et q2 :

TODO
 
### Executer une trajectoire

La clé sur le teach pendant du robot doit être en mode "remote".

```sh
ros2 run mfja_motoman_hc10 execute_trajectory.py $TRAJ_PATH/trajectory.csv --robot left
ros2 run mfja_motoman_hc10 execute_trajectory.py $TRAJ_PATH/trajectory.csv --robot right
```

### Allumer/éteindre la puissance (servos)

La clé sur le teach pendant du robot doit être en mode "remote".

Robot gauche :

```sh
ros2 service call /yaskawa_LEFT/start_traj_mode motoros2_interfaces/srv/StartTrajMode {}
ros2 service call /yaskawa_LEFT/stop_traj_mode std_srvs/srv/Trigger {}
```

Robot droit :

```sh
ros2 service call /yaskawa_RIGHT/start_traj_mode motoros2_interfaces/srv/StartTrajMode {}
ros2 service call /yaskawa_RIGHT/stop_traj_mode std_srvs/srv/Trigger {}
```

### Executer plusieurs trajectoires

```sh
./run_multip_traj_yaska.sh 3 left trajectory
```

## Troubleshooting

### Erreur en fin de trajectoire

Parfois la trajectoire finit en erreur avec le code -500302 ou -100303, même quand les trajectoires semblent s'être correctement executées.

## I/O link

Piste à étudier :
https://github.com/b-robotized/motoros2_hw_interfaces

