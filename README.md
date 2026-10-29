# MFJA ROS2 Yaskawa

Ce dépôt regroupe le nécessaire pour contrôler les robots Yaskawa HC10 et HC10DT via ROS2. Il utilise [MotoROS2](https://github.com/yaskawa-global/motoros2).

## Prérequis

- Une machine Ubuntu 24.04 connectée au même réseau que les robots
- ROS2 Jazzy et docker installé sur cette machine
- [MotoROS2](https://github.com/yaskawa-global/motoros2) installé sur les contrôleurs des robots

## Grandes étapes

Pour le vrai hardware :

- [Vérifier la configuration réseau de la machine Ubuntu](#Réseau)
- [Lancer l'agent microROS](#Lancer-l'agent-microROS)
- (optionnel) [Lancer RViz](#Visualiser-le-robot)
- [Exécuter une trajectoire](#Executer-une-trajectoire(vrai-hardware))
- [Contrôler le robot via RViz](#Contrôler-le-robot-via-RViz)

Pour le faux hardware (simulation) :

- (faux hardware) [Lancer la simulation et RViz](#Lancer-la-simulation)
- [Executer une trajectoire](#Executer-une-trajectoire(simulation))

Alternativement, une IHM a été développée par des étudiants GEII pour executer leurs trajectoires, voir (#Utiliser-l-IHM).

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

### Debug de MotoROS2

Le script debug_listener.py provient de MotoROS2 et permet de voir l'état des robots :

```bash
source ~/colcon_ws/install/setup.bash 
ros2 run mfja_motoman_hc10 debug_listener.py 
```

Une fois que l'agent tourne, vérifiez que vous voyez les robots :

```sh
ros2 topic echo /yaskawa_LEFT/joint_states --once
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

N'oubliez pas si ce n'est pas déjà fait de sourcer le workplace colcon dans chaque terminal :

```sh
source ~/colcon_ws/install/setup.bash
```

### Lancer la simulation

Yaskawa gauche :

```sh
ros2 launch motoman_hc10_moveit_config demo_mfja.launch.py robot:=left hardware:=sim
```

Yaskawa droit :

```sh
ros2 launch motoman_hc10_moveit_config demo_mfja.launch.py robot:=right hardware:=sim
```


## Executer une trajectoire (simulation)

```sh
ros2 run mfja_motoman_hc10 execute_trajectory.py $TRAJ_PATH/trajectory.csv --robot left --sim
ros2 run mfja_motoman_hc10 execute_trajectory.py $TRAJ_PATH/trajectory.csv --robot right --sim
```

## Vrai hardware

### Visualiser le robot

```sh
ros2 launch motoman_hc10_moveit_config demo_mfja.launch.py robot:=right hardware:=real
ros2 launch motoman_hc10_moveit_config demo_mfja.launch.py robot:=left hardware:=real
```

### Créer une trajectoire de test

Si connecté au vrai robot, pour créer une trajectoire qui diffère d'un delta de la configuration actuelle :

```sh
ros2 run mfja_motoman_hc10 create_trajectory_from_current.py --robot right --delta 0 0 0 0 0 1 --filename $TRAJ_PATH/trajectory.csv
```

Pour créer une trajectoire entre deux configurations q1 et q2 :

TODO
 
### Executer une trajectoire (vrai hardware)

La clé sur le teach pendant du robot doit être en mode "remote". Ce script exécute toutes les étapes décrites dans le README.md de motoROS2, à savoir vérifier l'état du robot, activer la puissance, envoyer la trajectoire, ...


```sh
ros2 run mfja_motoman_hc10 execute_trajectory.py $TRAJ_PATH/trajectory.csv --robot left
ros2 run mfja_motoman_hc10 execute_trajectory.py $TRAJ_PATH/trajectory.csv --robot right
```

### Executer plusieurs trajectoires

```sh
./run_multip_traj_yaska.sh 3 left trajectory
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

### Contrôler le robot via RViz

Après avoir lancé RViz et allumé les servos, sous RViz, vous pouvez bouger le robot via la vue 3D ou via la liste des articulations, et cliquer sur "Plan" puis sur "Execute".

## Utiliser l'IHM

### Setup

L'IHM nécessite certains paquets python. Il est recommandé de créer un environnement virtuel python pour les installer et lancer l'IHM.

Créez un environnement virtual que colcon ignorera pendant les builds :
```sh
cd ~/colcon_ws/src/mfja_ros2_yaskawa/mfja_motoman_hc10
python3 -m venv .venv --system-site-packages
touch .venv/COLCON_IGNORE
```

Activez l'environnement et installez les paquets requis :

```sh
source .venv/bin/activate
pip install -r requirements.txt
```

Lancez l'IHM :

```sh
python scripts/supervision_IHM.py
```

## Troubleshooting

### Erreur en fin de trajectoire

Parfois la trajectoire finit en erreur avec le code -500302 ou -100303, même quand les trajectoires semblent s'être correctement executées.


### Cannot enable trajectory mode (cannot enable servo power)

S'il y a une erreur comme :
```sh
[ERROR] [1792662097.426931669] [yaskawa_RIGHT.motoros2_single_trajectory_executor]: Failed to enable trajectory mode. Result code: 102
```

ou :

```sh
motoros2_interfaces.srv.StartTrajMode_Response(result_code=motoros2_interfaces.msg.MotionReadyEnum(value=102), message="The controller has an active Error: Can't turn on servo because: 'Servo power cannot be turned on' (0x3450)")
```

La cause est probablement une porte de sécurité laissée ouverte. Le robot ne peut pas démarrer la puissance en mode remote tant que le circuit de sécurité n'est pas fermé. Fermez les portes et recommencez.

## I/O link

Piste à étudier :
https://github.com/b-robotized/motoros2_hw_interfaces


