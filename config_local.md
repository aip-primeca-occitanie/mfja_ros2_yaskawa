# SAE GEII - Yaskawa ROS2

## VM

- Nom : "YaskawaROS2"
- VirtualBox
- Emplacement : E:\Machines_Virtuelles\YaskawaROS2
- Login : "user" / mdp : "user"

## Réseau

Dans la VM, vérifier que le robot est joignable :

$ ping 10.100.12.133
$ ping 10.100.12.134

Si la connexion de l'agent microROS avec les robots semblent instable, ouvrir les paramètres réseau de la VM et désactiver l'ethernet "enp0s8".
Vérifier que "enp0s3" est activé. Dans les paramètres de configuration de "enp0s3", onglet IPv4, sélectionnez "Manual" et entrez :
- Adress : 10.100.12.171 (c'est l'IP de la VM sur le réseau local, inscrite dans les paramètres sur les contrôleurs des robots)
- Netmask : 255.255.255.0


Pour rappel :
- IP du Yaskawa gauche : 10.100.12.134
- IP du Yaskawa droit : 10.100.12.133
- IP de la VM faisant tourner l'agent microROS : 10.100.12.171
