import customtkinter as ctk        
import tkinter as tk              
import numpy as np                 
from scipy.optimize import least_squares 
import math                       
import csv                        
import os                        
import sys                        
import threading                  
import time                       
import socket                     
import subprocess                 
import signal                     
from pathlib import Path         
from PIL import Image             

import matplotlib.pyplot as plt    
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg 
from matplotlib.figure import Figure 

from coppeliasim_zmqremoteapi_client import RemoteAPIClient 
from pymodbus.client import ModbusTcpClient                

try:
    import test 
except ImportError:
    pass 

# gestion des .exe et .logs

# permet de trouver les fichiers  même si le script est compile en .exe 
def resource_path(relative_path):
    try:
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")
    return os.path.join(base_path, relative_path)

# permet d'affichage des événements (console et interface graphique)
def gui_log(msg, level="INFO"):
    timestamp = time.strftime("%H:%M:%S")
    log_line = f"[{timestamp}] [{level}] {msg}\n"
    print(log_line.strip()) # affiche dans la console python 
    # si l'application graphique est lancee, on injecte le texte dans la page des logs via le thread principal (after 0)
    if 'app' in globals() and hasattr(app, 'log_event'):
        app.after(0, lambda: app.log_event(log_line))

# Fonction pour ouvrir physiquement un terminal Ubuntu 
def launch_external_terminal(title, command, keep_open=True):
    """ Ouvre un terminal Ubuntu explicite (gnome-terminal) en chargeant l'environnement """
    # chargement prealable du .bashrc et de ROS 2 humble + workspace local pour que les commandes ROS soient reconnues
    setup_env = "source ~/.bashrc 2>/dev/null; source /opt/ros/humble/setup.bash 2>/dev/null; source ~/colcon_ws/install/setup.bash 2>/dev/null;"
    # a la fin de la commande : rester ouvert ou se fermer au bout de 2 sec
    end_action = "exec bash" if keep_open else "sleep 2; exit"
    # construction de la commande finale 
    full_cmd = f"gnome-terminal --title='{title}' -- bash -ic '{setup_env} {command}; echo \"\"; echo \"[TERMINE]\"; {end_action}'"
    subprocess.Popen(full_cmd, shell=True) # execution non bloquante
    gui_log(f"Terminal externe lancé : {title}", "INFO")

# param modbus pince zimmer 
IP_TURCK = '10.100.12.115'   # adresse IP du boitier passerelle Turck 
PORT_MODBUS = 502            # port TCP standard pour le protocole Modbus
ADDR_OUTPUTS = 2049          # adresse du registre d'écriture 
ADDR_INPUTS = 1              # adresse du registre de lecture 

# param modbus automate maitre
IP_PLC = '10.100.12.121'     # adresse IP de l'automate maitre
PORT_PLC = 502               # port TCP standard 
COIL_START = 0               # coil de lecture pour l'ordre de depart
COIL_END = 1                 # coil d'ecriture pour signaler la fin de cycle

# pris du constructeur Zimmer pour activer les fonctions
CW_TRANSFER = 1              # handshake initial
CW_RESET = 0                 # remise à zero de l'ordre
CW_OPEN = 256                # commande d'ouverture (Move to Base)
CW_CLOSE = 512               # commande de fermeture (Move to Work)
CW_ERROR_RESET = 32768       # acquittement des erreurs de la pince

# fonction de formatage des mots selon la documentation zimmer IO-Link
def build_process_data(control_word, grip_force=4, drive_velocity=4):
    device_mode = 103        # mode de fonctionnement standard
    workpiece_no = 0         # numéro de pièce 
    base_pos = 0             # position de base (ouverte)
    work_pos = 2000          # position de travail (fermee)

    # decalages binaires pour regrouper les octets
    reg0 = control_word
    reg1 = (device_mode << 8) | workpiece_no
    reg2 = (0 << 8) | 50   
    reg3 = (grip_force << 8) | drive_velocity
    reg4 = base_pos
    reg5 = 0               
    reg6 = 0               
    reg7 = work_pos

    # retourne les 8 registres constituant la trame d'ecriture complete
    return [reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7]

# PARAMÈTRES MATHÉMATIQUES & KINÉMATIQUES
# dimensions du robot Yaskawa HC10 et temps d'échantillonnage (ts)
param = {"L1": 0.275, "L2": 0.190, "L3": 0.700, "L4": 0.190, "L5": 0.500, "L6": 0.162, "L7": 0.237, "ts": 0.05}

# calcul de la matrice de transformation homogene 4x4 (Convention Denavit-Hartenberg modifiée)
def calculate_homogeneous_trans_matrix(dh_row):
    alpha = math.radians(dh_row[0])
    a = dh_row[1]
    theta = math.radians(dh_row[2])
    d = dh_row[3]
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    ct = math.cos(theta)
    st = math.sin(theta)
    return np.array([
        [ct,        -st,         0,    a],
        [st * ca,   ct * ca,    -sa,  -d * sa],
        [st * sa,   ct * sa,     ca,   d * ca],
        [0,         0,           0,    1]
    ])

# calcule la position/orientation du TCP
def calculate_MGD(DH):
    nb_joints = DH.shape[0]
    T_all = np.zeros((4, 4, nb_joints))
    T_cum = np.eye(4)
    for i in range(nb_joints):
        T_i = calculate_homogeneous_trans_matrix(DH[i, :])
        T_cum = np.dot(T_cum, T_i)
        T_all[:, :, i] = T_cum
    return T_all # on retourne les matrices pour chaque articulation

# table de DH pour le Yaskawa HC10
def mate3(q, param):
    DH=np.array([[0,0,q[0], param["L1"]]]) 
    DH=np.vstack((DH, [90,0,90+q[1], param["L2"]]))
    DH=np.vstack((DH, [180, param["L3"],-90+q[2],param["L4"]]))
    DH=np.vstack((DH, [90,0,q[3],param["L5"]]))
    DH=np.vstack((DH, [-90,0,q[4],param["L6"]]))
    DH=np.vstack((DH, [90,0,q[5],param["L7"]]))
    return DH

# relation entre vitesse articulaire et TCP
def calculate_jacobian(T, nb_joint):
    Jv = np.zeros((3, nb_joint))
    Jw = np.zeros((3, nb_joint))
    P_E = T[0:3, 3, -1] # position de l'effecteur (dernière colonne)
    for ii in range(nb_joint):
        p_i = T[0:3, 3, ii]
        z_i = T[0:3, 2, ii]
        Jv[:, ii] = np.cross(z_i, P_E - p_i) # produit vectoriel pour la composante lineaire
        Jw[:, ii] = z_i                      # composante angulaire
    return np.vstack((Jv, Jw)) # jacobienne 6xN

# vitesse en m/s
def vitesse_TCP(Jacob, dq):
    dX = np.dot(Jacob, dq)
    return dX, np.linalg.norm(dX[0:3]) 

# generation de traj
def calculate_poly_coeff(qi, qf, tf):
    Q = np.array([[qi], [0], [qf], [0]])
    X = np.array([[1, 0, 0, 0],  [0, 1, 0, 0], [1, tf, tf**2, tf**3], [0, 1, 2*tf, 3*tf**2]])
    return np.matmul(np.linalg.inv(X), Q).flatten()

# moveJ
def moveJ_base(qi, qf, tf, param):
    nb_elements = len(qi)
    nb_points = int(tf/param["ts"]) + 1
    q = np.empty((nb_elements, nb_points))
    dq = np.empty((nb_elements, nb_points)) 
    t = np.linspace(0, tf, nb_points)
    for i in range(nb_elements):
        coeff = calculate_poly_coeff(qi[i], qf[i], tf)
        q[i,:] = coeff[0] + coeff[1]*t + coeff[2]*t**2 + coeff[3]*t**3 # Position
        dq[i,:] = coeff[1] + 2*coeff[2]*t + 3*coeff[3]*t**2            # Vitesse
    return q, dq, t

# moveL
def moveL(pos_i, pos_f, tf, param):
    nb_elements = len(pos_i)
    nb_points = int(tf/param["ts"]) + 1
    pos = np.empty((nb_elements, nb_points))
    t = np.linspace(0, tf, nb_points)
    for i in range(nb_elements):
        coeff = calculate_poly_coeff(pos_i[i], pos_f[i], tf)
        pos[i,:] = coeff[0] + coeff[1]*t + coeff[2]*t**2 + coeff[3]*t**3
    return pos

# adapte le temps si la vitesse max est dépassée
def moveJ_adaptatif(qi, qf, tf, param):
    vmax_joint = np.deg2rad(100) # vitesse max par axe (rad/s)
    vmax_tcp = 0.25              # vitesse max spatiale du TCP (m/s)
    
    q, dq, t = moveJ_base(qi, qf, tf, param)
    v_joint_max = np.max(np.abs(dq))
    # ecrêtage articulaire
    if v_joint_max > vmax_joint:
        tf = tf * (v_joint_max / vmax_joint) * 1.05 
        q, dq, t = moveJ_base(qi, qf, tf, param)
        
    vitesse_du_TCP_max = 0
    # ecrêtage TCP 
    for k in range(q.shape[1]):
        Jacob = calculate_jacobian(calculate_MGD(mate3(np.rad2deg(q[:, k]), param)), len(qi))
        _, v_tcp = vitesse_TCP(Jacob, dq[:, k])
        if v_tcp > vitesse_du_TCP_max:
            vitesse_du_TCP_max = v_tcp
            
    if vitesse_du_TCP_max > vmax_tcp:
        tf = tf * (vitesse_du_TCP_max / vmax_tcp) * 1.05
        q, dq, t = moveJ_base(qi, qf, tf, param)
    return q, dq, t

# EXECUTION MATERIELLE ROS2 (COMMANDES SERVO UNIQUEMENT)
# pour allumer ou eteindre la puissance des moteurs du Yaskawa via les services ROS2
def set_left_servos(enable=True):
    if enable:
        # appel du service d'activation 
        cmd = "ros2 service call /yaskawa_LEFT/start_traj_mode motoros2_interfaces/srv/StartTrajMode \"{}\""
        launch_external_terminal("4 - SERVOS ON", cmd, keep_open=False)
    else:
        # appel du service de desactivation 
        cmd = "ros2 service call /yaskawa_LEFT/stop_traj_mode std_srvs/srv/Trigger \"{}\""
        launch_external_terminal("4 - SERVOS OFF", cmd, keep_open=False)

# CONFIGURATION IHM (THÈME & COULEURS)
# paramétrage global de l'interface customTkinter
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# constantes de couleurs pour l'interface
C_BG = "#0d0d0d"       # fond general tres sombre
C_PANEL = "#171717"    # fond des encadres (panels)
C_ACCENT = "#0055A4"   # bleu yaskawa
BTN_COLOR = "#1f2b38"  # couleur boutons inactifs
BTN_HOVER = "#0055A4"  # couleur boutons survolés

C_TEXT = "#e0e0e0"     # texte principal gris clair
C_OFF = "#2c3e50"      # voyant eteint (gris bleute)
C_ON = "#27ae60"       # voyant allumé (vert)
C_ALARM = "#c0392b"    # alarme/erreur (rouge)
C_WARN = "#f39c12"     # avertissement/mouvement (orange)

# pour créer un voyant rond (LED) avec un texte à côté
class IndicatorLight(ctk.CTkFrame):
    def __init__(self, master, text, active_color=C_ON, **kwargs):
        super().__init__(master, fg_color="transparent", **kwargs)
        self.active_color = active_color
        self.light = ctk.CTkFrame(self, width=12, height=12, corner_radius=6, fg_color=C_OFF)
        self.light.pack(side="left", padx=(0, 12))
        self.light.pack_propagate(False)
        self.label = ctk.CTkLabel(self, text=text, font=("Arial", 11, "bold"), text_color="#7f8c8d")
        self.label.pack(side="left")

    def set_state(self, state, error=False):
        if error:
            self.light.configure(fg_color=C_ALARM)
            self.label.configure(text_color=C_ALARM)
        elif state:
            self.light.configure(fg_color=self.active_color)
            self.label.configure(text_color="#ecf0f1")
        else:
            self.light.configure(fg_color=C_OFF)
            self.label.configure(text_color="#7f8c8d")

# pour afficher les graphiques en grand format
class LargeGraphWindow(ctk.CTkToplevel):
    def __init__(self, master, app, mode="joints"):
        super().__init__(master)
        self.app = app
        self.mode = mode
        self.title("ANALYSE DETAILLEE")
        self.geometry("900x700")
        self.configure(fg_color=C_BG)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # choix de l'affichage : les axes en 2d (joints) ou la traj spatiale 3d (TCP)
        if mode == "joints":
            self.app.large_graph_window = self 
            self.canvas = tk.Canvas(self, bg="#050505", highlightthickness=0)
            self.canvas.pack(fill="both", expand=True, padx=20, pady=20)
            self.canvas.bind("<Configure>", self.draw_graph)
        else:
            self.app.large_tcp_window = self
            self.fig = Figure(figsize=(8, 6), facecolor=C_BG)
            self.fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.ax.set_facecolor(C_BG)
            self.ax.tick_params(colors='#7f8c8d')
            for axis in [self.ax.xaxis, self.ax.yaxis, self.ax.zaxis]:
                axis.label.set_color('#7f8c8d')
                axis.pane.fill = False
            self.ax.grid(color='#333333')
            self.ax.set_xlim([-1.0, 1.0])
            self.ax.set_ylim([-1.0, 1.0])
            self.ax.set_zlim([0.0, 1.5])
            
            # init des traces matplotlib vides
            self.line_3d, = self.ax.plot([], [], [], color=C_ACCENT, linewidth=2)
            self.scatter_start = self.ax.scatter([], [], [], color=C_ON, s=60)
            self.scatter_end = self.ax.scatter([], [], [], color=C_ALARM, s=60)
            
            # insertion du graphique matplotlib dans Tykinter
            self.canvas_3d = FigureCanvasTkAgg(self.fig, master=self)
            self.canvas_3d.get_tk_widget().pack(fill="both", expand=True, padx=20, pady=20)
            
    def on_close(self):
        if self.mode == "joints": self.app.large_graph_window = None
        else: self.app.large_tcp_window = None
        self.destroy() # fermet propre de la sous-fenêtre
        
    def draw_graph(self, event=None):
        if self.mode == "joints":
            self.app.draw_joints_logic(self.canvas, self.app.live_q_history, is_mini=False)
            
    def update_tcp_3d(self, history):
        if not history or len(history) < 2: return
        pts = np.array(history)
        # met à jour les donnees de la ligne 3d avec l'historique des positions cartesiennes
        self.line_3d.set_data(pts[:, 0], pts[:, 1])
        self.line_3d.set_3d_properties(pts[:, 2])
        # mise a jour des points de depart (vert) et d'arrivee (rouge)
        self.scatter_start._offsets3d = ([pts[0, 0]], [pts[0, 1]], [pts[0, 2]])
        self.scatter_end._offsets3d = ([pts[-1, 0]], [pts[-1, 1]], [pts[-1, 2]])
        self.canvas_3d.draw_idle()

# PAGES DE L'IHM avec creation des onglets
# page 1
class PageOverview(ctk.CTkFrame):
    def __init__(self, master, app):
        super().__init__(master, fg_color="transparent")
        self.app = app
        self.grid_columnconfigure((0, 1, 2), weight=1)
        self.grid_rowconfigure(1, weight=1)

        # ligne du haut 
        kpi_frame = ctk.CTkFrame(self, fg_color="transparent")
        kpi_frame.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 10))
        self.card_ok = self.create_kpi_card(kpi_frame, "PIECES CONFORMES", "0", C_ON)
        self.card_ok.pack(side="left", expand=True, padx=5)
        self.card_nok = self.create_kpi_card(kpi_frame, "PIECES DEFAUT", "0", C_ALARM)
        self.card_nok.pack(side="left", expand=True, padx=5)

        # colonne de gauche 
        q1 = ctk.CTkFrame(self, fg_color=C_PANEL, corner_radius=8)
        q1.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        ctk.CTkLabel(q1, text="STATUT GLOBAL", font=("Arial", 12, "bold"), text_color=C_ACCENT).pack(pady=15)
        
        self.led_sys = IndicatorLight(q1, "SYSTEME PRET", C_ON)
        self.led_sys.pack(pady=5, padx=30, anchor="w")
        self.led_secu = IndicatorLight(q1, "SECURITES VALIDEES", C_ON)
        self.led_secu.pack(pady=5, padx=30, anchor="w")
        self.led_modbus = IndicatorLight(q1, "LIAISON MODBUS", C_ON)
        self.led_modbus.pack(pady=5, padx=30, anchor="w")
        
        self.led_sys.set_state(True)
        self.led_secu.set_state(True)
        self.led_modbus.set_state(False) # S'allumera si Modbus réussit
        
        ctk.CTkFrame(q1, height=2, fg_color="#333").pack(fill="x", padx=20, pady=15)
        ctk.CTkLabel(q1, text="VITESSE TCP DIRECTE", font=("Arial", 10, "bold"), text_color="#7f8c8d").pack(pady=(5, 0))
        self.lbl_v_tcp = ctk.CTkLabel(q1, text="0.000 m/s", font=("Consolas", 28, "bold"), text_color=C_ON)
        self.lbl_v_tcp.pack(pady=5)

        # Colonnes du milieu/droite : Graphique 3D Matplotlib intégré
        q2 = ctk.CTkFrame(self, fg_color=C_PANEL, corner_radius=8)
        q2.grid(row=1, column=1, columnspan=2, sticky="nsew", padx=5, pady=5)
        
        ctk.CTkLabel(q2, text="TRAJECTOIRE SPATIALE DU TCP (3D)", font=("Arial", 12, "bold"), text_color=C_ACCENT).pack(pady=(15, 0))
        
        self.fig = Figure(figsize=(5, 4), facecolor=C_PANEL)
        self.fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor(C_PANEL)
        self.ax.tick_params(colors='#7f8c8d', labelsize=8)
        for axis in [self.ax.xaxis, self.ax.yaxis, self.ax.zaxis]:
            axis.label.set_color('#7f8c8d')
            axis.pane.fill = False
        self.ax.grid(color='#333333')
        
        self.ax.set_xlim([-1.0, 1.0])
        self.ax.set_ylim([-1.0, 1.0])
        self.ax.set_zlim([0.0, 1.5])
        
        self.line_3d, = self.ax.plot([], [], [], color=C_ACCENT, linewidth=2, alpha=0.8)
        self.scatter_start = self.ax.scatter([], [], [], color=C_ON, s=40)
        self.scatter_end = self.ax.scatter([], [], [], color=C_ALARM, s=40)
        
        self.canvas_3d = FigureCanvasTkAgg(self.fig, master=q2)
        self.canvas_widget = self.canvas_3d.get_tk_widget()
        self.canvas_widget.pack(fill="both", expand=True, padx=10, pady=10)
        # Clic sur le graphique pour l'agrandir en popup
        self.canvas_widget.bind("<Button-1>", lambda e: LargeGraphWindow(self.app, self.app, mode="tcp_3d"))

    # generation d'une case KPI 
    def create_kpi_card(self, parent, title, value, color):
        frame = ctk.CTkFrame(parent, fg_color=C_PANEL, corner_radius=8, height=80)
        frame.pack_propagate(False)
        ctk.CTkLabel(frame, text=title, font=("Arial", 10, "bold"), text_color="#7f8c8d").pack(pady=(12, 0))
        lbl_val = ctk.CTkLabel(frame, text=value, font=("Consolas", 28, "bold"), text_color=color)
        lbl_val.pack()
        frame.lbl_val = lbl_val
        return frame

    def update_kpi(self, ok, nok):
        self.card_ok.lbl_val.configure(text=str(ok))
        self.card_nok.lbl_val.configure(text=str(nok))

    # mise a jour visuelle de la vitesse avec alerte si > 0.25 m/s
    def update_tcp_speed(self, v_tcp):
        self.lbl_v_tcp.configure(text=f"{v_tcp:.3f} m/s", text_color=C_ALARM if v_tcp > 0.25 else C_ON)
        
    def update_tcp_3d(self, history):
        if not history or len(history) < 2: return
        pts = np.array(history)
        self.line_3d.set_data(pts[:, 0], pts[:, 1])
        self.line_3d.set_3d_properties(pts[:, 2])
        self.scatter_start._offsets3d = ([pts[0, 0]], [pts[0, 1]], [pts[0, 2]])
        self.scatter_end._offsets3d = ([pts[-1, 0]], [pts[-1, 1]], [pts[-1, 2]])
        self.canvas_3d.draw_idle()

# page 2
class PageControl(ctk.CTkFrame):
    def __init__(self, master, app):
        super().__init__(master, fg_color="transparent")
        self.app = app
        self.grid_columnconfigure((0, 1, 2), weight=1)
        self.grid_rowconfigure(0, weight=1)

        # colonne de gauche : Visualisation, Servos, Pince et Boutons CSV
        col1 = ctk.CTkFrame(self, fg_color=C_PANEL, corner_radius=8)
        col1.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        ctk.CTkLabel(col1, text="VISUALISATION CELLULE", font=("Arial", 12, "bold"), text_color=C_ACCENT).pack(pady=10)
        try:
            img = Image.open(resource_path("yaskawa.png")) # Chargement du logo
            logo = ctk.CTkImage(light_image=img, dark_image=img, size=(160, 160))
            ctk.CTkLabel(col1, image=logo, text="").pack(pady=5)
        except:
            ctk.CTkLabel(col1, text="[ IMAGE NON TROUVEE ]", font=("Consolas", 10), fg_color="#222", width=160, height=160).pack(pady=5)
            
        self.led_pince = IndicatorLight(col1, "OUTIL ACTIONNE", C_ACCENT)
        self.led_pince.pack(pady=5)

        frame_csv = ctk.CTkFrame(col1, fg_color="#111", corner_radius=6)
        frame_csv.pack(fill="x", padx=15, pady=(15, 10))
        ctk.CTkLabel(frame_csv, text="COMMANDES MATERIELLES", font=("Arial", 10, "bold"), text_color="#7f8c8d").pack(pady=(5,2))
        
        # boutons de contrôle direct des servos ROS2
        servo_grid = ctk.CTkFrame(frame_csv, fg_color="transparent")
        servo_grid.pack(fill="x", padx=5, pady=(2, 4))
        servo_grid.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkButton(servo_grid, text="SERVOS ON", height=25, fg_color=BTN_COLOR, hover_color=BTN_HOVER, command=lambda: threading.Thread(target=set_left_servos, args=(True,), daemon=True).start()).grid(row=0, column=0, padx=2, pady=2, sticky="ew")
        ctk.CTkButton(servo_grid, text="SERVOS OFF", height=25, fg_color=BTN_COLOR, hover_color=BTN_HOVER, command=lambda: threading.Thread(target=set_left_servos, args=(False,), daemon=True).start()).grid(row=0, column=1, padx=2, pady=2, sticky="ew")

        # boutons de contrôle direct de la pince Modbus tcp
        pince_grid = ctk.CTkFrame(frame_csv, fg_color="transparent")
        pince_grid.pack(fill="x", padx=5, pady=(2, 4))
        pince_grid.grid_columnconfigure((0, 1), weight=1)
        self.btn_open = ctk.CTkButton(pince_grid, text="OUVRIR PINCE", height=25, fg_color=BTN_COLOR, hover_color=BTN_HOVER, command=lambda: threading.Thread(target=self.app.modbus_command_sync, args=(CW_OPEN, True)).start())
        self.btn_open.grid(row=0, column=0, padx=2, pady=2, sticky="ew")
        self.btn_close = ctk.CTkButton(pince_grid, text="FERMER PINCE", height=25, fg_color=BTN_COLOR, hover_color=BTN_HOVER, command=lambda: threading.Thread(target=self.app.modbus_command_sync, args=(CW_CLOSE,)).start())
        self.btn_close.grid(row=0, column=1, padx=2, pady=2, sticky="ew")

        # grille de la machine à etats (bloque/débloque les csv sequentiellement)
        ctk.CTkLabel(frame_csv, text="TRAJECTOIRES CSV (MANUEL)", font=("Arial", 10, "bold"), text_color="#7f8c8d").pack(pady=(10,2))
        btn_grid = ctk.CTkFrame(frame_csv, fg_color="transparent")
        btn_grid.pack(fill="x", padx=5, pady=(0, 5))
        btn_grid.grid_columnconfigure((0, 1), weight=1)
        
        self.btn_csv1 = ctk.CTkButton(btn_grid, text="PART 1", height=25, fg_color=BTN_COLOR, hover_color=BTN_HOVER, command=lambda: self.app.run_csv("trajectoire_rad_RV_PART1.csv"))
        self.btn_csv1.grid(row=0, column=0, padx=2, pady=2, sticky="ew")
        self.btn_csv2 = ctk.CTkButton(btn_grid, text="PART 2", height=25, fg_color=BTN_COLOR, hover_color=BTN_HOVER, state="disabled", command=lambda: self.app.run_csv("trajectoire_rad_RV_PART2.csv"))
        self.btn_csv2.grid(row=0, column=1, padx=2, pady=2, sticky="ew")
        self.btn_csv3_oui = ctk.CTkButton(btn_grid, text="PART 3 (OUI)", height=25, fg_color=BTN_COLOR, hover_color=BTN_HOVER, state="disabled", command=lambda: self.app.run_csv("trajectoire_rad_RV_PART3_oui.csv"))
        self.btn_csv3_oui.grid(row=1, column=0, padx=2, pady=2, sticky="ew")
        self.btn_csv4_oui = ctk.CTkButton(btn_grid, text="PART 4 (OUI)", height=25, fg_color=BTN_COLOR, hover_color=BTN_HOVER, state="disabled", command=lambda: self.app.run_csv("trajectoire_rad_RV_PART4_oui.csv"))
        self.btn_csv4_oui.grid(row=1, column=1, padx=2, pady=2, sticky="ew")
        self.btn_csv3_non = ctk.CTkButton(btn_grid, text="PART 3 (NON)", height=25, fg_color=BTN_COLOR, hover_color=BTN_HOVER, state="disabled", command=lambda: self.app.run_csv("trajectoire_rad_RV_PART3_non.csv"))
        self.btn_csv3_non.grid(row=2, column=0, padx=2, pady=2, sticky="ew")
        self.btn_csv4_non = ctk.CTkButton(btn_grid, text="PART 4 (NON)", height=25, fg_color=BTN_COLOR, hover_color=BTN_HOVER, state="disabled", command=lambda: self.app.run_csv("trajectoire_rad_RV_PART4_non.csv"))
        self.btn_csv4_non.grid(row=2, column=1, padx=2, pady=2, sticky="ew")

        # colonne centrale : contrôle mode de fonctionnement et alertes
        col2 = ctk.CTkFrame(self, fg_color=C_PANEL, corner_radius=8)
        col2.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        ctk.CTkLabel(col2, text="CONTROLE OPERATIONNEL", font=("Arial", 12, "bold"), text_color=C_ACCENT).pack(pady=15)
        
        # selecteur de modes (Local, Simulation ROS 2, Manuel Réel, Automatique Réel)
        self.seg_mode = ctk.CTkSegmentedButton(col2, values=["MODE LOCAL", "MODE SIMULATION", "MODE REEL", "MODE REEL AUTO"], command=self.change_mode, selected_color=C_ACCENT, selected_hover_color="#003d7a")
        self.seg_mode.pack(fill="x", padx=30, pady=(0, 10))
        self.seg_mode.set("MODE LOCAL")
        
        self.led_mvt = IndicatorLight(col2, "ROBOT EN MOUVEMENT", C_WARN)
        self.led_mvt.pack(pady=10)
        
        ctk.CTkLabel(col2, text="ETAT DU PROCESSUS", font=("Arial", 10, "bold"), text_color="#7f8c8d").pack(pady=(15, 0))
        self.lbl_status = ctk.CTkLabel(col2, text="ATTENTE DEPART", font=("Arial", 18, "bold"), text_color="#bdc3c7")
        self.lbl_status.pack(pady=5)
        
        # bouton qui lance soit la simulation soit le cycle manuel (en mode AUTO il devient un indicateur API)
        self.btn_start = ctk.CTkButton(col2, text="DEMARRER CYCLE", height=45, font=("Arial", 12, "bold"), fg_color=C_ACCENT, hover_color="#003d7a", corner_radius=4, command=self.start_cycle)
        self.btn_start.pack(fill="x", padx=30, pady=(15, 5))

        # --- CURSEUR DE VITESSE (Uniquement pour MODE REEL AUTO) ---
        self.lbl_speed = ctk.CTkLabel(col2, text="VITESSE CYCLE AUTO : 100%", font=("Arial", 10, "bold"), text_color="#7f8c8d")
        self.lbl_speed.pack(pady=(10, 0))
        self.slider_speed = ctk.CTkSlider(col2, from_=0.1, to=1.0, number_of_steps=90, command=self.update_speed, progress_color=C_ACCENT, button_color=C_ACCENT)
        self.slider_speed.set(1.0)
        self.slider_speed.pack(fill="x", padx=30, pady=(0, 15))
        self.slider_speed.configure(state="disabled")
        # ---------------------------------------------------------------------
        
        # zone de validation pour simuler le contrôle qualite caméra
        frame_insp = ctk.CTkFrame(col2, fg_color="#111", corner_radius=6)
        frame_insp.pack(fill="x", padx=20, pady=15)
        ctk.CTkLabel(frame_insp, text="VALIDATION QUALITE", font=("Arial", 10, "bold"), text_color="#7f8c8d").pack(pady=(10,5))
        
        self.btn_oui = ctk.CTkButton(frame_insp, text="PIECE CONFORME", fg_color=BTN_COLOR, hover_color=BTN_HOVER, text_color="#fff", font=("Arial", 11, "bold"), corner_radius=4, state="disabled", command=lambda: self.app.set_inspection("oui"))
        self.btn_oui.pack(fill="x", padx=20, pady=5)
        self.btn_non = ctk.CTkButton(frame_insp, text="PIECE DEFAUT", fg_color=BTN_COLOR, hover_color=BTN_HOVER, text_color="#fff", font=("Arial", 11, "bold"), corner_radius=4, state="disabled", command=lambda: self.app.set_inspection("non"))
        self.btn_non.pack(fill="x", padx=20, pady=(5,15))

        # arret d'urgence logiciel
        self.btn_arret = ctk.CTkButton(col2, text="ARRET LOGICIEL", height=45, font=("Arial", 12, "bold"), fg_color=C_ALARM, hover_color="#a93226", corner_radius=4, command=self.stop_logiciel)
        self.btn_arret.pack(side="bottom", fill="x", padx=20, pady=20)

        # colonne de droite : télémetrie et graphique
        col3 = ctk.CTkFrame(self, fg_color=C_PANEL, corner_radius=8)
        col3.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
        ctk.CTkLabel(col3, text="TELEMETRIE AXES", font=("Arial", 12, "bold"), text_color=C_ACCENT).pack(pady=15)
        
        frame_jauges = ctk.CTkFrame(col3, fg_color="transparent")
        frame_jauges.pack(fill="x", padx=20, pady=5)
        self.jauges_q = []
        self.labels_q = []
        
        # creation des 6 jauges pour les articulations J1 a J6
        self.axe_colors = ["#e74c3c", "#3498db", "#2ecc71", "#f1c40f", "#9b59b6", "#e67e22"]
        for i in range(6):
            row_f = ctk.CTkFrame(frame_jauges, fg_color="transparent", height=20)
            row_f.pack(fill="x", pady=2)
            ctk.CTkLabel(row_f, text=f"J{i+1}", font=("Arial", 11, "bold"), width=25, text_color=self.axe_colors[i]).pack(side="left")
            pb = ctk.CTkProgressBar(row_f, progress_color=self.axe_colors[i], fg_color="#222", height=8, corner_radius=0)
            pb.pack(side="left", fill="x", expand=True, padx=10)
            pb.set(0.5) 
            lbl_val = ctk.CTkLabel(row_f, text="0.0°", font=("Consolas", 11), width=50, anchor="e")
            lbl_val.pack(side="right")
            self.jauges_q.append(pb)
            self.labels_q.append(lbl_val)

        # trace historique des angles
        frame_mini_g = ctk.CTkFrame(col3, fg_color="transparent")
        frame_mini_g.pack(fill="both", expand=True, padx=20, pady=(15, 20))
        ctk.CTkLabel(frame_mini_g, text="TRAJECTOIRES ARTICULAIRES", font=("Arial", 10, "bold"), text_color="#7f8c8d").pack()
        
        self.cv_joints_mini = tk.Canvas(frame_mini_g, bg="#050505", highlightthickness=0, cursor="hand2")
        self.cv_joints_mini.pack(fill="both", expand=True, pady=(5, 0))
        self.cv_joints_mini.bind("<Configure>", lambda e: self.app.draw_joints_logic(self.cv_joints_mini, self.app.live_q_history, is_mini=True))
        self.cv_joints_mini.bind("<Button-1>", lambda e: LargeGraphWindow(self.app, self.app, mode="joints"))

    # Mise à jour du libellé de vitesse et de la variable globale
    def update_speed(self, value):
        self.app.speed_factor = value
        self.lbl_speed.configure(text=f"VITESSE CYCLE AUTO : {int(value*100)}%")

    # execution de l'arret logiciel 
    def stop_logiciel(self):
        gui_log("ATTENTION : DEMANDE D'ARRET LOGICIEL DECLENCHEE", "ERROR")
        self.lbl_status.configure(text="ARRET LOGICIEL", text_color=C_ALARM)
        self.app.cycle_started = False
        self.app.auto_cycle_active = False # casse la boucle du cycle automatique si en cours
        
        if self.app.op_mode in ["MODE REEL", "MODE REEL AUTO", "MODE SIMULATION"]:
            # on force la destruction du script python qiu execute la traj en simulant un Ctrl+C
            subprocess.Popen("pkill -INT -f execute_trajectory", shell=True)
            gui_log("Trajectoire interceptée par pkill.", "WARN")
            
            # on envoie l'ordre de coupure directe aux moteurs 
            setup_env = "source /opt/ros/humble/setup.bash 2>/dev/null; source ~/colcon_ws/install/setup.bash 2>/dev/null;"
            cmd_stop = f"bash -c '{setup_env} ros2 service call /yaskawa_LEFT/stop_traj_mode std_srvs/srv/Trigger \"{{}}\"'"
            subprocess.Popen(cmd_stop, shell=True)
            gui_log("Coupure des servos envoyée à la baie (si matériel connecté).", "WARN")

    # gestion de la bascule entre les 4 modes de l'ihm
    def change_mode(self, value):
        self.app.op_mode = value
        
        # Désactiver le slider par défaut, on ne l'active que pour REEL AUTO
        self.slider_speed.configure(state="disabled", progress_color="#555", button_color="#555")
        
        if value == "MODE LOCAL":
            self.btn_start.configure(state="normal", text="DEMARRER CYCLE", fg_color=C_ACCENT)
            self.app.advance_real_sequence(0) # grise les boutons CSV
            
        elif value in ["MODE REEL", "MODE SIMULATION"]:
            self.btn_start.configure(state="disabled", text="UTILISEZ LES BOUTONS CSV ->", fg_color="#555")
            self.app.advance_real_sequence(1) 
            
            if value == "MODE SIMULATION":
                gui_log("Initialisation du Mode Simulation ROS 2 (RViz)...", "WARN")
                launch_external_terminal("1 - RVIZ SIMULATION", "ros2 launch motoman_hc10_moveit_config demo_sim.launch.py robot:=left", keep_open=True)
            else:
                gui_log("Initialisation du Mode Réel... Lancement des terminaux.", "WARN")
                launch_external_terminal("1 - DOCKER MICRO-ROS", "runmicrorosdocker", keep_open=True)
                time.sleep(0.5)
                launch_external_terminal("2 - DEBUG LISTENER", "ros2 run mfja_motoman_hc10 motoros2_debug_listener.py", keep_open=True)
                time.sleep(0.5)
                launch_external_terminal("3 - VERIFICATION NOEUDS ET TOPICS", "ros2 node list && echo '---' && ros2 topic echo /yaskawa_LEFT/joint_states --once", keep_open=True)
                
        elif value == "MODE REEL AUTO":
            # Le bouton manuel est désactivé et indique l'attente de l'automate
            self.btn_start.configure(state="disabled", text="ATTENTE API (10.100.12.121)", fg_color="#e67e22")
            self.app.advance_real_sequence(0) # grise les boutons CSV
            self.slider_speed.configure(state="normal", progress_color=C_ON, button_color=C_ON) # On réactive le curseur de vitesse
            
            gui_log(f"Mode Réel Auto sélectionné : En attente du signal de l'automate (Coil {COIL_START}).", "WARN")
            launch_external_terminal("1 - DOCKER MICRO-ROS", "runmicrorosdocker", keep_open=True)
            time.sleep(0.5)
            launch_external_terminal("2 - DEBUG LISTENER", "ros2 run mfja_motoman_hc10 motoros2_debug_listener.py", keep_open=True)
            time.sleep(0.5)
            launch_external_terminal("3 - VERIFICATION NOEUDS ET TOPICS", "ros2 node list && echo '---' && ros2 topic echo /yaskawa_LEFT/joint_states --once", keep_open=True)

    def start_cycle(self):
        # Désormais utilisé uniquement pour la simulation logicielle/locale
        if self.app.op_mode == "MODE LOCAL":
            self.app.cycle_started = True 

    # met a jour les jauges visuelles avec les nouvelles donnees télémétriques
    def update_telemetrie(self, q_rad):
        for i in range(min(6, len(q_rad))):
            deg = math.degrees(q_rad[i])
            ratio = max(0.0, min(1.0, (deg + 180) / 360.0))
            self.jauges_q[i].set(ratio)
            self.labels_q[i].configure(text=f"{deg:+.1f}°")
        
        self.app.live_q_history.append(q_rad)
        if len(self.app.live_q_history) > 200:
            self.app.live_q_history.pop(0)
            
        self.app.draw_joints_logic(self.cv_joints_mini, self.app.live_q_history, is_mini=True)

# page 3
class PageGripper(ctk.CTkFrame):
    def __init__(self, master, app):
        super().__init__(master, fg_color="transparent")
        self.app = app
        self.grid_columnconfigure((0, 1, 2), weight=1)
        self.grid_rowconfigure(0, weight=1)
        
        # Colonne de Paramètres Modbus (Force/Vitesse)
        col1 = ctk.CTkFrame(self, fg_color=C_PANEL, corner_radius=8)
        col1.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        ctk.CTkLabel(col1, text="PARAMETRES PINCE", font=("Arial", 12, "bold"), text_color=C_ACCENT).pack(pady=15)
        
        self.led_pince_conn = IndicatorLight(col1, "LIAISON MODBUS PINCE", C_ON)
        self.led_pince_conn.pack(pady=(0, 20))
        self.led_pince_conn.set_state(False)

        ctk.CTkLabel(col1, text="FORCE DE SERRAGE (1-4)", font=("Arial", 10, "bold"), text_color="#7f8c8d").pack(pady=(10, 5))
        self.seg_force = ctk.CTkSegmentedButton(col1, values=["1", "2", "3", "4"], selected_color=C_ACCENT, selected_hover_color="#003d7a")
        self.seg_force.pack(fill="x", padx=30, pady=5)
        self.seg_force.set("4")
        
        ctk.CTkLabel(col1, text="VITESSE MOTEUR (1-4)", font=("Arial", 10, "bold"), text_color="#7f8c8d").pack(pady=(30, 5))
        self.seg_speed = ctk.CTkSegmentedButton(col1, values=["1", "2", "3", "4"], selected_color=C_ACCENT, selected_hover_color="#003d7a")
        self.seg_speed.pack(fill="x", padx=30, pady=5)
        self.seg_speed.set("4")
        
        # colonne commandes directes modbus
        col2 = ctk.CTkFrame(self, fg_color=C_PANEL, corner_radius=8)
        col2.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        ctk.CTkLabel(col2, text="COMMANDES MANUELLES", font=("Arial", 12, "bold"), text_color=C_ACCENT).pack(pady=15)
        
        self.btn_init = ctk.CTkButton(col2, text="INITIALISER (HANDSHAKE)", height=45, fg_color=BTN_COLOR, hover_color=BTN_HOVER, command=self.app.modbus_init_pince)
        self.btn_init.pack(fill="x", padx=40, pady=(20, 10))
        
        self.btn_open = ctk.CTkButton(col2, text="OUVRIR (MOVE TO BASE)", height=45, fg_color=BTN_COLOR, hover_color=BTN_HOVER, command=self.app.modbus_open_pince)
        self.btn_open.pack(fill="x", padx=40, pady=10)
        
        self.btn_close = ctk.CTkButton(col2, text="FERMER (MOVE TO WORK)", height=45, fg_color=BTN_COLOR, hover_color=BTN_HOVER, command=self.app.modbus_close_pince)
        self.btn_close.pack(fill="x", padx=40, pady=10)
        
        self.btn_reset = ctk.CTkButton(col2, text="ACQUITTER ERREUR", height=45, fg_color=C_ALARM, hover_color="#a93226", command=self.app.modbus_reset_error)
        self.btn_reset.pack(fill="x", padx=40, pady=(30, 10))

        # colonne lecture en direct des informations pince
        col3 = ctk.CTkFrame(self, fg_color=C_PANEL, corner_radius=8)
        col3.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
        ctk.CTkLabel(col3, text="ETAT EN TEMPS REEL", font=("Arial", 12, "bold"), text_color=C_ACCENT).pack(pady=15)
        
        self.lbl_status = ctk.CTkLabel(col3, text="StatusWord: INCONNU", font=("Consolas", 14, "bold"), text_color="#bdc3c7")
        self.lbl_status.pack(pady=10)
        
        self.lbl_diag = ctk.CTkLabel(col3, text="Code Erreur: AUCUN", font=("Consolas", 14, "bold"), text_color=C_ON)
        self.lbl_diag.pack(pady=10)
        
        self.lbl_pos = ctk.CTkLabel(col3, text="Position: - mm", font=("Consolas", 14, "bold"), text_color="#bdc3c7")
        self.lbl_pos.pack(pady=10)
        
        self.txt_raw = ctk.CTkTextbox(col3, font=("Consolas", 11), fg_color="#0a0a0a", text_color="#7f8c8d", corner_radius=6)
        self.txt_raw.pack(fill="both", expand=True, padx=20, pady=(20, 20))
        self.txt_raw.insert("end", "En attente de la première commande...\nLa lecture se fera automatiquement.")
        self.txt_raw.configure(state="disabled")

    # fonction appelee pour rafraichir l'ecran de la pince apres une trame modbus
    def update_state_display(self, status, diag, pos, raw_text, has_error):
        self.lbl_status.configure(text=f"StatusWord: {hex(status)}")
        
        if has_error and diag != 0:
            self.lbl_diag.configure(text=f"Code Erreur: {hex(diag)}", text_color=C_ALARM)
        else:
            self.lbl_diag.configure(text=f"Code Erreur: OK (0x0)", text_color=C_ON)
            
        self.lbl_pos.configure(text=f"Position: {pos / 100.0:.2f} mm")
        
        self.txt_raw.configure(state="normal")
        self.txt_raw.delete("1.0", "end")
        self.txt_raw.insert("end", raw_text)
        self.txt_raw.configure(state="disabled")

# page 4 
class PageLogs(ctk.CTkFrame):
    def __init__(self, master, app):
        super().__init__(master, fg_color=C_PANEL, corner_radius=8)
        self.app = app
        header = ctk.CTkFrame(self, fg_color="transparent")
        header.pack(fill="x", padx=20, pady=15)
        ctk.CTkLabel(header, text="JOURNAL DES EVENEMENTS SYSTEME (LOGS)", font=("Arial", 14, "bold"), text_color=C_ACCENT).pack(side="left")
        
        self.txt_logs = ctk.CTkTextbox(self, font=("Consolas", 12), fg_color="#0a0a0a", text_color="#bdc3c7", corner_radius=4)
        self.txt_logs.pack(fill="both", expand=True, padx=20, pady=10)
        self.txt_logs.insert("end", "En attente des messages systèmes...\n")
        self.txt_logs.configure(state="disabled")

# IHM PRINCIPALE
class YaskawaDashboard(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("SUPERVISION YASKAWA HC10")
        self.geometry("1300x800")
        self.configure(fg_color=C_BG)

        # variables d'etat global du programme
        self.op_mode = "MODE LOCAL" 
        self.cycle_started = False
        self.auto_cycle_active = False # ttemoin d'activite pour le cycle automatique materiel
        
        self.live_q_history = []
        self.live_tcp_history = []
        
        self.inspection_result = None
        self.auto_inspection_result = None # variable dédiee à la decision dans le cycle automatique
        self.kpi_ok = 0
        self.kpi_nok = 0
        
        self.large_graph_window = None
        self.large_tcp_window = None
        self._last_modbus_state = None
        
        self.current_real_step = 1
        self.speed_factor = 1.0  

        # entête graphique
        self.header = ctk.CTkFrame(self, height=60, fg_color=C_PANEL, corner_radius=0)
        self.header.pack(fill="x", side="top")
        self.header.pack_propagate(False)
        self.lbl_title = ctk.CTkLabel(self.header, text="", font=("Arial", 16, "bold"), text_color=C_ACCENT)
        self.lbl_title.pack(side="left", padx=20)
        ctk.CTkLabel(self.header, text="YASKAWA HC10 - SYSTEME DE CONTROLE", font=("Arial", 11, "bold"), text_color="#7f8c8d").pack(side="right", padx=20)

        #conteneur principal pour changer de page dynamiquement
        self.container = ctk.CTkFrame(self, fg_color="transparent")
        self.container.pack(fill="both", expand=True, padx=20, pady=10)

        # les 4 pages 
        self.pages = [
            PageOverview(self.container, self),
            PageControl(self.container, self),
            PageGripper(self.container, self),
            PageLogs(self.container, self)
        ]
        self.page_names = ["VUE GLOBALE", "PUPITRE DE COMMANDE", "CONTROLE PINCE", "JOURNAL DES EVENEMENTS"]
        self.current_idx = 0

        # pied de page pour la navigation
        self.footer = ctk.CTkFrame(self, height=50, fg_color="transparent")
        self.footer.pack(fill="x", side="bottom")
        ctk.CTkButton(self.footer, text="PRECEDENT", width=120, fg_color=BTN_COLOR, hover_color=BTN_HOVER, font=("Arial", 11, "bold"), command=self.prev_page).pack(side="left", padx=20)
        self.lbl_dots = ctk.CTkLabel(self.footer, text="PAGE 1 / 4", font=("Arial", 12, "bold"), text_color="#7f8c8d")
        self.lbl_dots.pack(side="left", expand=True)
        ctk.CTkButton(self.footer, text="SUIVANT", width=120, fg_color=BTN_COLOR, hover_color=BTN_HOVER, font=("Arial", 11, "bold"), command=self.next_page).pack(side="right", padx=20)

        # caccourcis clavier
        self.bind("<Right>", self.next_page)
        self.bind("<Left>", self.prev_page)
        self.show_page(0)
        
        #  surveillance du Modbus TCP 
        threading.Thread(target=self.plc_monitor_thread, daemon=True).start()

    # surveillance de l'automate via les coils
    def plc_monitor_thread(self):
        """ Scanne en permanence le Coil 0 de l'API lorsque le mode auto est activé """
        while True:
            time.sleep(0.5) 
            if self.op_mode == "MODE REEL AUTO" and not self.auto_cycle_active:
                try:
                    client = ModbusTcpClient(IP_PLC, port=PORT_PLC, timeout=1.0)
                    if client.connect():
                        res = client.read_coils(address=COIL_START, count=1)
                        if not res.isError() and res.bits and res.bits[0] is True:
                            gui_log(f"Signal de départ API détecté (Coil {COIL_START} = True). Lancement du cycle auto.", "SUCCESS")
                            self.auto_cycle_active = True
                            
                            # on modifie l'interface visuelle
                            self.after(0, lambda: self.pages[1].btn_start.configure(text="CYCLE API EN COURS", fg_color=C_ALARM))
                            
                            # demarre l'execution
                            threading.Thread(target=self.auto_cycle_thread, daemon=True).start()
                        client.close()
                except Exception as e:
                    pass

    def signal_end_to_plc(self):
        """ Écrit True dans le Coil de fin de cycle, attend 10s, et écrit False """
        def task():
            try:
                client = ModbusTcpClient(IP_PLC, port=PORT_PLC, timeout=1.0)
                if client.connect():
                    gui_log(f"Envoi du signal de fin de cycle à l'API (Coil {COIL_END} = True)...", "INFO")
                    client.write_coil(address=COIL_END, value=True)
                    
                    time.sleep(10.0) 
                    
                    gui_log(f"Remise à zéro du signal de fin de cycle API (Coil {COIL_END} = False).", "INFO")
                    client.write_coil(address=COIL_END, value=False)
                    client.close()
                else:
                    gui_log(f"Echec de connexion à l'API ({IP_PLC}) pour signaler la fin.", "ERROR")
            except Exception as e:
                gui_log(f"Erreur Modbus avec l'API maître: {e}", "ERROR")
        
        threading.Thread(target=task, daemon=True).start()

    # reception du choix de l'operateur (oui / non)l'inspection visuelle
    def set_inspection(self, decision):
        self.pages[1].btn_oui.configure(state="disabled")
        self.pages[1].btn_non.configure(state="disabled")
        
        if self.op_mode in ["MODE REEL", "MODE SIMULATION"]:
            # comportement manuel 
            if decision == "oui":
                self.kpi_ok += 1
                self.advance_real_sequence("3_OUI")
            else:
                self.kpi_nok += 1
                self.advance_real_sequence("3_NON")
            self.pages[0].update_kpi(self.kpi_ok, self.kpi_nok)
            
        elif self.op_mode == "MODE REEL AUTO":
            # comportement auto (sauvegarde la decision, le thread auto gere la suite)
            self.auto_inspection_result = decision
            if decision == "oui": self.kpi_ok += 1
            else: self.kpi_nok += 1
            self.pages[0].update_kpi(self.kpi_ok, self.kpi_nok)
            
        else:
            # comportement local / coppeliaSim
            self.inspection_result = decision
            if decision == "oui": self.kpi_ok += 1
            else: self.kpi_nok += 1
            self.pages[0].update_kpi(self.kpi_ok, self.kpi_nok)

    # MACHINE A ETATS : verrouille et deverrouille les CSV dans l'ordre strict
    def advance_real_sequence(self, step):
        self.current_real_step = step
        p1 = self.pages[1] 
        
        # desactive tout par defaut
        p1.btn_csv1.configure(state="disabled", fg_color=BTN_COLOR)
        p1.btn_csv2.configure(state="disabled", fg_color=BTN_COLOR)
        p1.btn_csv3_oui.configure(state="disabled", fg_color=BTN_COLOR)
        p1.btn_csv4_oui.configure(state="disabled", fg_color=BTN_COLOR)
        p1.btn_csv3_non.configure(state="disabled", fg_color=BTN_COLOR)
        p1.btn_csv4_non.configure(state="disabled", fg_color=BTN_COLOR)
        
        # reactive uniquement le bon bouton selon l'etape actuelle
        if step == 0:
            p1.lbl_status.configure(text="ATTENTE DEPART", text_color="#bdc3c7")
            
        elif step == 1:
            p1.btn_csv1.configure(state="normal", fg_color=C_ACCENT)
            p1.lbl_status.configure(text="ATTENTE ORDRE CSV (PART 1)", text_color=C_WARN)
            
        elif step == 2:
            p1.btn_csv2.configure(state="normal", fg_color=C_ACCENT)
            p1.lbl_status.configure(text="FERMEZ LA PINCE PUIS LANCEZ PART 2", text_color=C_WARN)
            gui_log("Trajectoire d'approche terminée. Veuillez fermer la pince avant la PART 2.", "INFO")
            
        elif step == "INSPECTION":
            p1.lbl_status.configure(text="ATTENTE DECISION QUALITE", text_color=C_ALARM)
            p1.btn_oui.configure(state="normal")
            p1.btn_non.configure(state="normal")
            gui_log("Trajectoire de présentation terminée. Veuillez valider la pièce.", "WARN")
            
        elif step == "3_OUI":
            p1.btn_csv3_oui.configure(state="normal", fg_color=C_ON)
            p1.lbl_status.configure(text="LANCEZ PART 3 (CONFORME)", text_color=C_ON)
            
        elif step == "4_OUI":
            p1.btn_csv4_oui.configure(state="normal", fg_color=C_ON)
            p1.lbl_status.configure(text="OUVREZ LA PINCE PUIS LANCEZ PART 4", text_color=C_WARN)
            gui_log("Dépose conforme terminée. Veuillez ouvrir la pince avant la PART 4.", "INFO")
            
        elif step == "3_NON":
            p1.btn_csv3_non.configure(state="normal", fg_color=C_ALARM)
            p1.lbl_status.configure(text="LANCEZ PART 3 (DEFAUT)", text_color=C_ALARM)
            
        elif step == "4_NON":
            p1.btn_csv4_non.configure(state="normal", fg_color=C_ALARM)
            p1.lbl_status.configure(text="OUVREZ LA PINCE PUIS LANCEZ PART 4", text_color=C_WARN)
            gui_log("Dépose défaut terminée. Veuillez ouvrir la pince avant la PART 4.", "INFO")

    # change l'etat de la LED Modbus sur la page overview
    def set_modbus_gui_state(self, connected):
        if self._last_modbus_state != connected:
            self.pages[0].led_modbus.set_state(connected, error=not connected)
            self.pages[2].led_pince_conn.set_state(connected, error=not connected)
            self._last_modbus_state = connected

    # inscription du texte dans le widget de logs
    def log_event(self, log_line):
        self.pages[3].txt_logs.configure(state="normal")
        self.pages[3].txt_logs.insert("end", log_line)
        self.pages[3].txt_logs.see("end")
        self.pages[3].txt_logs.configure(state="disabled")

    # COMMANDES MODBUS TCP (pince zimmer via turck)
    def modbus_init_pince(self):
        threading.Thread(target=self._modbus_task, args=(CW_TRANSFER, True), daemon=True).start()

    def modbus_open_pince(self):
        threading.Thread(target=self._modbus_task, args=(CW_OPEN, False), daemon=True).start()

    def modbus_close_pince(self):
        threading.Thread(target=self._modbus_task, args=(CW_CLOSE, False), daemon=True).start()
        
    def modbus_reset_error(self):
        threading.Thread(target=self._modbus_task, args=(CW_ERROR_RESET, False), daemon=True).start()

    def _modbus_task(self, command, is_init):
        try:
            gui_log(f"Connexion Modbus au Turck ({IP_TURCK}:{PORT_MODBUS})...", "INFO")
            client = ModbusTcpClient(IP_TURCK, port=PORT_MODBUS, timeout=1.0)
            if client.connect():
                self.after(0, lambda: self.set_modbus_gui_state(True))
                gui_log("Connexion Modbus réussie.", "SUCCESS")
                
                f_val = int(self.pages[2].seg_force.get())
                s_val = int(self.pages[2].seg_speed.get())
                
                # sequence speciale d'initialisation (acquittement + reset requis par zimmer)
                if is_init:
                    gui_log("Envoi acquittement erreur (32768) + Reset...", "INFO")
                    client.write_registers(address=ADDR_OUTPUTS, values=build_process_data(CW_ERROR_RESET, f_val, s_val))
                    time.sleep(0.5)
                    client.write_registers(address=ADDR_OUTPUTS, values=build_process_data(CW_RESET, f_val, s_val))
                    time.sleep(0.5)
                    
                gui_log(f"Envoi du ControlWord = {command} (Force: {f_val}, Vitesse: {s_val})", "INFO")
                client.write_registers(address=ADDR_OUTPUTS, values=build_process_data(command, f_val, s_val))
                
                time.sleep(0.5)
                gui_log("Lecture de l'état de la pince...", "INFO")
                res = client.read_input_registers(address=ADDR_INPUTS, count=3)
                
                if not res.isError():
                    st_word = res.registers[0]
                    diag_word = res.registers[1]
                    pos_word = res.registers[2]
                    
                    data_transfer_ok = (st_word & (1 << 12)) != 0
                    has_error = (st_word & (1 << 15)) != 0
                    
                    raw = f" REGISTRES LUS\n"
                    raw += f"Reg 1 (StatusWord) : {st_word} ({bin(st_word)})\n"
                    raw += f"Reg 2 (Diagnosis)  : {diag_word} ({hex(diag_word)})\n"
                    raw += f"Reg 3 (Position)   : {pos_word}\n\n"
                    raw += f" DIAGNOSTIC RAPIDE\n"
                    raw += f"DataTransferOK (Bit 12) : {data_transfer_ok}\n"
                    raw += f" Erreur Active (Bit 15)  : {has_error}\n"
                    
                    self.after(0, lambda: self.pages[2].update_state_display(st_word, diag_word, pos_word, raw, has_error))
                    gui_log("Lecture de l'état réussie.", "SUCCESS")
                else:
                    gui_log("Erreur lors de la lecture des registres de la pince.", "ERROR")
                    
                client.close()
            else:
                self.after(0, lambda: self.set_modbus_gui_state(False))
                self.after(0, lambda: self.pages[1].lbl_status.configure(text="PINCE DECONNECTEE", text_color=C_ALARM))
                gui_log("Impossible d'établir la liaison Modbus TCP avec le boitier Turck.", "ERROR")
        except Exception as e:
            self.after(0, lambda: self.set_modbus_gui_state(False))
            self.after(0, lambda: self.pages[1].lbl_status.configure(text="PINCE DECONNECTEE", text_color=C_ALARM)) 
            gui_log(f"Erreur Modbus : {e}", "ERROR")

    def modbus_command_sync(self, command, is_init=False):
        """ Envoie un mot de contrôle via Modbus TCP. Bloque l'exécution pendant l'ouverture/fermeture """
        try:
            gui_log(f"Envoi de l'ordre pince : {command}", "INFO")
            client = ModbusTcpClient(IP_TURCK, port=PORT_MODBUS, timeout=1.0)
            if client.connect():
                self.after(0, lambda: self.set_modbus_gui_state(True))
                # init (acquittement + reset) utile lors de la toute premiere action
                if is_init:
                    client.write_registers(address=ADDR_OUTPUTS, values=build_process_data(CW_ERROR_RESET, 4, 4))
                    time.sleep(0.5)
                    client.write_registers(address=ADDR_OUTPUTS, values=build_process_data(CW_RESET, 4, 4))
                    time.sleep(0.5)
                # ordre reel d'ouverture ou de fermeture
                client.write_registers(address=ADDR_OUTPUTS, values=build_process_data(command, 4, 4))
                time.sleep(1.5) # Délai mécanique permettant à la pince de se fermer/s'ouvrir physiquement
                client.close()
            else:
                self.after(0, lambda: self.set_modbus_gui_state(False))
                gui_log("Echec connexion Modbus pince.", "ERROR")
        except Exception as e:
            self.after(0, lambda: self.set_modbus_gui_state(False))
            gui_log(f"Erreur Modbus : {e}", "ERROR")

    # EXECUTION CSV MANUELLE (Reel et Simulation RViz)
    def run_csv(self, filename):
        if self.op_mode not in ["MODE REEL", "MODE SIMULATION"]:
            gui_log("Tentative d'envoi CSV rejetée: Basculez sur 'MODE REEL' ou 'MODE SIMULATION'.", "WARN")
            return
        self.cycle_started = True
        threading.Thread(target=self._run_csv_thread, args=(filename,), daemon=True).start()

    def _run_csv_thread(self, filename):
        abs_path = os.path.abspath(filename)
        self.after(0, lambda: self.pages[1].lbl_status.configure(text=f"EXECUTION: {filename[-12:]}", text_color="#9b59b6"))
        self.after(0, lambda: self.pages[1].led_mvt.set_state(True))
        
        # fichier mouchard : permet de savoir quand le terminal externe s'est referme
        flag_file = "/tmp/ros2_traj_done.flag"
        if os.path.exists(flag_file):
            try: os.remove(flag_file)
            except: pass
            
        setup_env = "source ~/.bashrc 2>/dev/null; source /opt/ros/humble/setup.bash 2>/dev/null; source ~/colcon_ws/install/setup.bash 2>/dev/null;"
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        if self.op_mode == "MODE SIMULATION":
            cmd_ros = f"ros2 run mfja_motoman_hc10 execute_trajectory_sim.py {abs_path} --robot left"
        else:
            local_script = os.path.join(script_dir, "execute_trajectory.py")
            cmd_ros = f"python3 {local_script} {abs_path} --robot left"
        
        # Le trap déclenche la creation du fichier .flag même si l'utilisateur fait un Ctrl+C dans le terminal
        full_cmd = f"gnome-terminal --title='EXECUTION CSV' -- bash -c \"trap 'touch {flag_file}' EXIT; {setup_env} {cmd_ros}; sleep 1\""
        
        subprocess.Popen(full_cmd, shell=True)
        gui_log(f"Terminal externe ouvert pour le CSV : {filename[-12:]}", "SUCCESS")
        
        # boucle d'attente : L'IHM surveille l'apparition du fichier mouchard pour deverouiller la suite
        while not os.path.exists(flag_file):
            if not self.cycle_started: # Si Arrêt logiciel pressé
                break
            time.sleep(0.2)
            
        try: os.remove(flag_file)
        except: pass
        
        self.after(0, lambda: self.pages[1].led_mvt.set_state(False))
        
        # si l'arret d'urgence a été pressé
        if not self.cycle_started:
            gui_log("Séquence interrompue par l'opérateur.", "WARN")
            self.after(0, lambda: self.advance_real_sequence(self.current_real_step))
            return
            
        gui_log(f"Fin d'exécution détectée pour {filename[-12:]}.", "INFO")
        
        # machine à etat final : deverrouille la partie n+1 en fonction de la partie qui vient de se terminer
        if "PART1" in filename: self.after(0, lambda: self.advance_real_sequence(2))
        elif "PART2" in filename: self.after(0, lambda: self.advance_real_sequence("INSPECTION"))
        elif "PART3_oui" in filename: self.after(0, lambda: self.advance_real_sequence("4_OUI"))
        elif "PART4_oui" in filename: self.after(0, lambda: self.advance_real_sequence(1))
        elif "PART3_non" in filename: self.after(0, lambda: self.advance_real_sequence("4_NON"))
        elif "PART4_non" in filename: self.after(0, lambda: self.advance_real_sequence(1))

    # MOTEUR DU CYCLE COMPLET AUTOMATIQUE (Robot Réel Uniquement)
    def run_csv_sync(self, filename):
        """ Exécute un CSV via ROS 2 et met le thread en pause jusqu'à la fin du mouvement """
        abs_path = os.path.abspath(filename)
        gui_log(f"[AUTO] Lancement de la trajectoire : {filename} (Vitesse: {int(self.speed_factor*100)}%)", "INFO")
        
        flag_file = "/tmp/ros2_traj_done.flag"
        if os.path.exists(flag_file):
            try: os.remove(flag_file)
            except: pass
            
        setup_env = "source ~/.bashrc 2>/dev/null; source /opt/ros/humble/setup.bash 2>/dev/null; source ~/colcon_ws/install/setup.bash 2>/dev/null;"
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        local_script = os.path.join(script_dir, "execute_trajectory.py")
        speed_arg = f"--speed {self.speed_factor:.2f}"
        cmd_ros = f"python3 {local_script} {abs_path} --robot left {speed_arg}"
        
        full_cmd = f"gnome-terminal --title='EXECUTION AUTO CSV' -- bash -c \"trap 'touch {flag_file}' EXIT; {setup_env} {cmd_ros}; sleep 1\""
        subprocess.Popen(full_cmd, shell=True)
        
        #attente bloquante mais thread-safe pour suivre l'exécution
        while not os.path.exists(flag_file):
            if not self.auto_cycle_active: # Permet de sortir proprement si l'arrêt d'urgence est pressé
                break
            time.sleep(0.2)
            
        try: os.remove(flag_file)
        except: pass

    # cycle automatisé (le robot fait tout y compris l'ouverture et la fermeture de la pince mais on choisit quand meme si la piece est bonne ou non)
    def auto_cycle_thread(self):
        gui_log("[AUTO] Démarrage du cycle 100% Automatique.", "SUCCESS")
        p1 = self.pages[1]
        
        # on commence par louverture de la pince
        self.after(0, lambda: p1.lbl_status.configure(text="ETAPE 1: INIT PINCE...", text_color=C_WARN))
        self.after(0, lambda: p1.led_mvt.set_state(True))
        
        self.modbus_command_sync(CW_OPEN, is_init=True) # On passe is_init=True pour débloquer la pince au cas où
        if not self.auto_cycle_active: return
        
        # etape traj 1
        self.after(0, lambda: p1.lbl_status.configure(text="ETAPE 2: EXECUTION PART 1...", text_color=C_WARN))
        self.run_csv_sync("trajectoire_rad_RV_PART1.csv")
        if not self.auto_cycle_active: return
        
        # etape fermeture de la pince
        self.after(0, lambda: p1.lbl_status.configure(text="ETAPE 3: FERMETURE PINCE...", text_color=C_WARN))
        self.modbus_command_sync(CW_CLOSE)
        if not self.auto_cycle_active: return
        
        # etape traj2
        self.after(0, lambda: p1.lbl_status.configure(text="ETAPE 4: EXECUTION PART 2...", text_color=C_WARN))
        self.run_csv_sync("trajectoire_rad_RV_PART2.csv")
        if not self.auto_cycle_active: return
        
        # etape oui ou non par l'operateur
        self.after(0, lambda: p1.lbl_status.configure(text="ATTENTE DECISION QUALITE...", text_color=C_ALARM))
        self.after(0, lambda: p1.led_mvt.set_state(False))
        gui_log("[AUTO] En attente de la décision opérateur.", "WARN")
        self.auto_inspection_result = None
        # on degrise les boutons oui et non de la page principale
        self.after(0, lambda: p1.btn_oui.configure(state="normal"))
        self.after(0, lambda: p1.btn_non.configure(state="normal"))
        
        #on attend tant que l'opérateur ne clique pas
        while self.auto_inspection_result is None:
            if not self.auto_cycle_active: return
            time.sleep(0.1)
            
        reponse = self.auto_inspection_result
        self.after(0, lambda: p1.btn_oui.configure(state="disabled"))
        self.after(0, lambda: p1.btn_non.configure(state="disabled"))
        self.after(0, lambda: p1.led_mvt.set_state(True))
        
        # on choisit en fonction de l'etat de la pince
        if reponse == "oui":
            # traj 3 oui(transport vers rack bon)
            self.after(0, lambda: p1.lbl_status.configure(text="ETAPE 6: PART 3 (CONFORME)...", text_color=C_ON))
            self.run_csv_sync("trajectoire_rad_RV_PART3_oui.csv")
            if not self.auto_cycle_active: return
            
            # ouvert pince (depose)
            self.after(0, lambda: p1.lbl_status.configure(text="ETAPE 7: OUVERTURE PINCE...", text_color=C_WARN))
            self.modbus_command_sync(CW_OPEN)
            if not self.auto_cycle_active: return
            
            # traj 4 oui (retour position sécu)
            self.after(0, lambda: p1.lbl_status.configure(text="ETAPE 8: PART 4 (CONFORME)...", text_color=C_ON))
            self.run_csv_sync("trajectoire_rad_RV_PART4_oui.csv")
            
        else:
            # traj 3 non(transport vers poubelle)
            self.after(0, lambda: p1.lbl_status.configure(text="ETAPE 6: PART 3 (DEFAUT)...", text_color=C_ALARM))
            self.run_csv_sync("trajectoire_rad_RV_PART3_non.csv")
            if not self.auto_cycle_active: return
            
            # ouvert pince (dépose dans la poubelle)
            self.after(0, lambda: p1.lbl_status.configure(text="ETAPE 7: OUVERTURE PINCE...", text_color=C_WARN))
            self.modbus_command_sync(CW_OPEN)
            if not self.auto_cycle_active: return
            
            # traj 4 non (retour position sécu)
            self.after(0, lambda: p1.lbl_status.configure(text="ETAPE 8: PART 4 (DEFAUT)...", text_color=C_ALARM))
            self.run_csv_sync("trajectoire_rad_RV_PART4_non.csv")
            
        # fin de la boucle
        self.after(0, lambda: p1.lbl_status.configure(text="CYCLE TERMINE AVEC SUCCES", text_color=C_ON))
        self.after(0, lambda: p1.led_mvt.set_state(False))
        gui_log("[AUTO] Cycle complètement terminé !", "SUCCESS")
        
        # Envoi de la confirmation de fin à l'API (Coil 1)
        self.signal_end_to_plc()
        
        self.auto_cycle_active = False
        
        # Restaure l'état visuel du bouton selon le mode
        if self.op_mode == "MODE REEL AUTO":
            self.after(0, lambda: p1.btn_start.configure(state="disabled", text="ATTENTE AUTOMATE (10.100.12.121)", fg_color="#e67e22"))
        else:
            self.after(0, lambda: p1.btn_start.configure(state="normal", text="DEMARRER CYCLE"))

    # DESSIN GRAPHIQUE DES JOUGES ET HISTORIQUES
    # dessin manuel du graphique historique des articulations dans un canvas tkinter
    def draw_joints_logic(self, canvas, history, is_mini=True):
        canvas.delete("all")
        w, h = canvas.winfo_width(), canvas.winfo_height()
        if w < 10: return
        
        # quadrillage pour de fond
        for i in range(1, 10):
            canvas.create_line(0, h*(i/10), w, h*(i/10), fill="#111")
            canvas.create_line(w*(i/10), 0, w*(i/10), h, fill="#111")
            
        if not is_mini:
            canvas.create_text(30, 15, text="+180°", fill="#555", font=("Consolas", 10))
            canvas.create_text(30, h/2, text="0°", fill="#555", font=("Consolas", 10))
            canvas.create_text(30, h-15, text="-180°", fill="#555", font=("Consolas", 10))

        if not history: return
        
        # pour eviter de laguer si o a trop de points
        step = max(1, len(history) // 200)
        display_hist = history[::step]
        
        colors = ["#e74c3c", "#3498db", "#2ecc71", "#f1c40f", "#9b59b6", "#e67e22"]
        dx = w / max(1, len(display_hist) - 1)
        
        # trace des 6 courbes (une par articulation)
        for axe in range(6):
            points = []
            for i, row in enumerate(display_hist):
                val_deg = math.degrees(float(row[axe]))
                y = h/2 - (val_deg / 180.0) * (h/2) # Echelle -180 à +180
                points.extend([i * dx, y])
            if len(points) >= 4:
                canvas.create_line(points, fill=colors[axe], width=1 if is_mini else 2, smooth=True)

    # fonct de navigation entre les pages
    def show_page(self, idx):
        self.pages[self.current_idx].pack_forget()
        self.current_idx = idx
        self.pages[self.current_idx].pack(fill="both", expand=True)
        self.lbl_title.configure(text=self.page_names[self.current_idx])
        self.lbl_dots.configure(text=f"PAGE {self.current_idx + 1} SUR {len(self.pages)}")

    def next_page(self, event=None):
        if self.current_idx < len(self.pages) - 1: self.show_page(self.current_idx + 1)
    def prev_page(self, event=None):
        if self.current_idx > 0: self.show_page(self.current_idx - 1)

# LECTURE DE LA TELEMETRIE SANS DEPENDANCE ROS2 
def ros2_listener_subprocess(app_instance):
    """ Écoute les angles du robot en lisant la sortie texte pure de ROS 2 pour ne pas dépendre de rclpy """
    setup_env = "source /opt/ros/humble/setup.bash 2>/dev/null; source ~/colcon_ws/install/setup.bash 2>/dev/null;"
    
    # stdbuf -oL force la console à recracher le texte en temps réel
    cmd = f"bash -c '{setup_env} stdbuf -oL ros2 topic echo /yaskawa_LEFT/joint_states'"
    process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
    
    q_rad = [0.0]*6
    pos_idx = 0
    in_position = False
    
    # lecture ligne par ligne du terminal 
    for line in iter(process.stdout.readline, ''):
        line = line.strip()
        
        # on repere le début de la liste "position:" 
        if line.startswith('position:'):
            in_position = True
            pos_idx = 0
            continue
            
        if in_position:
            if line.startswith('-'):
                try:
                    val = float(line[1:].strip())
                    q_rad[pos_idx] = val
                    pos_idx += 1
                    
                    if pos_idx == 6: # On a récupéré les 6 axes
                        # Ne met à jour l'IHM que si on n'est pas en simulation locale CoppeliaSim
                        if app_instance.op_mode in ["MODE REEL", "MODE REEL AUTO", "MODE SIMULATION"]:
                            # On envoie les valeurs à la boucle principale Tkinter pour animer les jauges
                            app_instance.after(0, lambda q=list(q_rad): app_instance.pages[1].update_telemetrie(q))
                        in_position = False
                except ValueError:
                    pass
            else:
                in_position = False

# INIT COPPELIASIM 
sim = None
base_handle = None
tcp_handle = None
target_handle = None
joint_handles = []
secu_handle = None
appro_navette_handle = None
navette_handle = None
rack_handle = None
rack1_handle = None
rack2_handle = None
appro_rack_handle = None
appro_rack2_handle = None
cylinder_handle = None
sim_ik = None
ik_env = None
ik_group = None
simToIkMap = {}
COPPELIA_AVAILABLE = False

def get_ik_handle(scene_handle): 
    return simToIkMap.get(scene_handle) or simToIkMap.get(str(scene_handle))

def sync_ik():
    try: sim_ik.syncFromSim(ik_env, [ik_group])
    except Exception:
        try: sim_ik.applySceneToIkEnvironment(ik_env, ik_group)
        except Exception:
            matrice = sim.getObjectMatrix(target_handle, base_handle)
            sim_ik.setObjectMatrix(ik_env, get_ik_handle(target_handle), get_ik_handle(base_handle), matrice)

# ping simple pour vérifier que Coppelia tourne sur le port 23000
def is_coppelia_running(host='127.0.0.1', port=23000):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex((host, port)) == 0

# configuration de la scène 3D et récup des poignees des objets 3D
def init_coppelia():
    global sim, base_handle, tcp_handle, target_handle, joint_handles
    global secu_handle, appro_navette_handle, navette_handle, rack_handle
    global rack1_handle, rack2_handle, appro_rack_handle, appro_rack2_handle
    global cylinder_handle, sim_ik, ik_env, ik_group, simToIkMap, COPPELIA_AVAILABLE

    if is_coppelia_running():
        try:
            client = RemoteAPIClient()
            sim = client.require('sim')
            sim.setStepping(True)

            base_handle = sim.getObject("/HC10[0]")
            tcp_handle = sim.getObject("/HC10[0]/Q1/HC10/Q2/HC10/Q3/HC10/Q4/HC10/Q5/HC10/Q6/HC10/TCP")
            target_handle = sim.getObject("/Target_IK")

            joint_handles = [
                sim.getObject('/HC10[0]/Q1'), sim.getObject('/HC10[0]/Q1/HC10/Q2'), 
                sim.getObject('/HC10[0]/Q1/HC10/Q2/HC10/Q3'), sim.getObject('/HC10[0]/Q1/HC10/Q2/HC10/Q3/HC10/Q4'), 
                sim.getObject('/HC10[0]/Q1/HC10/Q2/HC10/Q3/HC10/Q4/HC10/Q5'), sim.getObject('/HC10[0]/Q1/HC10/Q2/HC10/Q3/HC10/Q4/HC10/Q5/HC10/Q6')
            ]

            # objets cartésiens cibles de la scène
            secu_handle = sim.getObject('/Secu')
            appro_navette_handle = sim.getObject('/Approche_navette')
            navette_handle = sim.getObject('/Navette')
            rack_handle = sim.getObject('/Rack')
            rack1_handle = sim.getObject('/Rack1')
            rack2_handle = sim.getObject('/Rack2')
            appro_rack_handle = sim.getObject('/Appro_rack')
            appro_rack2_handle = sim.getObject('/Appro_rack2')
            cylinder_handle = sim.getObject('/Cylinder') #pièce cylindrique manipulée

            # init de la cinématique inverse (IK)
            sim_ik = client.require('simIK')
            ik_env = sim_ik.createEnvironment()
            ik_group = sim_ik.createIkGroup(ik_env)
            sim_ik.setIkGroupCalculation(ik_env, ik_group, sim_ik.method_damped_least_squares, 1e-1, 1000)
            _, simToIkMap, _ = sim_ik.addIkElementFromScene(ik_env, ik_group, base_handle, tcp_handle, target_handle, sim_ik.constraint_pose)
            
            COPPELIA_AVAILABLE = True
            gui_log("Connexion au Jumeau Numérique CoppeliaSim réussie.", "SUCCESS")
        except Exception as e:
            COPPELIA_AVAILABLE = False
            gui_log(f"Erreur d'initialisation CoppeliaSim : {e}", "WARN")
    else:
        COPPELIA_AVAILABLE = False
        gui_log("CoppeliaSim n'est pas ouvert (Port 23000 fermé). IHM démarrée sans simulation.", "WARN")

# GESTION DES DÉPLACEMENTS COPPELIASIM 
# maj des graphiques locaux avec les donnees calculees
def common_telemetry_update(q_curr, dq_curr, pos_tcp):
    global temps_global
    
    historique_t.append(temps_global)
    historique_q.append(q_curr)
    historique_dq.append(dq_curr)
    historique_tcp_pos.append(pos_tcp)
    buffer_q.append(q_curr)
    
    app.live_q_history.append(q_curr)
    app.live_tcp_history.append(pos_tcp)
    
    if len(historique_tcp_pos) > 1:
        v_tcp = np.linalg.norm(np.array(pos_tcp) - np.array(historique_tcp_pos[-2])) / param["ts"]
    else:
        v_tcp = 0.0
        
    historique_tcp_v.append(v_tcp)
    temps_global += param["ts"]

    try:
        app.after(0, lambda q=q_curr: app.pages[1].update_telemetrie(q))
        app.after(0, lambda v=v_tcp: app.pages[0].update_tcp_speed(v))
        app.after(0, lambda h=app.live_tcp_history: app.pages[0].update_tcp_3d(h))
        if app.large_graph_window: app.after(0, lambda: app.draw_joints_logic(app.large_graph_window.canvas, app.live_q_history, is_mini=False))
        if app.large_tcp_window: app.after(0, lambda h=app.live_tcp_history: app.large_tcp_window.update_tcp_3d(h))
    except Exception: pass

# jouer une trajectoire point à point articulaire
def play_moveJ(q_cible, temps_initial):
    q_actuel = [sim.getJointPosition(j) for j in joint_handles]
    q_traj, dq_traj, _ = moveJ_adaptatif(q_actuel, q_cible, temps_initial, param)
    for k in range(q_traj.shape[1]):
        q_curr = q_traj[:, k].tolist()
        dq_curr = dq_traj[:, k].tolist()
        for i in range(6): sim.setJointPosition(joint_handles[i], q_curr[i])
        sim.step()
        pos_tcp = sim.getObjectPosition(tcp_handle, base_handle)
        common_telemetry_update(q_curr, dq_curr, pos_tcp)

# jouer une trajectoire linéaire (cinématique Inverse)
def play_moveL(dummy_cible_handle, temps_voulu):
    pos_actuelle = sim.getObjectPosition(tcp_handle, base_handle)
    orient_actuelle = sim.getObjectOrientation(tcp_handle, base_handle)
    pos_cible = sim.getObjectPosition(dummy_cible_handle, base_handle)
    orient_cible = sim.getObjectOrientation(dummy_cible_handle, base_handle)
    
    orient_cible_corrige = np.unwrap([orient_actuelle, orient_cible], axis=0)[1].tolist()
    distance = np.linalg.norm(np.array(pos_cible) - np.array(pos_actuelle))
    temps_min_cart = distance / 0.25
    temps_min_ang = np.max(np.abs(np.array(orient_cible_corrige) - np.array(orient_actuelle))) / np.deg2rad(50) 
    
    traj_6d = moveL(pos_actuelle + orient_actuelle, pos_cible + orient_cible_corrige, max(temps_voulu, temps_min_cart, temps_min_ang), param)
    q_prev = [sim.getJointPosition(j) for j in joint_handles]
    
    for k in range(traj_6d.shape[1]):
        sim.setObjectPosition(target_handle, base_handle, traj_6d[0:3, k].tolist())
        sim.setObjectOrientation(target_handle, base_handle, traj_6d[3:6, k].tolist())
        sync_ik()
        sim_ik.handleIkGroup(ik_env, ik_group)
        q_curr = [sim_ik.getJointPosition(ik_env, get_ik_handle(j)) for j in joint_handles]
        for i, j in enumerate(joint_handles): sim.setJointPosition(j, q_curr[i])
        sim.step()
        dq_curr = [(q_curr[i] - q_prev[i]) / param["ts"] for i in range(6)]
        q_prev = q_curr
        pos_tcp = sim.getObjectPosition(tcp_handle, base_handle)
        common_telemetry_update(q_curr, dq_curr, pos_tcp)

# sauvegarde de la trajectoire locale sous forme de CSV (pour utilisation  en Réel)
def save_csv_part(part_number, suffixe=""):
    global buffer_q
    if not buffer_q: return
    q_array_rad = np.array(buffer_q)
    if len(q_array_rad) > 200:
        q_array_rad = q_array_rad[np.linspace(0, len(q_array_rad) - 1, 200).astype(int)]
    np.savetxt(f"trajectoire_rad_RV_PART{part_number}{suffixe}.csv", q_array_rad, delimiter=",", fmt="%.4f")
    buffer_q.clear()

# exécute un bloc logique (moveJ, moveL, pause)
def executer_sous_sequence(sequence):
    for etape in sequence:
        if etape["action"] == "moveJ": play_moveJ(etape["cible"], etape["temps"])
        elif etape["action"] == "moveL": play_moveL(etape["cible"], etape["temps"])
        elif etape["action"] == "pause":
            for _ in range(int(etape["temps"] / param["ts"])): sim.step()

# THREAD PRINCIPAL DE LA SÉQUENCE (simulation coppeliasim uniquement)
def robot_logic_thread():
    init_coppelia()

    # def des points statiques en deg puis conversion en rad
    q_navette_deg = [-160.1090, 8.2981, 4.5049, -4.33, -85.3011, 74.7381]
    q_navette = np.deg2rad(q_navette_deg).tolist()
    q_appro_navette_deg = [-160.1090, 8.5, 15, -4.33, -95.00, 65]
    q_appro_navette = np.deg2rad(q_appro_navette_deg).tolist()
    q_zero = [0.0, 0.0, 0.0, 0.0, np.deg2rad(-90), 0.0]
    q_secud= [-74.0986, 9.1255, 19.3387, -1.1592, -104.4761, 161.4395]
    q_secu = np.deg2rad(q_secud).tolist()

    # construction des 4 sous-parties du programme 
    seq_part1 = [
        {"action": "moveJ", "cible": q_zero, "temps": 10, "nom": "SECU"},
        {"action": "moveJ", "cible": q_secu, "temps": 10, "nom": "SECU"},
        {"action": "moveJ", "cible": q_appro_navette, "temps": 15, "nom": "appro NAVETTE"},
        {"action": "moveJ", "cible": q_navette, "temps": 15, "nom": "NAVETTE (Prise)"}
    ]
    seq_part2 = [
        {"action": "moveJ", "cible": q_appro_navette, "temps": 15, "nom": "appro NAVETTE"},
        {"action": "moveJ", "cible": q_secu, "temps": 10, "nom": "SECU"},
        {"action": "moveL", "cible": rack1_handle, "temps": 10, "nom": "RACK1 (Présentation)"}
    ]
    seq_part3_oui = [
        {"action": "moveL", "cible": appro_rack_handle, "temps": 10, "nom": "APPRO RACK"},
        {"action": "moveL", "cible": rack_handle, "temps": 10, "nom": "RACK"}
    ]
    seq_part3_non = [
        {"action": "moveL", "cible": appro_rack2_handle, "temps": 10, "nom": "APPRO RACK2"},
        {"action": "moveL", "cible": rack2_handle, "temps": 10, "nom": "RACK2"}
    ]
    seq_part4_oui = [
        {"action": "moveL", "cible": appro_rack_handle, "temps": 10, "nom": "DEGAGEMENT RACK"},
        {"action": "moveJ", "cible": q_zero, "temps": 10, "nom": "Retour à zéro"}
    ]
    seq_part4_non = [
        {"action": "moveL", "cible": appro_rack2_handle, "temps": 10, "nom": "DEGAGEMENT RACK2"},
        {"action": "moveJ", "cible": q_zero, "temps": 10, "nom": "Retour à zéro"}
    ]

    # mise en position initiale si Coppelia  actif
    if COPPELIA_AVAILABLE:
        for i in range(6): sim.setJointPosition(joint_handles[i], q_zero[i])
        sim.step()
        sim.setObjectPosition(target_handle, base_handle, sim.getObjectPosition(secu_handle, base_handle))
        sim.setObjectOrientation(target_handle, base_handle, sim.getObjectOrientation(secu_handle, base_handle))
        sync_ik()
        sim_ik.handleIkGroup(ik_env, ik_group)
        pos_tcp_zero = sim.getObjectPosition(tcp_handle, base_handle)
        common_telemetry_update(q_zero, [0]*6, pos_tcp_zero)
        sim.startSimulation()
        time.sleep(0.5)
        gui_log("Simulation CoppeliaSim démarrée.", "INFO")
    else:
        gui_log("Processus en attente (Sans Simulation). Pilotez en Mode REEL.", "WARN")
    
    #boucle infinie d'exécution cyclique
    while True:
        # etape de depart
        while True:
            current_mode = app.op_mode
            if current_mode == "MODE LOCAL":
                app.after(0, lambda: app.pages[1].lbl_status.configure(text="ATTENTE DEPART (LOCAL)", text_color=C_WARN))
                if app.cycle_started:
                    if not COPPELIA_AVAILABLE:
                        gui_log("Simulation impossible sans CoppeliaSim.", "ERROR")
                        app.after(0, lambda: app.pages[1].lbl_status.configure(text="ERREUR SIMULATION", text_color=C_ALARM))
                        app.cycle_started = False
                        time.sleep(2)
                    else:
                        gui_log("Ordre de départ LOCAL reçu.", "INFO")
                        break
            elif current_mode in ["MODE REEL", "MODE REEL AUTO", "MODE SIMULATION"]:
                pass
            
            if COPPELIA_AVAILABLE: sim.step()
            time.sleep(0.05)

        # on lance les étapes sequentielles
        app.after(0, lambda: app.pages[1].lbl_status.configure(text="PRODUCTION EN COURS", text_color=C_ACCENT))
        app.after(0, lambda: app.pages[1].led_mvt.set_state(True))

        gui_log("Exécution de la PARTIE 1", "INFO")
        executer_sous_sequence(seq_part1)
        save_csv_part(1) 
        # simulation daccroche de la pièce
        sim.setObjectParent(cylinder_handle, tcp_handle, True)
        app.after(0, lambda: app.pages[1].led_pince.set_state(True))

        gui_log("Exécution de la PARTIE 2", "INFO")
        executer_sous_sequence(seq_part2)
        save_csv_part(2)

        # etape qualite
        gui_log("Attente de validation d'inspection de l'opérateur.", "WARN")
        app.after(0, lambda: app.pages[1].lbl_status.configure(text="ATTENTE DECISION QUALITE", text_color=C_ALARM))
        app.after(0, lambda: app.pages[1].led_mvt.set_state(False))
        # Débloque les boutons logiciels "Pièce Conforme" et "Pièce Défaut"
        app.after(0, lambda: app.pages[1].btn_oui.configure(state="normal"))
        app.after(0, lambda: app.pages[1].btn_non.configure(state="normal"))
        
        while app.inspection_result is None:
            if COPPELIA_AVAILABLE: sim.step()
            time.sleep(0.1)
        
        reponse = app.inspection_result
        app.inspection_result = None 
        gui_log(f"Opérateur a validé la pièce comme : {'CONFORME' if reponse == 'oui' else 'DEFAUT'}.", "INFO")
        
        app.after(0, lambda: app.pages[1].btn_oui.configure(state="disabled"))
        app.after(0, lambda: app.pages[1].btn_non.configure(state="disabled"))
        app.after(0, lambda: app.pages[1].lbl_status.configure(text="PRODUCTION EN COURS", text_color=C_ACCENT))
        app.after(0, lambda: app.pages[1].led_mvt.set_state(True))

        # etape qualite (oui ou non)
        if reponse == "oui":
            gui_log("Exécution de la PARTIE 3_oui", "INFO")
            executer_sous_sequence(seq_part3_oui)
            save_csv_part(3, "_oui")
            # simulation depose 
            sim.setObjectParent(cylinder_handle, rack_handle, True)
            app.after(0, lambda: app.pages[1].led_pince.set_state(False))
            gui_log("Exécution de la PARTIE 4_oui", "INFO")
            executer_sous_sequence(seq_part4_oui)
            save_csv_part(4, "_oui")
        else:
            gui_log("Exécution de la PARTIE 3_non", "INFO")
            executer_sous_sequence(seq_part3_non)
            save_csv_part(3, "_non")
            # simulation depose 
            sim.setObjectParent(cylinder_handle, rack2_handle, True)
            app.after(0, lambda: app.pages[1].led_pince.set_state(False))
            gui_log("Exécution de la PARTIE 4_non", "INFO")
            executer_sous_sequence(seq_part4_non)
            save_csv_part(4, "_non")

        # fin de cycle
        app.after(0, lambda: app.pages[1].lbl_status.configure(text="CYCLE TERMINE", text_color=C_ON))
        app.after(0, lambda: app.pages[1].led_mvt.set_state(False))
        gui_log("Cycle local terminé avec succès.", "SUCCESS")
        app.cycle_started = False

if __name__ == "__main__":
    app = YaskawaDashboard()
    
    # lancement du processus principal 
    threading.Thread(target=robot_logic_thread, daemon=True).start()
    
    # lancement du thread caché qui interroge ROS 2 par la ligne de commande  pour animer l'IHM
    threading.Thread(target=ros2_listener_subprocess, args=(app,), daemon=True).start()
        
    app.mainloop()