import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLabel, QComboBox, QDoubleSpinBox, QPushButton, QGridLayout, QGroupBox, QStackedWidget, QStatusBar
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot , QObject
from robot3D_visualisation import PyBulletCanvas


# Intégration de Matplotlib dans PyQt5
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

import Leven_Marq as LM 
import Interface_pybullet as In   
import Transmission as tr


import cv2
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap



class CameraThread(QThread):
    """Thread dédié à la capture vidéo pour ne pas bloquer l'IHM"""
    frame_received = pyqtSignal(QImage)

    def __init__(self, camera_index=0):
        super().__init__()
        self.camera_index = camera_index
        self.running = False

    def run(self):
        cap = cv2.VideoCapture(self.camera_index)
        self.running = True
        while self.running:
            ret, frame = cap.read()
            if ret:
                # OpenCV utilise le format BGR, on convertit en RGB pour PyQt
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_frame.shape
                bytes_per_line = ch * w
                
                # Création de l'image Qt
                qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.frame_received.emit(qt_image)
            else:
                self.msleep(30) # Évite de saturer le CPU si la caméra coupe
        cap.release()

    def stop(self):
        self.running = False
        self.wait()



class MTD3DCanvas(FigureCanvas):
    """Zone dédiée au dessin 3D du bras robotique"""
    def __init__(self, parent=None, width=4, height=3, dpi=100):
        # Initialisation de la figure Matplotlib avec fond transparent/sombre
        self.fig = Figure(figsize=(width, height), dpi=dpi, facecolor='#1e1e24')
        super().__init__(self.fig)
        self.setParent(parent)
        
        # Configuration des axes 3D
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor('#1e1e24')
        self.init_axes_style()
        
        # Ligne représentant le robot (segments entre articulations)
        self.robot_line, = self.ax.plot([], [], [], 'o-', lw=4, color='#00adb5', markerfacecolor='#ffffff', markersize=6)
        
    def init_axes_style(self):
        self.ax.set_xlabel('X (mm)', color='#eeeeee', fontsize=8)
        self.ax.set_ylabel('Y (mm)', color='#eeeeee', fontsize=8)
        self.ax.set_zlabel('Z (mm)', color='#eeeeee', fontsize=8)
        
        # Définition des limites de l'espace de travail (Ajustez selon votre robot)
        self.ax.set_xlim([-350, 350])
        self.ax.set_ylim([-350, 350])
        self.ax.set_zlim([0, 400])
        
        # Couleurs des graduations et grilles
        self.ax.tick_params(colors='#a0a0ab', labelsize=8)
        self.ax.xaxis.pane.set_edgecolor('#3a3a44')
        self.ax.yaxis.pane.set_edgecolor('#3a3a44')
        self.ax.zaxis.pane.set_edgecolor('#3a3a44')
        self.ax.grid(True, color='#3a3a44', linestyle='--')

    def draw_robot(self, points):
        """Prend une liste de coordonnées [[x0,y0,z0], [x1,y1,z1], ...] et anime le robot"""
        pts = np.array(points)
        self.robot_line.set_data(pts[:, 0], pts[:, 1])
        self.robot_line.set_3d_properties(pts[:, 2])
        self.draw_idle() # Rafraîchissement asynchrone ultra-rapide



class QDoubleSlider(QSlider):
    """Un QSlider personnalisé qui accepte et retourne des valeurs flottantes"""
    # Crée un signal personnalisé qui envoie un float au lieu d'un int
    doubleValueChanged = pyqtSignal(float)

    def __init__(self, orientation, parent=None, decimals=2):
        super().__init__(orientation, parent)
        self.decimals = decimals
        self.factor = 10 ** self.decimals
        self.valueChanged.connect(self.on_value_changed)

    def on_value_changed(self, value):
        # Convertit l'entier interne en float haute précision
        self.doubleValueChanged.emit(value / self.factor)

    def setFloatRange(self, min_val, max_val):
        self.setRange(int(min_val * self.factor), int(max_val * self.factor))

    def floatValue(self):
        return self.value() / self.factor

    def setFloatValue(self, value):
        self.setValue(int(value * self.factor))
        
    def floatMinimum(self):
        """Retourne la borne minimale en float pour le clamping"""
        return self.minimum() / self.factor

    def floatMaximum(self):
        """Retourne la borne maximale en float pour le clamping"""
        return self.maximum() / self.factor    
    


class RobotControllerSignals(QWidget):
    """Classe dédiée à la gestion des signaux pour la communication avec le robot"""
    angles_changed = pyqtSignal(list)       
    cartesian_changed = pyqtSignal(dict)    
    tool_command = pyqtSignal(str, dict)    
    emergency_stop = pyqtSignal()           
    Redemarre = pyqtSignal()
    begin_simulation = pyqtSignal(list)


class RobotControlGUI(QMainWindow):
    def __init__(self, robot , physicClient = None):
        super().__init__()
        self.robot = robot
        self.physicClient = physicClient
        self.signals = RobotControllerSignals()
        self.init_ui()
        self.connect_events()
        
        self.viewer = None 
        self.calculate_only = False
        
        # Premier affichage de la structure 3D au démarrage
        self.is_animating = False
        self.is_manipulating = False
        self.control_mode = 0 # O pour IHM et 1 pour pybullet
        self.reset_to_home()
        
        
    def init_ui(self):
        self.setWindowTitle("Supervision & Contrôle - Bras Robot 5 Axes")
        self.resize(1000, 750) # Légèrement agrandi pour le confort visuel de la 3D
        self.apply_dark_theme()

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setSpacing(15)

        # ==========================================
        # PANNEAU GAUCHE : ANIMATION 3D MATPLOTLIB + SLIDERS COMPACTS
        # ==========================================
        left_panel = QVBoxLayout()
        self.control_group = QGroupBox("Visualisation & Articulations")
        control_layout = QVBoxLayout(self.control_group)
        
        # Réintégration directe du canevas Matplotlib d'origine (Sans sélecteur ni stack)
        self.canvas_3d = MTD3DCanvas(self, width=5, height=4, dpi=100)
        control_layout.addWidget(self.canvas_3d, stretch=3)
        
        control_layout.addWidget(QLabel("<b>Commandes Articulaires :</b>"))

        
        # 2. DESIGN DES SLIDERS COMPACTS (Grille sur deux colonnes)
        sliders_grid = QGridLayout()
        sliders_grid.setSpacing(8)
        
        
        self.sliders = []
        self.angle_labels = []
        # limites_axes = [(-180, 180), (-180, 180), (-180, 180), (-180, 180), (-180, 180)]
        limites_axes = list(np.rad2deg(self.robot.limits))
        
        for i in range(5):
            row_idx = i // 2
            col_idx = (i % 2) * 3 
            
            lbl_name = QLabel(f"<b>A{i+1}:</b>")
            lbl_name.setFixedWidth(30)
                
            # Utilisation du nouveau Slider flottant
            slider = QDoubleSlider(Qt.Horizontal, decimals=2)
            slider.setFloatRange(limites_axes[i][0], limites_axes[i][1])
            slider.setFloatValue(0.0)
            slider.setFixedHeight(18) 
                
            lbl_val = QLabel("0.00°") # Format d'affichage étendu
            lbl_val.setFixedWidth(45) # Élargi pour les décimales
            lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                
            sliders_grid.addWidget(lbl_name, row_idx, col_idx)
            sliders_grid.addWidget(slider, row_idx, col_idx + 1)
            sliders_grid.addWidget(lbl_val, row_idx, col_idx + 2)
                
            self.sliders.append(slider)
            self.angle_labels.append(lbl_val)


        # Pour le 5ème axe (Axe 5), on le fait s'étendre sur la deuxième moitié ou on le centre
        # Ici il occupe naturellement la ligne 2, colonne de gauche. On peut mettre le bouton Home à sa droite !
        
        self.btn_home = QPushButton("Home")
        self.btn_home.setFixedHeight(24)
        sliders_grid.addWidget(self.btn_home, 2, 4, 1, 2) # Place vacante à côté de l'Axe 5

        control_layout.addLayout(sliders_grid, stretch=1)
        left_panel.addWidget(self.control_group)
        main_layout.addLayout(left_panel, stretch=5)

        # ==========================================
        # PANNEAU DROIT : CARTÉSIEN, EFFECTEUR & SÉCURITÉ
        # ==========================================
        right_panel = QVBoxLayout()

        self.cartesian_group = QGroupBox("Cinématique Inverse (XYZ / Orientation)")
        cartesian_layout = QGridLayout(self.cartesian_group)
        
        
        self.spins = {}
        coords = [("X (mm)", -200, 200), ("Y (mm)", -200, 200), ("Z (mm)", 0, 300),
                  ("Pitch (°)", -90, 90), ("Roll (°)", -180, 180)]
        
        for idx, (name, min_v, max_v) in enumerate(coords):
            lbl = QLabel(name)
            spin = QDoubleSpinBox()
            spin.setRange(min_v, max_v)
            spin.setDecimals(1)
            cartesian_layout.addWidget(lbl, idx, 0)
            cartesian_layout.addWidget(spin, idx, 1)
            self.spins[name.split()[0]] = spin
            
        self.btn_move_xyz = QPushButton("Calculer & Exécuter la Trajectoire")
        self.btn_move_xyz.setObjectName("ActionBtn")
        cartesian_layout.addWidget(self.btn_move_xyz, len(coords), 0, 1, 2)
        right_panel.addWidget(self.cartesian_group)

        # --- SÉLECTEUR DE MODE DE PILOTAGE ---
        mode_group = QGroupBox("Mode de Pilotage")
        mode_layout = QHBoxLayout(mode_group)
        
        self.combo_control_mode = QComboBox()
        self.combo_control_mode.addItems(["Contrôle via IHM (Sliders/XYZ)", "Contrôle via PyBullet (Souris 3D)"])
        mode_layout.addWidget(self.combo_control_mode)
        right_panel.addWidget(mode_group)


        # Configuration de l'Effecteur
        self.effector_group = QGroupBox("Configuration de l'Effecteur")
        effector_layout = QVBoxLayout(self.effector_group)
        self.combo_tool = QComboBox()
        self.combo_tool.addItems(["Pince Électrique", "Ventouse à Vide"])
        effector_layout.addWidget(self.combo_tool)

        self.tool_stack = QStackedWidget()
        self.pince_widget = QWidget()
        pince_lay = QHBoxLayout(self.pince_widget)
        self.slider_pince = QSlider(Qt.Horizontal)
        self.slider_pince.setRange(0, 100)
        pince_lay.addWidget(QLabel("Ouverture :"))
        pince_lay.addWidget(self.slider_pince)
        
        self.ventouse_widget = QWidget()
        ventouse_lay = QHBoxLayout(self.ventouse_widget)
        self.btn_ventouse = QPushButton("Activer l'Aspiration")
        self.btn_ventouse.setCheckable(True)
        ventouse_lay.addWidget(self.btn_ventouse)

        self.tool_stack.addWidget(self.pince_widget)
        self.tool_stack.addWidget(self.ventouse_widget)
        effector_layout.addWidget(self.tool_stack)
        right_panel.addWidget(self.effector_group)


        # ==========================================
        # ZONE DE DÉPÔT / IMPORTATION G-CODE
        # ==========================================
        gcode_group = QGroupBox("Programme G-Code")
        gcode_layout = QVBoxLayout(gcode_group)
        
        # Étiquette affichant le fichier actuellement chargé
        self.lbl_gcode_status = QLabel("Aucun fichier chargé (*.gcode, *.nc)")
        self.lbl_gcode_status.setWordWrap(True)
        self.lbl_gcode_status.setStyleSheet("color: #a0a0ab; font-style: italic; font-size: 10px;")
        
        # Bouton d'importation de fichier
        self.btn_import_gcode = QPushButton("📁 Charger un fichier G-Code")
        self.btn_import_gcode.setFixedHeight(30)
        
        gcode_layout.addWidget(self.btn_import_gcode)
        gcode_layout.addWidget(self.lbl_gcode_status)
        right_panel.addWidget(gcode_group)


        # Arrêt d'Urgence
        self.btn_estop = QPushButton("ARRÊT D'URGENCE")
        self.btn_estop.setObjectName("EmergencyBtn")
        right_panel.addWidget(self.btn_estop)
        
        # Redemarrage
        self.btn_red = QPushButton("Redemarré")
        self.btn_red.setObjectName("Redemarrer")
        right_panel.addWidget(self.btn_red) 

        main_layout.addLayout(right_panel, stretch=3)
        
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("Système prêt. En attente de commandes.")

        self.lbl_webcam = QLabel("Caméra déconnectée")
        self.lbl_webcam.setAlignment(Qt.AlignCenter)
        self.lbl_webcam.setStyleSheet("background-color: #222831; border: 1px solid #3a3a44; min-height: 200px;")
        right_panel.addWidget(self.lbl_webcam)

        
        self.is_animating = False 


        
    def connect_events(self):
        for idx, slider in enumerate(self.sliders):
            # Attention : on utilise doubleValueChanged au lieu de valueChanged
            slider.doubleValueChanged.connect(lambda val, i=idx: self.on_joint_slider_moved(i, val))
        
        self.combo_tool.currentIndexChanged.connect(self.tool_stack.setCurrentIndex)
        self.btn_home.clicked.connect(self.reset_to_home)
        self.btn_move_xyz.clicked.connect(self.send_cartesian_target)
        self.btn_estop.clicked.connect(self.trigger_emergency)
        self.btn_red.clicked.connect(self.redemarre)
        self.slider_pince.sliderReleased.connect(self.send_tool_command)
        self.btn_ventouse.clicked.connect(self.send_tool_command)
        self.combo_control_mode.currentIndexChanged.connect(self.set_control_mode)
        self.btn_import_gcode.clicked.connect(self.select_gcode_file)


        self.cam_thread = CameraThread(camera_index=0)
        self.cam_thread.frame_received.connect(self.update_webcam_frame)
        self.cam_thread.start()


    def set_control_mode(self, index):
        """Bascule le mode de contrôle et gère l'activation des widgets"""
        self.control_mode = index
        if index == 0:
            self.statusBar.showMessage("🎮 Mode IHM activé : Contrôle par sliders et XYZ.")
            self.control_group.setEnabled(True)   # Active les sliders
            self.cartesian_group.setEnabled(True) # Active les cases XYZ
        else:
            self.statusBar.showMessage("🖱️ Mode Simulation activé : Manipulez le robot à la souris dans PyBullet.")
            self.control_group.setEnabled(False)  # Désactive les sliders pour éviter les conflits
            self.cartesian_group.setEnabled(False)# Désactive le XYZ        
    
    # ==========================================
    # LOGIQUE DES pyqtSLOTS ET ENVOI DE DONNÉES
    # ==========================================
    
    def on_joint_slider_moved(self, index, value):
        if hasattr(self, 'control_mode') and self.control_mode == 1:
            return # Sécurité : Interdit en mode PyBullet
            
        self.angle_labels[index].setText(f"{value:.2f}°")
        if hasattr(self, 'is_animating') and not self.is_animating:
            self.current_robot_angles = np.array([np.deg2rad(s.floatValue()) for s in self.sliders])
            
        current_angles_deg = [s.floatValue() for s in self.sliders]
        self.updatePosition()

        if hasattr(self, 'viewer') and self.viewer is not None:
            self.viewer.update_Simulation(current_angles_deg)


        
    @pyqtSlot(QImage)
    def update_webcam_frame(self, qt_image):
        """Met à jour l'affichage de la caméra en adaptant la taille au conteneur"""
        if self.isEnabled(): # Optionnel : coupe l'affichage si arrêt d'urgence activé
            pixmap = QPixmap.fromImage(qt_image)
            # Redimensionne l'image proportionnellement sans la déformer
            scaled_pixmap = pixmap.scaled(self.lbl_webcam.width(), self.lbl_webcam.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.lbl_webcam.setPixmap(scaled_pixmap)


    def reset_to_home(self):
        for slider in self.sliders:
            slider.blockSignals(True)
            slider.setValue(0)
            slider.blockSignals(False)
            
        for lbl in self.angle_labels:
            lbl.setText("0°")
            
        self.updatePosition()
        if hasattr(self, 'viewer') and self.viewer is not None:
            current_angles_deg = [float(s.floatValue()) for s in self.sliders]
            self.viewer.update_Simulation(current_angles_deg)
        self.statusBar.showMessage("Commande renvoyée : Retour à la position Home.")

    def send_cartesian_target(self):
        
        self.last_cartesian_target = {k: spin.value() for k, spin in self.spins.items()}
        self.calculate_only = False 
                
        self.updateSlider()
        # # 3. On demande la validation physique au jumeau numérique PyBullet
        # if hasattr(self, 'viewer') and self.viewer is not None and hasattr(self , "angles_cible_absolus"):
        #     angles_cible_deg = [np.rad2deg(a) for a in self.angles_cible_absolus]
        #     self.statusBar.showMessage("⏳ Validation de la trajectoire par le jumeau numérique...")
        #     # self.viewer.simulate_motion(angles_cible_deg)
            




    def send_tool_command(self):
        tool_type = self.combo_tool.currentText()
        if tool_type == "Pince Électrique":
            params = {"ouverture": self.slider_pince.value()}
        else:
            params = {"aspiration": self.btn_ventouse.isChecked()}
            self.btn_ventouse.setText("Aspiration ACTIVE" if params["aspiration"] else "Activer l'Aspiration")
        self.signals.tool_command.emit(tool_type, params)


    def trigger_emergency(self):
        """Déclenche l'arrêt d'urgence de manière ultra-sécurisée sans crash"""
        try:
            # 1. Émission du signal d'urgence
            self.signals.emergency_stop.emit()
            self.statusBar.showMessage("❌ COUPE-CIRCUIT ACTIVÉ : Arrêt immédiat du robot !")
            
            # 2. Désactivation des panneaux de contrôle
            if hasattr(self, 'cartesian_group'): self.cartesian_group.setEnabled(False)
            if hasattr(self, 'control_group'): self.control_group.setEnabled(False)
            if hasattr(self, 'effector_group'): self.effector_group.setEnabled(False)
            
            # 3. Gestion des boutons de sécurité
            self.btn_estop.setEnabled(False)
            self.btn_red.setEnabled(True)
            
            # 4. Arrêt de la caméra (Encapsulé pour éviter le crash fatal de PyQt)
            if hasattr(self, 'cam_thread') and self.cam_thread.isRunning(): 
                # Déconnecter temporairement le signal pour éviter qu'une frame résiduelle ne crash
                try:
                    self.cam_thread.frame_received.disconnect(self.update_webcam_frame)
                except Exception:
                    pass # Déjà déconnecté
                self.cam_thread.stop()
                
        except Exception as e:
            # Si une erreur survient, elle s'affiche dans la barre d'état au lieu de fermer l'application
            self.statusBar.showMessage(f"⚠️ Erreur durant l'arrêt d'urgence : {str(e)}")
            (f"Erreur Urgence Critique: {e}")


    def redemarre(self):
        """Réarme le système sans risque de plantage"""
        try:
            self.signals.Redemarre.emit()
            self.statusBar.showMessage("🔄 Réarmement du système : Redémarrage du robot...")
            
            # 1. Réactivation des panneaux
            if hasattr(self, 'effector_group'): self.effector_group.setEnabled(True)
            if hasattr(self, 'cartesian_group'): self.cartesian_group.setEnabled(True)
            if hasattr(self, 'control_group'): self.control_group.setEnabled(True)
            
            # 2. Gestion des boutons
            self.btn_estop.setEnabled(True)
            self.btn_red.setEnabled(False)
            
            # 3. Relance de la caméra proprement
            if hasattr(self, 'cam_thread'):
                if not self.cam_thread.isRunning():
                    # Reconnecter le signal avant de démarrer
                    self.cam_thread.frame_received.connect(self.update_webcam_frame)
                    self.cam_thread.start()
                    
        except Exception as e:
            self.statusBar.showMessage(f"⚠️ Erreur durant le redémarrage : {str(e)}")
            (f"Erreur Redémarrage Critique: {e}")



    # cette methode vise a verifier le mouvement du robot dans l interface pybullet 
    # mais n est pas utiliser pour l instant 
    
    @pyqtSlot(dict)
    def on_simulation_finished(self, result):
        """Reçoit le verdict de PyBullet. Si c'est valide, on exécute enfin le mouvement !"""
        if result["valid"]:
            self.statusBar.setStyleSheet("background-color: #222831; color: #2ecc71;")
            self.statusBar.showMessage("✅ Trajectoire validée ! Exécution du mouvement...")
            
            # 1. Le jumeau numérique donne son feu vert : 
            # On coupe le mode "calcul seul" et on relance updateSlider pour démarrer le QTimer
            self.calculate_only = False
            self.updateSlider() 
            
        else:
            # En cas de collision ou couple trop grand détecté par PyBullet
            self.statusBar.setStyleSheet("background-color: #222831; color: #e74c3c;")
            self.statusBar.showMessage("❌ Mouvement refusé : Risque de collision ou de surcharge !")
            
            # On force le robot virtuel à se réaligner sur la position actuelle (sécurité)
            self.updatePosition()
            
            # On débloque immédiatement le bouton cartésien pour l'opérateur
            self.btn_move_xyz.setEnabled(True)
            self.is_animating = False



    @pyqtSlot(list)
    def on_pybullet_mouse_moved(self, angles_deg):
        """Reçoit les angles mesurés depuis la simulation PyBullet (mouvement souris)"""
        # --- FILTRAGE DE SÉCURITÉ DE LA BOUCLE ---
        if (hasattr(self, 'is_animating') and self.is_animating):
            return
            
        # 1. Mise à jour graphique instantanée de tous les curseurs et étiquettes
        
        for idx, slider in enumerate(self.sliders):
            if idx < len(angles_deg) and angles_deg[idx] is not None:
                slider.blockSignals(True)  # Évite l'effet d'écho IHM -> PyBullet
                slider.setFloatValue(angles_deg[idx])
                self.angle_labels[idx].setText(f"{angles_deg[idx]:.2f}°")
                slider.blockSignals(False)
                
        # 2. Recalcule la position cartésienne réelle via le MGD
        self.updatePosition()


    # ==========================================
    # CALCULS CINÉMATIQUES & RAFRAÎCHISSEMENT 3D
    # ==========================================
    # Dans le __init__ de votre RobotControlGUI, pensez à initialiser :
    # self.current_robot_angles = np.zeros(5) 
    # self.is_animating = False

    def updateSlider(self, T_Pos=None):
        try:
            # 1. FIX SÉCURITÉ & VERROU
            self.btn_move_xyz.setEnabled(False)
            self.is_animating = True
            self.statusBar.showMessage("⏳ Calcul et planification du profil de vitesse fluide...")

            # Récupération de la cible cartésienne depuis l'IHM
            x = float(self.spins['X'].value())
            y = float(self.spins['Y'].value())
            z = float(self.spins['Z'].value())
            pitch = np.deg2rad(float(self.spins['Pitch'].value()))
            roll = np.deg2rad(float(self.spins['Roll'].value()))

            cp, sp = np.cos(pitch), np.sin(pitch)
            cr, sr = np.cos(roll), np.sin(roll)
            
            T_target = np.array([
                [cr, -sr * cp,  sr * sp, x],
                [sr,  cr * cp, -cr * sp, y],
                [0,   sp,       cp,      z],
                [0,   0,        0,       1]
            ], dtype=np.float64)
            
            # 2. Calcul unique des angles cibles par le MGI (Haute précision)
            angles_cible = LM.inverseKinematic6D(self.robot, T_target )
            self.angles_cible_absolus = angles_cible 
            # 3. Lecture de la position de départ réelle
            self.angles_depart_absolus = np.array([np.deg2rad(float(s.floatValue())) for s in self.sliders])
            
            # 4. Calcul de la distance articulaire maximale (en degrés)
            distance_max_deg = np.rad2deg(np.max(np.abs(angles_cible - self.angles_depart_absolus)))
            
            # Si le robot est déjà au point souhaité, on stoppe de suite
            if distance_max_deg < 0.1:
                self.appliquer_angles_ihm(angles_cible)
                self.btn_move_xyz.setEnabled(True)
                self.is_animating = False
                self.statusBar.showMessage("✅ Le robot est déjà à la position cible.")
                return

            # 5. Planification temporelle à fréquence fixe (Régulation matérielle)
            vitesse_deg_s = 40.0  # Vitesse de croisière maximale tolérée
            duree_totale_s = distance_max_deg / vitesse_deg_s
            
            # 40ms (25 Hz) : Fréquence idéale pour ne pas saturer le buffer de l'Arduino
            self.periode_ms = 40 
            self.anim_total_steps = int(duree_totale_s * (1000 / self.periode_ms))
            self.anim_total_steps = max(10, self.anim_total_steps) # Minimum 10 pas pour lisser
            
            # Initialisation du cache de transmission série pour filtrer le bruit mécanique
            self.last_sent_angles_deg = np.zeros(5)
            self.anim_current_step = 0
            
            if hasattr(self, 'anim_timer') and self.anim_timer.isActive():
                self.anim_timer.stop()
                
            from PyQt5.QtCore import QTimer
            self.anim_timer = QTimer(self)
            self.anim_timer.timeout.connect(self.executer_pas_trajectoire)
            self.anim_timer.start(self.periode_ms)

        except Exception as e:
            self.statusBar.showMessage(f"⚠️ Erreur MGI ou Trajectoire : {str(e)}")
            self.btn_move_xyz.setEnabled(True)
            self.is_animating = False


    def executer_pas_trajectoire(self):
        """Calcule l'état lissé des moteurs par profil de vitesse à chaque tick du Timer"""
        try:
            if self.anim_current_step < self.anim_total_steps:
                # Coefficient temporel normalisé progressant strictement de 0.0 à 1.0
                t = self.anim_current_step / (self.anim_total_steps - 1)
                
                # LOI COSINUSOÏDALE (Profil en S) : Accélération douce au départ, décélération au freinage.
                # Supprime les chocs mécaniques sur le robot réel.
                s = 0.5 * (1.0 - np.cos(np.pi * t))
                
                # Interpolation articulaire lissée (Garantit l'absence totale de dépassement)
                self.current_robot_angles = (1.0 - s) * self.angles_depart_absolus + s * self.angles_cible_absolus
                
                # Mise à jour de l'IHM graphique et du viewer PyBullet
                self.appliquer_angles_ihm(self.current_robot_angles)
                
                # --- FILTRE ET RÉGULATION SÉRIE POUR LE ROBOT RÉEL ---
                current_angles_deg = np.array([s.floatValue() for s in self.sliders])
                diff_moteurs = np.abs(current_angles_deg - self.last_sent_angles_deg)
                
                # Bande morte de 0.15°. Si le micro-déplacement est invisible, 
                # on n'envoie pas la trame pour économiser le processeur du robot.
                if np.any(diff_moteurs > 0.15):
                    self.signals.angles_changed.emit(current_angles_deg.tolist())
                    self.last_sent_angles_deg = current_angles_deg.copy()
                
                self.anim_current_step += 1
                
            else:
                # ARRIVÉE À DESTINATION
                self.anim_timer.stop()
                self.appliquer_angles_ihm(self.angles_cible_absolus)
                
                # Envoi final forcé de calage précis sur la cible absolue
                current_angles_deg = [s.floatValue() for s in self.sliders]
                self.signals.angles_changed.emit(current_angles_deg)
                
                self.is_animating = False
                self.btn_move_xyz.setEnabled(True)
                self.statusBar.showMessage("✅ Trajectoire articulaire fluide exécutée avec succès.")

        except Exception as e:
            if hasattr(self, 'anim_timer'):
                self.anim_timer.stop()
            self.is_animating = False
            self.btn_move_xyz.setEnabled(True)
            print(f"Erreur pas trajectoire: {e}")


    def appliquer_angles_ihm(self, angles_rad):
        """Met à jour graphiquement l'IHM et PyBullet sans doubler l'envoi série"""
        if hasattr(self.robot, 'current_angle'):
            self.robot.current_angle = angles_rad

        for idx, slider in enumerate(self.sliders):
            if idx < len(angles_rad):
                slider.blockSignals(True)
                angle_deg = np.rad2deg(angles_rad[idx])
                angle_clamped = max(slider.floatMinimum(), min(slider.floatMaximum(), angle_deg))
                
                slider.setFloatValue(angle_clamped)
                self.angle_labels[idx].setText(f"{slider.floatValue():.2f}°")
                slider.blockSignals(False)
        
        # Rafraîchissement synchrone de votre jumeau numérique / squelette 3D
        self.draw_robot_animation(angles_rad)
        
        # Transmission directe vers la simulation autonome PyBullet
        if hasattr(self, 'viewer') and self.viewer is not None:
            angles_deg = [np.rad2deg(a) for a in angles_rad]
            self.viewer.update_Simulation(angles_deg)
            
        # IMPORTANT : On a retiré self.signals.angles_changed.emit() d'ici pour que seul 
        # le filtre régulé à 25Hz de executer_pas_trajectoire gère l'envoi vers le vrai robot.


    def updatePosition(self):
        if hasattr(self, 'is_animating') and self.is_animating:
            return
            
        try:
            # Lecture directe en float
            current_angles_rad = [np.deg2rad(s.floatValue()) for s in self.sliders]
            
            T_ee = LM.ForwardKinematic(self.robot, current_angles_rad, joint=-1)
            
            X, Y, Z = T_ee[0, 3], T_ee[1, 3], T_ee[2, 3]
            
            pitch_rad = np.arctan2(T_ee[2, 1], T_ee[2, 2])
            
            roll_rad = np.arctan2(T_ee[1, 0], T_ee[0, 0])
            
            for spin in self.spins.values():
                spin.blockSignals(True)
                
            self.spins["X"].setValue(X)
            self.spins["Y"].setValue(Y)
            self.spins["Z"].setValue(Z)
            self.spins["Pitch"].setValue(np.clip(np.rad2deg(pitch_rad), -180, 180))
            self.spins["Roll"].setValue(np.clip(np.rad2deg(roll_rad), -180, 180))
            
            
            for spin in self.spins.values():
                spin.blockSignals(False)
            
            self.draw_robot_animation(current_angles_rad)
                
        except Exception as e:
            self.statusBar.showMessage(f"⚠️ Erreur MGD : {str(e)}")
            
            
    def draw_robot_animation(self, angles_rad):
        """Met à jour directement le squelette Matplotlib de l'IHM"""
        try:
            points_3d = [[0.0, 0.0, 0.0]]
            for j in range(self.robot.joint_nombre):
                T_joint = LM.ForwardKinematic(self.robot, angles_rad, joint=j)
                pos_x = T_joint[0, 3]
                pos_y = T_joint[1, 3]
                pos_z = T_joint[2, 3]
                points_3d.append([pos_x, pos_y, pos_z])
            
            self.canvas_3d.draw_robot(points_3d)
                
        except Exception as e:
            (f"Erreur d'affichage Matplotlib : {e}")



# =======================================================================
# AJOUT DES METHODES DE TRAITEMENTS DES FICHIERS GCODE 
# =======================================================================

    def select_gcode_file(self):
        """Ouvre un explorateur de fichiers natif pour sélectionner le G-Code"""
        from PyQt5.QtWidgets import QFileDialog
        
        options = QFileDialog.Options()
        # Ouvre la boîte de dialogue Windows/Linux filtrée sur les extensions G-code standards
        file_path, _ = QFileDialog.getOpenFileName(
            self, 
            "Sélectionner un fichier G-Code", 
            "", 
            "Fichiers G-Code (*.gcode *.nc *.txt);;Tous les fichiers (*)", 
            options=options
        )
        
        if file_path:
            # Récupère uniquement le nom du fichier (sans tout le chemin absolu) pour l'affichage
            import os
            file_name = os.path.basename(file_path)
            self.lbl_gcode_status.setText(f"Fichier actif : <b>{file_name}</b>")
            
            # Lance immédiatement la phase de pré-calcul en tâche de fond (Bufferisation)
            self.process_and_buffer_gcode(file_path)

    def process_and_buffer_gcode(self, file_path):
        """Traite le G-Code ligne par ligne et découpe chaque mouvement en profil lissé"""
        try:
            self.statusBar.showMessage("⏳ Analyse et lissage du G-Code en cours...")
            self.trajectory_buffer = [] # Réinitialisation du buffer d'angles
            
            # Initialisation du point de départ cartésien (lecture des positions actuelles du robot)
            angles_actuels = np.array([np.deg2rad(s.floatValue()) for s in self.sliders])
            T_current = LM.ForwardKinematic(self.robot, angles_actuels, joint=-1)
            last_x, last_y, last_z = T_current[0, 3], T_current[1, 3], T_current[2, 3]
            
            # Utilisation des orientations de saisie actuelles par défaut
            pitch = np.deg2rad(float(self.spins['Pitch'].value()))
            roll = np.deg2rad(float(self.spins['Roll'].value()))

            with open(file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith(';') or line.startswith('('):
                        continue # Ignore les commentaires G-code
                        
                    if line.startswith('G1') or line.startswith('G0'):
                        parts = line.split()
                        target_x, target_y, target_z = last_x, last_y, last_z
                        
                        # Extraction des axes XYZ du bloc de texte
                        for part in parts:
                            if part.startswith('X'): target_x = float(part[1:])
                            if part.startswith('Y'): target_y = float(part[1:])
                            if part.startswith('Z'): target_z = float(part[1:])
                        
                        # --- INTERPOLATION SÉCURISÉE ENTRE CHAQUE COORDONNÉE ---
                        distance_segment = np.linalg.norm(np.array([target_x, target_y, target_z]) - np.array([last_x, last_y, last_z]))
                        if distance_segment < 0.1:
                            continue
                            
                        # Vitesse max imposée pour découper le temps de chaque ligne G-code (ex: 30 mm/s)
                        vitesse_mm_s = 30.0
                        duree_segment_s = distance_segment / vitesse_mm_s
                        
                        # Échantillonnage à 25 Hz (Toutes les 40ms) pour coller aux performances de l'Arduino
                        periode_ms = 40
                        pas_segment = max(4, int((duree_segment_s * 1000) / periode_ms))
                        
                        # Évolution temporelle lissée en Cosinus (Profil de vitesse sans chocs)
                        for k in range(pas_segment):
                            t = k / (pas_segment - 1)
                            s = 0.5 * (1.0 - np.cos(np.pi * t)) # Loi d'accélération douce
                            
                            # Calcul de la coordonnée spatiale rectiligne intermédiaire
                            inter_x = (1.0 - s) * last_x + s * target_x
                            inter_y = (1.0 - s) * last_y + s * target_y
                            inter_z = (1.0 - s) * last_z + s * target_z
                            
                            # Construction de la matrice homogène
                            cp, sp = np.cos(pitch), np.sin(pitch)
                            cr, sr = np.cos(roll), np.sin(roll)
                            T_step = np.array([
                                [cr, -sr * cp,  sr * sp, inter_x],
                                [sr,  cr * cp, -cr * sp, inter_y],
                                [0,   sp,       cp,      inter_z],
                                [0,   0,        0,       1]
                            ], dtype=np.float64)
                            
                            # MGI avec initialisation chaude (Warm-Start) sur le point précédent
                            angles_actuels = LM.inverseKinematic6D(self.robot, T_step, init_estimation=angles_actuels)
                            self.trajectory_buffer.append(angles_actuels.copy())
                            
                        # Enregistrement du point d'arrivée comme point de départ de la ligne suivante
                        last_x, last_y, last_z = target_x, target_y, target_z

            self.statusBar.showMessage(f"✅ Trajectoire G-Code compilée : {len(self.trajectory_buffer)} étapes prêtes.", 5000)
            
            # Modification dynamique du bouton principal de trajectoire pour devenir l'exécuteur du G-Code
            self.btn_move_xyz.setText("▶️ Exécuter le G-Code")
            # On déconnecte l'ancien événement et on connecte la lecture du buffer
            try: self.btn_move_xyz.clicked.disconnect()
            except: pass
            self.btn_move_xyz.clicked.connect(self.execute_buffer_trajectory)
            
        except Exception as e:
            self.statusBar.showMessage(f"❌ Échec de la compilation du fichier : {str(e)}")

    def execute_buffer_trajectory(self):
        """Lance l'exécution pas à pas cadencée du buffer sans latence processeur"""
        if not hasattr(self, 'trajectory_buffer') or len(self.trajectory_buffer) == 0:
            return
            
        self.btn_move_xyz.setEnabled(False)
        self.btn_import_gcode.setEnabled(False)
        self.is_animating = True
        
        self.anim_current_step = 0
        self.anim_total_steps = len(self.trajectory_buffer)
        self.last_sent_angles_deg = np.zeros(5)

        if hasattr(self, 'anim_timer') and self.anim_timer.isActive():
            self.anim_timer.stop()
            
        from PyQt5.QtCore import QTimer
        self.anim_timer = QTimer(self)
        self.anim_timer.timeout.connect(self.executer_pas_buffer)
        self.anim_timer.start(40) # Cadencement mécanique régulé à 25 Hz (40 ms)

    def executer_pas_buffer(self):
        """Déroule le buffer, filtre le bruit mécanique et envoie à l'Arduino à 25Hz"""
        try:
            if self.anim_current_step < self.anim_total_steps:
                angles_rad = self.trajectory_buffer[self.anim_current_step]
                
                # Mise à jour synchronisée de l'IHM et de PyBullet (zéro calcul lourd)
                self.appliquer_angles_ihm(angles_rad)
                
                # Filtre de bande morte matériel
                current_angles_deg = np.array([s.floatValue() for s in self.sliders])
                diff = np.abs(current_angles_deg - self.last_sent_angles_deg)
                
                if np.any(diff > 0.15):
                    self.signals.angles_changed.emit(current_angles_deg.tolist())
                    self.last_sent_angles_deg = current_angles_deg.copy()
                    
                self.anim_current_step += 1
            else:
                self.anim_timer.stop()
                self.is_animating = False
                self.btn_move_xyz.setEnabled(True)
                self.btn_import_gcode.setEnabled(True)
                self.statusBar.showMessage("✅ Exécution du fichier G-Code terminée.")
                
                # Rétablir le bouton en mode manuel classique
                self.btn_move_xyz.setText("Calculer & Exécuter la Trajectoire")
                self.btn_move_xyz.clicked.disconnect()
                self.btn_move_xyz.clicked.connect(self.send_cartesian_target)
                
        except Exception as e:
            if hasattr(self, 'anim_timer'): self.anim_timer.stop()
            self.btn_move_xyz.setEnabled(True)
            self.btn_import_gcode.setEnabled(True)
            print(f"Erreur d'exécution du buffer : {e}")


    def closeEvent(self, event):
        """S'assure que la caméra se coupe si l'utilisateur ferme la fenêtre"""
        if hasattr(self, 'cam_thread'):
            self.cam_thread.stop()
        event.accept()
        

    def apply_dark_theme(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #1e1e24; color: #ffffff; }
            QGroupBox { border: 2px solid #3a3a44; border-radius: 8px; margin-top: 1ex; font-weight: bold; color: #00adb5; }
            QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; }
            QLabel { color: #eeeeee; }
            QSlider::groove:horizontal { height: 4px; background: #393e46; }
            QSlider::handle:horizontal { background: #00adb5; width: 12px; margin: -4px 0; border-radius: 6px; }
            QPushButton { background-color: #393e46; color: white; border-radius: 4px; padding: 6px; font-weight: bold; }
            QPushButton:hover { background-color: #4b525d; }
            QPushButton:pressed { background-color: #00adb5; }
            QDoubleSpinBox, QComboBox { background-color: #222831; border: 1px solid #3a3a44; color: white; border-radius: 4px; }
            QPushButton#ActionBtn { background-color: #00adb5; color: #222831; }
            QPushButton#ActionBtn:hover { background-color: #00fff5; }
            QPushButton#EmergencyBtn { background-color: #d63031; color: white; font-size: 14px; padding: 12px; }
            QPushButton#EmergencyBtn:hover { background-color: #ff7675; }
            QStatusBar { background-color: #222831; color: #00adb5; }
        """)





if __name__ == "__main__":
    nb_joint = 5
    phis = [np.pi/2, 0, 0, np.pi/2, 0]
    r = [0, 200, 100, 0, 10]
    d = [50, 0, 0, 10, 0]

    # 1. Modèle mathématique
    limites_standards = np.array([
        [-np.pi,           np.pi],           # Axe 1 : Base (-180° à +180°)
        [-np.pi/2,         np.pi/2],         # Axe 2 : Épaule (-90° à +90°)
        [-2*np.pi/3,       2*np.pi/3],       # Axe 3 : Coude (-120° à +120°)
        [-11*np.pi/18,     11*np.pi/18],     # Axe 4 : Poignet (-110° à +110°)
        [-np.pi,           np.pi]            # Axe 5 : Outil / Torsion (-180° à +180°)
    ])
    robot5DoF = LM.Robot(nb_joint, phis, r, d ,limits=limites_standards )
    
    app = QApplication(sys.argv)
    
    # 2. Fenêtre Graphique
    gui = RobotControlGUI(robot=robot5DoF)
    physicClient = In.RobotViewer(gui)
    gui.viewer = physicClient
    
    # 3. Pont Matériel (Modifiez "COM3" ou "/dev/ttyUSB0" selon votre système)
    hardware = tr.RobotHardwareBridge(port="COM3", gui=gui , baudrate=115200)
    
    gui.show()
    
    # Sécurité à la fermeture
    sys.exit_on_close = app.exec_()
    
    hardware.close()
    physicClient.close_bullet()
    sys.exit(sys.exit_on_close)