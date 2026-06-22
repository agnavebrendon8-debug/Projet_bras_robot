import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLabel, QComboBox, QDoubleSpinBox, QPushButton, QGridLayout, QGroupBox, QStackedWidget, QStatusBar
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot

# Intégration de Matplotlib dans PyQt5
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

import Leven_Marq as LM 

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

class RobotControllerSignals(QWidget):
    """Classe dédiée à la gestion des signaux pour la communication avec le robot"""
    angles_changed = pyqtSignal(list)       
    cartesian_changed = pyqtSignal(dict)    
    tool_command = pyqtSignal(str, dict)    
    emergency_stop = pyqtSignal()           
    Redemarre = pyqtSignal()

class RobotControlGUI(QMainWindow):
    def __init__(self, robot):
        super().__init__()
        self.robot = robot
        self.signals = RobotControllerSignals()
        self.init_ui()
        self.connect_events()
        # Premier affichage de la structure 3D au démarrage
        self.updatePosition()
        
    def init_ui(self):
        self.setWindowTitle("Supervision & Contrôle - Bras Robot 5 Axes")
        self.resize(1000, 750) # Légèrement agrandi pour le confort visuel de la 3D
        self.apply_dark_theme()

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setSpacing(15)

        # ==========================================
        # PANNEAU GAUCHE : ANIMATION 3D + SLIDERS COMPACTS
        # ==========================================
        left_panel = QVBoxLayout()
        
        # En-tête / Groupe Principal de gauche
        self.control_group = QGroupBox("Visualisation & Articulations")
        control_layout = QVBoxLayout(self.control_group)
        
        # 1. ZONE D'ANIMATION (Haut Gauche)
        self.canvas_3d = MTD3DCanvas(self, width=5, height=4, dpi=100)
        control_layout.addWidget(self.canvas_3d, stretch=3) # Donne de l'importance à la 3D
        
        # Separateur visuel léger ou petit titre
        control_layout.addWidget(QLabel("<b>Commandes Articulaires :</b>"))
        
        # 2. DESIGN DES SLIDERS COMPACTS (Grille sur deux colonnes)
        sliders_grid = QGridLayout()
        sliders_grid.setSpacing(8)
        
        self.sliders = []
        self.angle_labels = []
        limites_axes = [(-180, 180), (-90, 90), (-120, 120), (-180, 180), (-110, 110)]

        for i in range(5):
            # Calcul de la ligne et de la colonne pour compacter l'espace (2 axes par ligne)
            row_idx = i // 2
            col_idx = (i % 2) * 3 # Décale de 3 sous-colonnes (Label, Slider, Valeur)
            
            lbl_name = QLabel(f"<b>A{i+1}:</b>")
            lbl_name.setFixedWidth(30)
            
            slider = QSlider(Qt.Horizontal)
            slider.setRange(limites_axes[i][0], limites_axes[i][1])
            slider.setValue(0)
            slider.setFixedHeight(18) # Rendu plus fin
            
            lbl_val = QLabel("0°")
            lbl_val.setFixedWidth(35)
            lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            
            # Ajout à la grille compacte
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


    def connect_events(self):
        for idx, slider in enumerate(self.sliders):
            slider.valueChanged.connect(lambda val, i=idx: self.on_joint_slider_moved(i, val))
        
        self.combo_tool.currentIndexChanged.connect(self.tool_stack.setCurrentIndex)
        self.btn_home.clicked.connect(self.reset_to_home)
        self.btn_move_xyz.clicked.connect(self.send_cartesian_target)
        self.btn_estop.clicked.connect(self.trigger_emergency)
        self.btn_red.clicked.connect(self.redemarre)
        self.slider_pince.sliderReleased.connect(self.send_tool_command)
        self.btn_ventouse.clicked.connect(self.send_tool_command)

        self.cam_thread = CameraThread(camera_index=0)
        self.cam_thread.frame_received.connect(self.update_webcam_frame)
        self.cam_thread.start()
        
    
    # ==========================================
    # LOGIQUE DES pyqtSLOTS ET ENVOI DE DONNÉES
    # ==========================================
    def on_joint_slider_moved(self, index, value):
        self.angle_labels[index].setText(f"{value}°")
        current_angles = [s.value() for s in self.sliders]
        self.signals.angles_changed.emit(current_angles)
        self.updatePosition()

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
        self.statusBar.showMessage("Commande renvoyée : Retour à la position Home.")

    def send_cartesian_target(self):
        target = {k: spin.value() for k, spin in self.spins.items()}
        self.signals.cartesian_changed.emit(target)
        self.updateSlider()

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
            print(f"Erreur Urgence Critique: {e}")

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
            print(f"Erreur Redémarrage Critique: {e}")


    # ==========================================
    # CALCULS CINÉMATIQUES & RAFRAÎCHISSEMENT 3D
    # ==========================================
    def updateSlider(self):
        try:
            x = float(self.spins['X'].value())
            y = float(self.spins['Y'].value())
            z = float(self.spins['Z'].value())
            pitch = np.deg2rad(float(self.spins['Pitch'].value()))
            roll = np.deg2rad(float(self.spins['Roll'].value()))
            
            c_p, s_p = np.cos(pitch), np.sin(pitch)
            c_r, s_r = np.cos(roll), np.sin(roll)
            
            T_target = np.array([
                [c_p * c_r, -s_r,  s_p * c_r, x],
                [c_p * s_r,  c_r,  s_p * s_r, y],
                [-s_p,       0,     c_p,       z],
                [0,          0,     0,         1]
            ])
           
            angles_rad = LM.inverseKinematic6D(self.robot, T_target)
            
            for idx, slider in enumerate(self.sliders):
                if idx < len(angles_rad):
                    slider.blockSignals(True)
                    angle_deg = int(np.rad2deg(angles_rad[idx]))
                    angle_clamped = max(slider.minimum(), min(slider.maximum(), angle_deg))
                    slider.setValue(angle_clamped)
                    self.angle_labels[idx].setText(f"{angle_clamped}°")
                    slider.blockSignals(False)
            
            # Après avoir mis à jour les sliders, on rafraîchit l'animation 3D
            self.draw_robot_animation(angles_rad)
                    
        except Exception as e:
            self.statusBar.showMessage(f"⚠️ Erreur MGI : {str(e)}")

    def updatePosition(self):
        try:
            current_angles_rad = [np.deg2rad(float(s.value())) for s in self.sliders]
            
            # Récupération de la matrice finale pour l'IHM
            T_ee = LM.ForwardKinematic(self.robot, current_angles_rad, joint=-1)
            
            X, Y, Z = T_ee[0, 3], T_ee[1, 3], T_ee[2, 3]
            pitch_rad = -np.arcsin(T_ee[2, 0])
            roll_rad = np.arctan2(T_ee[2, 1], T_ee[2, 2])
            
            for spin in self.spins.values():
                spin.blockSignals(True)
                
            self.spins["X"].setValue(X)
            self.spins["Y"].setValue(Y)
            self.spins["Z"].setValue(Z)
            self.spins["Pitch"].setValue(np.clip(np.rad2deg(pitch_rad), -90, 90))
            self.spins["Roll"].setValue(np.clip(np.rad2deg(roll_rad), -180, 180))
            
            for spin in self.spins.values():
                spin.blockSignals(False)
            
            # Mise à jour du graphique 3D en envoyant les angles actuels
            self.draw_robot_animation(current_angles_rad)
                
        except Exception as e:
            self.statusBar.showMessage(f"⚠️ Erreur MGD : {str(e)}")

    def draw_robot_animation(self, angles):
        """Récupère les matrices de chaque articulation et envoie la liste des points 3D au Canvas"""
        try:
            points_3d = [[0.0, 0.0, 0.0]] # La base fixe du robot à l'origine (0,0,0)
            
            # On boucle pour récupérer la position de CHAQUE articulation (de 0 à 4)
            for j in range(self.robot.joint_nombre):
                T_joint = LM.ForwardKinematic(self.robot, angles, joint=j)
                pos_x = T_joint[0, 3]
                pos_y = T_joint[1, 3]
                pos_z = T_joint[2, 3]
                points_3d.append([pos_x, pos_y, pos_z])
                
            # Envoi des points collectés au canevas graphique pour tracé instantané
            self.canvas_3d.draw_robot(points_3d)
        except Exception as e:
            print(f"Erreur d'affichage du squelette 3D: {e}")

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
    r = [0, 200, 100, 0, 40]
    d = [50, 0, 0, 20, 0]

    robot5DoF = LM.Robot(nb_joint, phis, r, d)
    
    app = QApplication(sys.argv)
    gui = RobotControlGUI(robot=robot5DoF)
    gui.show()
    sys.exit(app.exec_())



# Insérer ces lignes dans votre méthode init_ui() [dans le panneau droit ou sous l'effecteur] :
# Insérer ces lignes à la fin de votre méthode connect_events() :
# Insérer cette ligne dans votre méthode trigger_emergency() :
