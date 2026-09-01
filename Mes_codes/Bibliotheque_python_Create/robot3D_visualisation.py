import sys
import os
import pybullet as p
import pybullet_data
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt , QTimer
from PyQt5.QtGui import QImage, QPixmap


class PyBulletCanvas(QWidget):
    def __init__(self, parent=None, urdf_path="robot5DoF.urdf"):
        super().__init__(parent)
        self.urdf_path = urdf_path
        self.robot_id = None
        
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.video_label = QLabel(self)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.video_label)
        
        # 1. Connexion PyBullet en mode DIRECT
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # 2. Chargement de l'environnement (Sol + Votre Robot)
        p.loadURDF("plane.urdf")
        try:
            self.robot_id = p.loadURDF(self.urdf_path, [0, 0, 0], useFixedBase=True)
            self.num_joints = p.getNumJoints(self.robot_id)
            print(f"✅ URDF chargé avec succès. Nombre d'articulations : {self.num_joints}")
        except Exception as e:
            print(f"⚠️ Erreur lors du chargement de l'URDF : {e}")
        
        # 3. Paramètres de la caméra virtuelle de capture (Zoomés sur le bras)
        self.cam_dist = 0.8
        self.cam_yaw = 50
        self.cam_pitch = -35
        self.cam_target = [0, 0, 0.1]
        self.render_w = 400
        self.render_h = 300

    def draw_robot(self, angles_rad):
        """Met à jour les moteurs PyBullet et capture l'image (Appelée par l'IHM)"""
        if self.robot_id is None:
            return
            
        # Mise à jour des articulations de type REVOLUTE
        joint_idx = 0
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            if joint_info[2] == p.JOINT_REVOLUTE and joint_idx < len(angles_rad):
                p.resetJointState(self.robot_id, i, angles_rad[joint_idx])
                joint_idx += 1
                
        p.stepSimulation()
        
        # Rendu de la caméra virtuelle
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=self.cam_target,
            distance=self.cam_dist,
            yaw=self.cam_yaw,
            pitch=self.cam_pitch,
            roll=0,
            upAxisIndex=2
        )
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=float(self.render_w)/self.render_h, nearVal=0.1, farVal=100.0
        )
        
        _, _, rgb_img, _, _ = p.getCameraImage(
            width=self.render_w,
            height=self.render_h,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        
        # Conversion et affichage identiques à votre script fonctionnel
        rgb_bytes = bytes(rgb_img)
        image = QImage(rgb_bytes, self.render_w, self.render_h, QImage.Format_RGBA8888)
        
        # Redimensionnement élastique pour s'adapter à la taille de l'IHM
        scaled_pixmap = QPixmap.fromImage(image).scaled(
            self.width(), self.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.video_label.setPixmap(scaled_pixmap)

    def closeEvent(self, event):
        p.disconnect()
        event.accept()
