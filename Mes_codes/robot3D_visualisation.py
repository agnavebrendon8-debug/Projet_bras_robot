import pybullet as p
import pybullet_data
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import QTimer



class PyBulletCanvas(QWidget):
    """Zone de simulation et d'affichage 3D basée sur PyBullet et le fichier URDF"""
    def __init__(self, parent=None, urdf_path="robot5dof.urdf"):
        super().__init__(parent)
        self.urdf_path = urdf_path
        self.robot_id = None
        
        # Initialisation de PyBullet en mode DIRECT (sans ouvrir de fenêtre externe)
        # Note : Si vous voulez que PyBullet gère sa propre fenêtre OpenGL, utilisez p.GUI
        p.connect(p.DIRECT) 
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Chargement du sol et du robot URDF
        self.plane_id = p.loadURDF("plane.urdf")
        self.load_robot()

    def load_robot(self):
        try:
            # On charge le robot à l'origine (0,0,0)
            self.robot_id = p.loadURDF(self.urdf_path, [0, 0, 0], useFixedBase=True)
            self.num_joints = p.getNumJoints(self.robot_id)
            print(f"✅ URDF chargé avec succès. Nombre d'articulations PyBullet : {self.num_joints}")
        except Exception as e:
            print(f"⚠️ Erreur lors du chargement de l'URDF : {e}")

    def draw_robot(self, angles_rad):
        """Met à jour la position des articulations du modèle URDF dans la simulation"""
        if self.robot_id is None:
            return
            
        # Filtrer et appliquer les angles uniquement sur les joints motorisés (type REVOLUTE)
        joint_idx = 0
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_type = joint_info[2]
            
            # Si le joint est de type Revolute (pivot) et qu'on ne dépasse pas les 5 axes
            if joint_type == p.JOINT_REVOLUTE and joint_idx < len(angles_rad):
                p.resetJointState(self.robot_id, i, angles_rad[joint_idx])
                joint_idx += 1
                
        p.stepSimulation()
