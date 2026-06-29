import pybullet as p
import pybullet_data
import time 
from PyQt5.QtCore import pyqtSlot , pyqtSignal , QObject , QTimer
import numpy as np


class RobotViewer(QObject):
    
    # Signaux de communication bidirectionnels
    pybullet_angles_changed = pyqtSignal(list)
    simulation_result = pyqtSignal(dict)

    def __init__(self, gui, robot_urdf_path="robot5DoF.urdf", init_pos=[0, 0, 0.0]):
        super().__init__()
        self.robot_urdf = robot_urdf_path
        self.robot_pos = init_pos
        
        # FIX 1 : Orientation de base normale [0,0,0] pour correspondre au repère Matplotlib/D-H
        self.robot_orien = p.getQuaternionFromEuler([0, 0, 0])
        
        self.robot = None
        self.num_joints = 0
        self._init_pybullet()
        
        # Caractéristiques moteur
        self.max_force = 0.45
        self.max_velocity = 5.0
        
        # FIX 2 : Initialisation de la mémoire de référence obligatoire pour la simulation
        self.last_target = [0.0] * 5
        
        # TIMER POUR LIRE LA SOURIS (30 FPS)
        self.read_timer = QTimer(self)
        self.read_timer.timeout.connect(self.check_pybullet_mouse_movement)
        self.read_timer.start(33)

        self.connect_gui(gui=gui)
    
    def _init_pybullet(self):
        try:       
            p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.resetSimulation()
            p.setGravity(0, 0, -9.81)
            plane = p.loadURDF("plane.urdf")
            
            self.robot = p.loadURDF(self.robot_urdf, self.robot_pos, self.robot_orien, useFixedBase=True)
            self.num_joints = p.getNumJoints(self.robot)
            print(f"✅ URDF chargé avec succès ; Nombre de joints = {self.num_joints}")
            
        except Exception as e:
            print(f"⚠️ Erreur d'initialisation de pybullet : {e}") 
            
    def connect_gui(self, gui):
        """Liaison sémantique stricte avec les signaux de l'IHM principale"""
        gui.signals.angles_changed.connect(self.update_Simulation)
        gui.signals.emergency_stop.connect(self.progress_emergency_stop)
        gui.signals.Redemarre.connect(self.process_begin)
        
        # Connexion du signal personnalisé de l'IHM vers la méthode de validation physique
        gui.signals.begin_simulation.connect(self.simulate_motion)
        
        self.pybullet_angles_changed.connect(gui.on_pybullet_mouse_moved)
        self.simulation_result.connect(gui.on_simulation_finished)
                
    def progress_emergency_stop(self):
        print("🚨 Arrêt d'urgence appliqué sur le simulateur.")
    
    def process_begin(self):
        print("🔄 Réinitialisation et réarmement du simulateur.")
        
    def reset(self):
        p.disconnect()
        self._init_pybullet()
        
    def check_pybullet_mouse_movement(self):
        """Lit les angles manipulés à la souris et filtre les liaisons fixes"""
        if self.robot is None:
            return
            
        p.stepSimulation() 
        angles_deg = []
        
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot, i)
            # FIX 3 : Extraction stricte du type Revolute et reconstruction propre de la liste
            if joint_info[2] == p.JOINT_REVOLUTE:
                joint_state = p.getJointState(self.robot, i)
                angles_deg.append(np.rad2deg(joint_state[0]))
                
        # On émet la trame vers l'IHM uniquement si elle contient les 5 axes motorisés
        if len(angles_deg) == 5:
            self.pybullet_angles_changed.emit(angles_deg)

    @pyqtSlot(list)
    def update_Simulation(self, angles_deg):
        if self.robot is None:
            return

        joint_indices = []
        target_positions = []
        index = 0

        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot, i)
            if info[2] == p.JOINT_REVOLUTE and index < len(angles_deg):
                joint_indices.append(i)
                target_positions.append(np.deg2rad(angles_deg[index]))
                index += 1

        p.setJointMotorControlArray(
            bodyUniqueId=self.robot,
            jointIndices=joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=target_positions,
            targetVelocities=[self.max_velocity] * len(joint_indices),
            forces=[self.max_force] * len(joint_indices)
        )
        self.last_target = target_positions.copy()
        
    @pyqtSlot(list)
    def simulate_motion(self, target_angles_deg):
        """Simule la trajectoire de manière isolée sans altérer l'état courant de l'IHM"""
        if self.robot is None:
            return

        state_id = p.saveState()
        joint_indices = []

        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot, i)
            if info[2] == p.JOINT_REVOLUTE:
                joint_indices.append(i)

        target_angles = np.deg2rad(target_angles_deg)
        nb_points = 20
        trajectory = []
        torques_history = []
        velocities_history = []
        max_motor_torque = 0.45
        valid = True

        for k in range(1, nb_points + 1):
            alpha = k / nb_points
            
            # FIX 4 : Conversion explicite en tableaux numpy pour autoriser l'opération algébrique
            angles = np.array(self.last_target) + alpha * (target_angles - np.array(self.last_target))

            p.setJointMotorControlArray(
                bodyUniqueId=self.robot,
                jointIndices=joint_indices,
                controlMode=p.POSITION_CONTROL,
                targetPositions=angles.tolist(), # Reconversion en liste standard pour PyBullet
                targetVelocities=[5.0] * len(joint_indices),
                forces=[0.45] * len(joint_indices)
            )

            for _ in range(80):
                p.stepSimulation()
                time.sleep(1./240)
                
            current_positions = []
            current_velocities = []

            for j in joint_indices:
                state = p.getJointState(self.robot, j)
                current_positions.append(state[0])
                current_velocities.append(state[1])

            torques = p.calculateInverseDynamics(
                self.robot,
                current_positions,
                current_velocities,
                [0.0] * len(joint_indices)
            )

            trajectory.append(current_positions)
            velocities_history.append(current_velocities)
            torques_history.append(torques)

            if any(abs(t) > max_motor_torque for t in torques):
                valid = False
                break

            if len(p.getContactPoints(bodyA=self.robot)) > 0:
                valid = False
                break

        p.restoreState(state_id)
        p.removeState(state_id)

        if valid:
            self.last_target = target_angles.tolist()

        result = {
            "valid": valid,
            "trajectory": np.rad2deg(np.array(trajectory)).tolist() if valid and len(trajectory) > 0 else [],
            "velocities": velocities_history,
            "torques": torques_history
        }
        
        # FIX 5 : Utilisation du bon signal d'émission déclaré en haut (simulation_result)
        self.simulation_result.emit(result)
        
    def close_bullet(self):
        p.disconnect()
        print("Interface déconnectée et fermée avec succès.")










# class RobotViewer(QObject):
#     # NOUVEAU : Signal pour envoyer les angles lus dans PyBullet vers l'IHM
#     pybullet_angles_changed = pyqtSignal(list)
#     simulation_result = pyqtSignal(dict)

#     def __init__(self, gui , robot_urdf_path="robot5DoF.urdf", init_pos=[0, 0, 0]):
#         super().__init__()
#         self.robot_urdf = robot_urdf_path
#         self.robot_pos = init_pos
#         self.robot_orien = p.getQuaternionFromEuler([0, 0, 0])
        
#         self.robot = None
#         self.num_joints = 0
#         self._init_pybullet()
        
#         #Caracteristique moteur
#         self.max_force = 0.45
#         self.max_velocity = 5.0
        
        
#         # --- NOUVEAU : TIMER POUR LIRE LA SOURIS (30 FPS) ---
#         self.read_timer = QTimer(self)
#         self.read_timer.timeout.connect(self.check_pybullet_mouse_movement)
#         self.read_timer.start(33) # ~33ms = 30 HZ

#         self.connect_gui(gui=gui)
    
#     def _init_pybullet(self):
#         try :       
#             p.connect(p.GUI)
#             p.setAdditionalSearchPath(pybullet_data.getDataPath())
#             p.resetSimulation()
#             p.setGravity(0 , 0 , -9.81)
#             plane = p.loadURDF("plane.urdf")
            
#             self.robot = p.loadURDF(self.robot_urdf , self.robot_pos , self.robot_orien , useFixedBase = True )
#             print(f"Urdf chargé avec succes ; Nombre de joints = {p.getNumJoints(self.robot)}")
#             self.num_joints = p.getNumJoints(self.robot)
            
#         except Exception as e :
#             print(f"Erreur d'initialisation de pybullet : {e}") 
            

#     def connect_gui(self, gui):
#         """Reçoit l'instance de votre IHM et connecte les signaux dans les DEUX sens"""

#         gui.signals.angles_changed.connect(self.update_Simulation)
#         gui.signals.emergency_stop.connect(self.progress_emergency_stop)
#         gui.signals.Redemarre.connect(self.process_begin)
#         gui.signals.begin_simulation.connect(self.simulate_motion)
        
#         self.pybullet_angles_changed.connect(gui.on_pybullet_mouse_moved)
#         self.simulation_result.connect(gui.on_simulation_finished)
                
#     #code a ameliorer 
#     def progress_emergency_stop(self):
#         print("Robot en arret ")
    
#     #code a ameliorer 
#     def process_begin(self):
#         print("Redemarrage du robot")
        
    
#     def reset(self):
#         p.disconnect()
#         self._init_pybullet()
        
#     def check_pybullet_mouse_movement(self):
#         """Lit les angles dans PyBullet et les envoie à l'IHM si l'IHM n'est pas en train d'animer"""
#         if self.robot is None:
#             return
            
#         p.stepSimulation() # Indispensable pour rafraîchir l'interaction souris
        
#         angles_deg = [None] * self.num_joints
        
#         for i in range(self.num_joints):
#             joint_info = p.getJointInfo(self.robot, i)
#             if joint_info[2] == p.JOINT_REVOLUTE:
#                 # p.getJointState(body, jointIndex)[0] retourne la position actuelle en RADIANS
#                 joint_state = p.getJointState(self.robot, i)
#                 angle_rad = joint_state[0]
#                 angles_deg[i] = np.rad2deg(angle_rad)
                
#         # On émet la liste des 5 angles en degrés vers l'IHM
#         if len(angle_rad) == 5 :
#             self.pybullet_angles_changed.emit(angles_deg)
        


#     @pyqtSlot(list)
#     def update_Simulation(self, angles_deg):

#         if self.robot is None:
#             return

#         joint_indices = []
#         target_positions = []

#         index = 0

#         for i in range(self.num_joints):

#             info = p.getJointInfo(self.robot, i)

#             if info[2] == p.JOINT_REVOLUTE:

#                 joint_indices.append(i)

#                 target_positions.append(
#                     np.deg2rad(angles_deg[index])
#                 )

#                 index += 1

#         p.setJointMotorControlArray(
#             bodyUniqueId=self.robot,
#             jointIndices=joint_indices,
#             controlMode=p.POSITION_CONTROL,
#             targetPositions=target_positions,
#             targetVelocities=[self.max_velocity]*len(joint_indices),
#             forces=[self.max_force]*len(joint_indices)
#         )
        
#         self.last_target = target_positions
        
        
#     @pyqtSlot(list)
    
#     def simulate_motion(self, target_angles_deg):

#         if self.robot is None:
#             return None

#         # Sauvegarde de l'état actuel
#         state_id = p.saveState()

#         # Liste des articulations rotatives
#         joint_indices = []

#         for i in range(self.num_joints):
#             info = p.getJointInfo(self.robot, i)

#             if info[2] == p.JOINT_REVOLUTE:
#                 joint_indices.append(i)

#         target_angles = np.deg2rad(target_angles_deg)

#         nb_points = 20

#         trajectory = []
#         torques_history = []
#         velocities_history = []

#         max_motor_torque = 0.45
#         valid = True

#         for k in range(1, nb_points + 1):

#             alpha = k / nb_points

#             angles = self.last_target + alpha * (target_angles - self.last_target)

#             # Commande des moteurs
#             p.setJointMotorControlArray(
#                 bodyUniqueId=self.robot,
#                 jointIndices=joint_indices,
#                 controlMode=p.POSITION_CONTROL,
#                 targetPositions=angles,
#                 targetVelocities=[5.0] * len(joint_indices),
#                 forces=[0.45] * len(joint_indices)
#             )

#             for _ in range(80):
#                 p.stepSimulation()

#             current_positions = []
#             current_velocities = []

#             for j in joint_indices:
#                 state = p.getJointState(self.robot, j)

#                 current_positions.append(state[0])
#                 current_velocities.append(state[1])

#             torques = p.calculateInverseDynamics(
#                 self.robot,
#                 current_positions,
#                 current_velocities,
#                 [0.0] * len(joint_indices)
#             )

#             trajectory.append(current_positions)
#             velocities_history.append(current_velocities)
#             torques_history.append(torques)

#             # Vérification des couples
#             if any(abs(t) > max_motor_torque for t in torques):
#                 valid = False
#                 break

#             # Vérification des collisions
#             if len(p.getContactPoints(bodyA=self.robot)) > 0:
#                 valid = False
#                 break

#         # Retour à l'état initial
#         p.restoreState(state_id)
#         p.removeState(state_id)

#         # Si la trajectoire est valide,
#         # elle devient la nouvelle position de référence
#         if valid:
#             self.last_target = target_angles.copy()

#         result = {
#             "valid": valid,
#             "trajectory": np.rad2deg(np.array(trajectory)).tolist(),
#             "velocities": velocities_history,
#             "torques": torques_history
#         }
#         self.simulate_motion.emit(result)
        
#     def close_bullet(self):
#         p.disconnect()
#         print("interface fermé avec succes")

