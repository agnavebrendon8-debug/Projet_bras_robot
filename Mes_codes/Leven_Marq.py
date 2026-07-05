
import numpy as np

  
def transformMatrice(phi, teta, r, d):
    co, so = np.cos(teta), np.sin(teta)
    cp, sp = np.cos(phi), np.sin(phi)
    return np.array([[co, -so*cp,  so*sp, r*co],
                     [so,  co*cp, -co*sp, r*so],
                     [0,   sp,     cp,    d   ],
                     [0,   0,      0,     1   ]], dtype=np.float64)

def ForwardKinematic(robot, angles, joint=-1):
    phis = robot.phis 
    r = robot.r
    d = robot.d
    T = np.eye(4)
    transforms = []
    
    for i in range(robot.joint_nombre):
        T = T @ transformMatrice(phis[i], angles[i], r[i], d[i])
        transforms.append(T.copy())
        
    if joint == -1:
        return T
    return transforms[np.clip(joint, 0, robot.joint_nombre - 1)]

class Robot:
    def __init__(self, joint_nombre, phis, r, d, limits=None , ):
        self.joint_nombre = joint_nombre
        self.phis = phis
        self.r = r
        self.d = d
        self.current_angle = None
        if limits is None:
            self.limits = np.array([[-np.pi, np.pi]] * joint_nombre)
        else:
            self.limits = np.array(limits)


def inverseKinematic6D(robot, target_T, init_epsilon=0.1, max_iter=150 , k=0.0001 , init_estimation=None) :
    num = robot.joint_nombre
    
    # Initialisation au centre des limites articulaires
    if init_estimation is None :
        estimation = np.mean(robot.limits, axis=1)
    else :
        estimation = init_estimation
    
    epsilonne = init_epsilon
    last_norm = float('inf')
    
    # parametre du gradient pour définir le nulle space 
    qi_max = robot.limits[:, 1] 
    qi_min = robot.limits[:, 0]
    qi_moy = (qi_max + qi_min)/2
    q_dif = (qi_max - qi_min)**2
    
    for iteration in range(max_iter):
        Z = [None] * num
        Posi = [None] * num
        
        for i in range(num):
            T = ForwardKinematic(robot, estimation, i)
            Z[i] = T[:3, 2]
            Posi[i] = T[:3, 3]
            
        T_ee = ForwardKinematic(robot, estimation, -1)
        p_ee = T_ee[:3, 3]
        R_ee = T_ee[:3, :3]
        
        p_target = target_T[:3, 3]
        R_target = target_T[:3, :3]
        
        # --- Calcul de l'erreur 6D ---
        err_pos = p_target - p_ee
        R_err = R_target @ R_ee.T
        err_ori = 0.5 * np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1]
        ])
        
        error = np.concatenate((err_pos, err_ori))
        current_norm = np.linalg.norm(error)
        
        # Condition de succès
        if current_norm < 1e-5:
            # print(f"-> Convergé en {iteration} itérations.")
            break
            
        # --- Stratégie Adaptive Levenberg-Marquardt ---
        if current_norm < last_norm:
            epsilonne /= 1.2  # L'erreur diminue, on réduit l'amortissement
        else:
            epsilonne *= 2.0  # L'erreur stagne/augmente, on sécurise le pas
            
        last_norm = current_norm
        
        # --- Construction du Jacobien Géométrique 6D ---
        J = np.zeros((6, num))
        for i in range(num):
            if i == 0:
                z_prev = np.array([0.0, 0.0, 1.0])   # A ajuster en fonction du premier joints du robots 
                p_prev = np.array([0.0, 0.0, 0.0])   
            else:
                z_prev = Z[i-1]
                p_prev = Posi[i-1]
                
            J[:3, i] = np.cross(z_prev, p_ee - p_prev)
            J[3:, i] = z_prev
            
        # --- Résolution du système ---
        JT = J.T
        lambda_sq = epsilonne ** 2
        A = JT @ J + lambda_sq * np.eye(num)
        b = JT @ error
        dq = np.linalg.solve(A, b)
        
        #----definition de l'espace nul et la minimisation par le gradient 
        J_inv = np.linalg.solve(A , J.T)
        Null_espace = np.eye(num) - J_inv @ J 
        grad_H = 2 * (estimation - qi_moy)/q_dif
        dq_grad =-k * grad_H
        dq_nul = Null_espace @ dq_grad
        
        
        # --- Mise à jour et Clamping ---
        estimation += dq + dq_nul
        estimation = np.clip(estimation, robot.limits[:, 0], robot.limits[:, 1])
        
    return estimation

# ==========================================
#          SCÉNARIO DE TEST (6 AXES)
# ==========================================
if __name__ == "__main__":
    # Définition d'un robot type "Stanford / Anthropomorphe" à 6 joints
    nb_joints = 6
    phis_robot = [0, np.pi/2, 0, -np.pi/2, np.pi/2, 0]
    r_robot    = [0, 0.4, 0.3, 0, 0, 0]
    d_robot    = [0.5, 0, 0, 0.4, 0, 0.1]
    
    # Limites mécaniques en radians
    limites_articulations = [
        [-np.pi, np.pi],        # Joint 1
        [-np.pi/2, np.pi/2],    # Joint 2
        [-np.pi/2, np.pi/2],    # Joint 3
        [-np.pi, np.pi],        # Joint 4
        [-np.pi/2, np.pi/2],    # Joint 5
        [-2*np.pi, 2*np.pi]     # Joint 6
    ]
    
    mon_robot = Robot(nb_joints, phis_robot, r_robot, d_robot, limites_articulations)
    
    # 1. Génération d'une pose cible valide via la cinématique directe
    angles_cibles_reels = np.array([0.5, -0.2, 0.4, 0.1, -0.3, 0.8])
    matrice_cible = ForwardKinematic(mon_robot, angles_cibles_reels)
    
    print("--- MATRICE CIBLE À ATTEINDRE ---")
    print(np.round(matrice_cible, 4))
    
    # 2. Calcul des angles requis par la cinématique inverse
    # print("\nCalcul de la cinématique inverse...")
    # k = float(input("entre la valeur de k = "))
    
    angles_trouves = inverseKinematic6D(mon_robot, matrice_cible)
    
    # 3. Vérification du résultat
    matrice_atteinte = ForwardKinematic(mon_robot, angles_trouves)
    erreur_finale = np.linalg.norm(matrice_cible[:3, 3] - matrice_atteinte[:3, 3])
    
    print("\n--- RÉSULTATS ---")
    print(f"Angles réels d'origine : {np.round(angles_cibles_reels, 3)}")
    print(f"Angles trouvés par l'IK : {np.round(angles_trouves, 3)}")
    print(f"Erreur de position finale : {erreur_finale:.6f} mètres")    
    print(f"Position final : {ForwardKinematic(mon_robot , angles_trouves)[:3 , 3]}")
    mon_robot.current_angle = angles_trouves
    
    

# Envoi des données via pyserial 
def send():
    pass



# import numpy as np 
    

# def transformMatrice(phi , teta , r , d ):
#     co , so = np.cos(teta) , np.sin(teta)
#     cp , sp = np.cos(phi) , np.sin(phi)
    
#     return np.array([[co , -so*cp , so*sp, r*co ],
#                      [so , co*cp , -co*sp , r*so],
#                      [0  ,   sp  ,    cp  ,  d  ],
#                      [0 ,    0  ,     0   , 1   ]] , dtype = np.float64 )
    

# def ForwardKinematic(robot , angles , joint = -1 , limits = None):

#     transforms = [None] * (robot.joint_nombre+1)
    
#     phis = robot.phis 
#     r = robot.r
#     d = robot.d
 
#     T = np.eye(4)
        
#     for i in range(robot.joint_nombre):
#         T = T @ transformMatrice(phis[i] , angles[i] ,r[i] , d[i] )
#         transforms[i] = T.copy()
    
#     transforms[-1] = T.copy() 

#     if joint == -1 :
#         return transforms[-1]
    
#     return transforms[np.clip(joint , 0 , robot.joint_nombre-1)]
    

# class joint():
    
#     def __init__(self , tab_DH , bool_phi:bool , bool_teta:bool ):
#         self.DH_parametre = tab_DH
#         self.phi = bool_phi 
#         self.teta = bool_teta



# class Robot :

#     def __init__(self, joint_nombre , phis , r , d ):
#         self.joint_nombre = joint_nombre
#         self.phis = phis
#         self.r = r
#         self.d = d
    
  
  


# def inverseKinematic(robot , target , epsilonne):
#     num = robot.joint_nombre

#     estimation = np.full(num , np.pi/5 , dtype=np.float32)
    
#     for iteration in range(100):
        
#         Z = [None] * num
#         Posi = [None] * num
        
#         for i in range(robot.joint_nombre):
       
#             T = ForwardKinematic(robot, estimation , i) # i pour retrouver la matrice de transformation sbsolu du joint i
       
#             Z[i] = T[:3, 2]
        
#             Posi[i] = T[:3,3]
                        
#         error = target - Posi[-1]
    
#         if np.linalg.norm(error) < 1e-6:
#             break
        
#         # Levenberg-Marquardt 
#         # Δ𝑞=(𝐽𝑇𝐽+𝜆2𝐼)−1𝐽𝑇𝑒Δq=(JTJ+λ2I)−1JTe
        
#         # Jacobien 
#         J = np.zeros((3, num))
        
#         for j in range(num):
#             J[: , j] = np.cross(Z[j], (Posi[-1] - Posi[j]))
            
#         A = J.T@J + epsilonne**2 * np.eye(num)
#         B = J.T @ error
        
#         dq = np.linalg.solve(A , B)
        
#         # New_estimation = estimation + dq
#         # new_error = np.linalg.norm(target - ForwardKinematic(robot, New_estimation)[:3 , 3])
#         # old_error = np.linalg.norm(error)
        
#         # if new_error < old_error:
#         #     estimation = New_estimation
#         #     epsilonne *= 0.7   # on fait confiance
        
#         # else:
#         #     epsilonne *= 2.0   # on stabilise
#         estimation += dq
        
#         for k in range(num):
#             print(Z[k])
#         # print( iteration , np.linalg.norm(error))
#         # print(f"rang = {np.linalg.matrix_rank(J)} et dq = {dq}")
    
#     return estimation

# phis = [0 , np.pi/2 , 0 , 0 ]
# # phis = [np.pi/2, np.pi/2, -np.pi/2, 0]
# r = [0 , 10 , 10 , 20]
# d = [10 , 0 , 0 ,0 ]

# robot = Robot(4 , phis , r , d )
# estimation = inverseKinematic(robot , np.array([5.0 , 10.0 , 10 ]), 0.01)
# position = ForwardKinematic(robot , estimation)

# print(f"Estimation = {estimation}")
# print(f"vérification = {position[:3 , 3]}")
