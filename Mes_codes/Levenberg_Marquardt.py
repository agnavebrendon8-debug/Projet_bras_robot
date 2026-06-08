import numpy as np 
    

def transformMatrice(phi , teta , r , d ):
    co , so = np.cos(teta) , np.sin(teta)
    cp , sp = np.cos(phi) , np.sin(phi)
    
    return np.array([[co , -so*cp , so*sp, r*co ],
                     [so , co*cp , -co*sp , r*so],
                     [0  ,   sp  ,    cp  ,  d  ],
                     [0 ,    0  ,     0   , 1   ]] , dtype = np.float64 )
    

def ForwardKinematic(robot , angles , joint = -1):

    transforms = [None] * (robot.joint_nombre+1)
    
    phis = robot.phis 
    r = robot.r
    d = robot.d
 
    T = np.eye(4)
        
    for i in range(robot.joint_nombre):
        
        T = T @ transformMatrice(phis[i] , angles[i] ,r[i] , d[i] )
        transforms[i] = T.copy()
    
    transforms[-1] = T.copy() 
    if joint == -1 :
        return transforms[-1]
    
    return transforms[np.clip(joint , 0 , robot.joint_nombre-1)]
    

class joint():
    
    def __init__(self , tab_DH , bool_phi:bool , bool_teta:bool ):
        self.DH_parametre = tab_DH
        self.phi = bool_phi 
        self.teta = bool_teta



class Robot :

    def __init__(self, joint_nombre , phis , r , d ):
        self.joint_nombre = joint_nombre
        self.phis = phis
        self.r = r
        self.d = d
    
  
  


def inverseKinematic(robot , target , epsilonne):
    num = robot.joint_nombre

    estimation = np.full(num , np.pi/5 , dtype=np.float32)
    
    for iteration in range(100):
        
        Z = [None] * num
        Posi = [None] * num
        
        for i in range(robot.joint_nombre):
       
            T = ForwardKinematic(robot, estimation , i) # i pour retrouver la matrice de transformation sbsolu du joint i
       
            Z[i] = T[:3, 2]
        
            Posi[i] = T[:3,3]
                        
        error = target - Posi[-1]
    
        if np.linalg.norm(error) < 1e-6:
            break
        
        # Levenberg-Marquardt 
        # Δ𝑞=(𝐽𝑇𝐽+𝜆2𝐼)−1𝐽𝑇𝑒Δq=(JTJ+λ2I)−1JTe
        
        # Jacobien 
        J = np.zeros((3, num))
        
        for j in range(num):
            J[: , j] = np.cross(Z[j], (Posi[-1] - Posi[j]))
            
        A = J.T@J + epsilonne**2 * np.eye(num)
        B = J.T @ error
        
        dq = np.linalg.solve(A , B)
        
        # New_estimation = estimation + dq
        # new_error = np.linalg.norm(target - ForwardKinematic(robot, New_estimation)[:3 , 3])
        # old_error = np.linalg.norm(error)
        
        # if new_error < old_error:
        #     estimation = New_estimation
        #     epsilonne *= 0.7   # on fait confiance
        
        # else:
        #     epsilonne *= 2.0   # on stabilise
        estimation += dq
        
        for k in range(num):
            print(Z[k])
        # print( iteration , np.linalg.norm(error))
        # print(f"rang = {np.linalg.matrix_rank(J)} et dq = {dq}")
    
    return estimation

phis = [np.pi/2 , 0 , 0 , ]
# phis = [np.pi/2, np.pi/2, -np.pi/2, 0]
r = [0 , 10 , 10 , 20]
d = [10 , 0 , 0 ,0 ]

robot = Robot(3 , phis , r , d )
estimation = inverseKinematic(robot , np.array([5.0 , 10.0 , 10 ]), 0.01)
position = ForwardKinematic(robot , estimation)

print(f"Estimation = {estimation}")
print(f"vérification = {position[:3 , 3]}")
