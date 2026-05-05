import numpy as np
import matplotlib.pyplot as plt 
from scipy.interpolate import CubicSpline
from matplotlib.animation import FuncAnimation
from gcodeparser import GcodeParser



with open(r"C:\Users\BRENDON\Documents\Arduino\robot.code\bras_robot\bras_robot_trajectoire_ciculaire\MICKEY.nc" , 'r') as f :
    gcode = GcodeParser(f.read())
    
mickey_trajx = []
mickey_trajy = []
mickey_trajz = []

x_prev = 50
z_prev = 0
y_prev = 70

for line in gcode.lines :
    if line.command_str in ['GO' , 'G1'] :
        if line.params.get('X') != None :
            x = line.params.get('X')
            x_prev = x
            
        else :
            x = x_prev
             
        if line.params.get('Y') != None :
            y = line.params.get('Y')
            y_prev = y
            
        else :
            y = y_prev
            
        if line.params.get('Z') != None :
            z = line.params.get('Z')
            z_prev = z
            
        else :
            z = z_prev
            
        
        mickey_trajx.append(x)
        mickey_trajy.append(y)
        mickey_trajz.append(z)
        
        
print(len(mickey_trajx))
print(mickey_trajx)
print(mickey_trajz)
print(mickey_trajy)

#============================================================================================

def roulis_mat(teta) :
    c , s = np.cos(teta) , np.sin(teta)
    
    return np.array([[  1 , 0  , 0 ],
                     [  0  , c  , -s ],
                     [ 0  ,  s  , c ]])

def tangage_mat(teta):
    c , s = np.cos(teta) , np.sin(teta)
    
    return np.array([[ c ,  0 , s ],
                     [ 0 ,  1 , 0 ],
                     [ -s,  0 , c ]
                     ])
    
def larcet_mat(teta):
    c , s = np.cos(teta) , np.sin(teta)
    
    return np.array([[ c  , -s , 0 ],
                     [ s  ,  c , 0 ],
                     [ 0  ,  0 , 1 ]
                     ])


#============================================================================================

def cinematique_inverse(x , y , z , teta_des , L ):
    
    phi = np.atan2(y , x)
    
    r = np.sqrt( x**2 + y**2 )
    
    r = r - L[3]*np.cos(teta_des) 
    z = z - L[3]*np.sin(teta_des) - L[0]

    #r = np.clip(r , 3 ,L[1] + L[2] + L[3] )
    
    R = np.sqrt(r**2 + z**2)
        
    cos1 = (L[1]**2 + R**2 - L[2]**2) / (2*L[1]*R)
    cos2 = (L[1]**2 + L[2]**2 - R**2 )/(2*L[1]*L[2])
    
    cos1 = np.clip(cos1 , -1 , 1 )
    cos2 = np.clip(cos2 , -1 , 1 )
    
    teta1 = np.acos(cos1) + np.atan2(z , r)
    teta2 = np.acos(cos2) - np.pi
    
    teta_des = teta_des - teta1 - teta2
        
    return phi , teta1 , teta2 , teta_des



#============================================================================================

def cinematique_inverse2(x , y , z , teta_des, L ):

    phi = np.atan2(y , x)
    
    r = np.sqrt(x**2 + y**2)
    
    r =  r - L[3]*np.cos(teta_des)
    
    z = z - L[3]*np.sin(teta_des) - L[0]

    R = np.sqrt(r**2 + z**2 )
    
    cos1 = (L[1]**2 + R**2 - L[2]**2) / (2*L[1]*R)
    cos2 = (L[1]**2 + L[2]**2 - R**2 )/(2*L[1]*L[2])
    
    cos1 = np.clip(cos1 , -1 , 1 )
    cos2 = np.clip(cos2 , -1 , 1 )
    
    teta1 = np.acos(cos1) + np.atan2(z , r)
    teta2 = np.acos(cos2) - np.pi
    
    teta_des = teta_des - teta1 - teta2

    return phi , teta1 , teta2 , teta_des 


#=============================================================================================


#============================================================================================

def cinematique_inverse3(x , y , z , teta_des, roty , L ):
  
    R = np.sqrt(x**2 + y**2 )
    
    l = L[3]*np.cos(teta_des)
    l2 = l*np.sin(roty)
    
    r = np.sqrt(R**2 - l2 **2) - l*np.cos(roty)
    
    z = z - L[3]*np.sin(teta_des) - L[0]

    R = np.sqrt(r**2 + z**2 )
    
    phi = np.atan2(y , x) - np.atan2(l2 , np.sqrt(R**2 - l2 **2))
    
    cos1 = (L[1]**2 + R**2 - L[2]**2) / (2*L[1]*R)
    cos2 = (L[1]**2 + L[2]**2 - R**2 )/(2*L[1]*L[2])
    
    cos1 = np.clip(cos1 , -1 , 1 )
    cos2 = np.clip(cos2 , -1 , 1 )
    
    teta1 = np.acos(cos1) + np.atan2(z , r)
    teta2 = np.acos(cos2) - np.pi
    
    teta_des = teta_des - teta1 - teta2

    return phi , teta1 , teta2 , teta_des , roty


#=============================================================================================

def transforme(r , d , teta , phi ):
    
    c_teta , s_teta = np.cos(teta) , np.sin(teta)
    c_phi , s_phi = np.cos(phi) , np.sin(phi)
    
    return np.array([[c_teta , -s_teta*c_phi , s_teta*s_phi , r*c_teta ],
                     [s_teta , c_teta *c_phi , -c_teta*s_phi , r*s_teta ],
                     [   0   ,    s_phi      ,     c_phi     ,    d    ],
                     [   0    ,   0      ,        0      ,    1        ] 
                      ])



fig = plt.figure()
ax = fig.add_subplot(111 , projection='3d')

ax.set_xlim(-100 , 100)
ax.set_ylim(-100 , 100)
ax.set_zlim(0 , 100)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')



#ax.scatter(mickey_trajx , mickey_trajy , mickey_trajz ,color='gray' ,  s=1 , alpha=0.2 ,zorder=1 )
#zorder permet de passer la trajectoire au dessus

nb_segment = 4
L = [10 , 30 , 30 , 5 ]
lines = []
points, = ax.plot([] , [] , [] , 'r-' , linewidth = 1)

pointx = []
pointy = []
pointz = []

for i in range(nb_segment):
    segment, = ax.plot([] , [] , [] , 'b-', linewidth = 3 )
    lines.append(segment)
    
def init():
    for line in lines :
        line.set_data([] , [])
        line.set_3d_properties([])
    
    points.set_data([] , [])
    points.set_3d_properties([])
    
    return lines + [points]

def update(frame):
    
    
    theta_i = cinematique_inverse2(mickey_trajx[frame] , mickey_trajy[frame] , mickey_trajz[frame] , -np.pi/2 , L )
    
    T1 = transforme(0 , L[0] , theta_i[0] , np.pi/2 )
    T2 = transforme(L[1] , 0 ,theta_i[1] , 0)    
    T3 = transforme(L[2] , 0 , theta_i[2] , 0)    
    T4 = transforme(L[3] , 0 ,theta_i[3] , 0)
    
    
     
    T_global1 = T1
    T_global2 = T1 @ T2
    T_global3 = T1 @ T2 @ T3
    T_global4 = T1 @ T2 @ T3 @ T4
    
    
    P0 = [0 , 0 , 0 ]
    P1 = T_global1[:3 , 3]
    P2 = T_global2[:3 , 3]
    P3 = T_global3[:3 , 3]
    P4 = T_global4[:3 , 3]
    
    
    Pos = [P0 , P1 , P2 , P3 , P4 ]
    
    for i in range(nb_segment) :
        
        X = [Pos[i][0] , Pos[i+1][0]]        
        Y = [Pos[i][1] , Pos[i+1][1]]        
        Z = [Pos[i][2] , Pos[i+1][2]]        

        lines[i].set_data(X , Y)
        lines[i].set_3d_properties(Z)
        
    #if frame == 0 :
    #    pointx.clear()
    #    pointy.clear()
    #    pointz.clear()
    
    
    if mickey_trajz[frame] < 0 :
        pointx.append(mickey_trajx[frame])        
        pointy.append(mickey_trajy[frame])        
        pointz.append(mickey_trajz[frame])        

    points.set_data(pointx , pointy)
    points.set_3d_properties(pointz) 
    
    #print("Cible :", mickey_trajx[frame], mickey_trajy[frame], mickey_trajz[frame])
    #print("Robot :", P4[0], P4[1], P4[2])
    
    if frame == len(mickey_trajx):
        print("FIN")
        
    return lines + [points]

frames_mickey = len(mickey_trajx)

ani = FuncAnimation(fig , update , frames=frames_mickey , init_func=init , interval = 50)
ani.save("robot.gif" , writer='pillow' , fps= 30 )

plt.show()

#fig.canvas.p

