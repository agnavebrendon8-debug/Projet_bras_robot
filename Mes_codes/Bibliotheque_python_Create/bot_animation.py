import numpy as np
import matplotlib.pyplot as plt
import Leven_Marq as LM 
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider


nb_joint = 6
phis = [np.pi/2 , 0 , 0 , np.pi/2 , -np.pi/2 , 0 ]
r = [0 ,10 , 10 , 0 , 0 , 2 ]
d = [5 , 0 , 0 , -2, 5 , 0 ]

limits = np.array([[-np.pi , np.pi], [-np.pi/2 ,np.pi/2 ] ,
                   [-np.pi/2 , np.pi/2], [-np.pi/2 , np.pi/2] ,
                   [-np.pi , np.pi] , [-np.pi/2 , np.pi/2]])

robot6DoF = LM.Robot(nb_joint , phis , r , d , limits)

fig = plt.figure()
ax = fig.add_subplot(111 , projection="3d")

plt.subplots_adjust(bottom=0.25)

ax.set_xlim(-50 , 50)
ax.set_ylim(-50 , 50)
ax.set_zlim(-50 , 50)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

Line = [None] *( nb_joint + 3 ) 
for i in range(nb_joint):
    segment, = ax.plot([] , [] ,[] , lw=3)
    Line[i] = segment

segment1, =ax.plot([] , [] ,[] , lw=3 , c="r")
segment2, =ax.plot([] , [] ,[] , lw=3 , c="r")
segment3, =ax.plot([] , [] ,[] , lw=3 , c="r")

Line[nb_joint] = segment1
Line[nb_joint + 1] = segment2
Line[nb_joint + 2] = segment3

rot_cible = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
p_cible = np.array([10 , 10 , 5])
mat_cible = np.eye(4)
mat_cible[:3,:3] = rot_cible
mat_cible[:3, 3] = p_cible

angle_trouve = LM.inverseKinematic6D(robot6DoF , mat_cible)    
# parametre de position du slider (en % : gauche , bas , largeur , Hauteur)

ax_slider1 = plt.axes([0.25, 0.22 , 0.65 ,0.03 ])
ax_slider2 = plt.axes([0.25, 0.185 , 0.65 ,0.03 ])
ax_slider3 = plt.axes([0.25, 0.15 , 0.65 ,0.03 ])
ax_slider4 = plt.axes([0.25, 0.115 , 0.65 ,0.03 ])
ax_slider5 = plt.axes([0.25, 0.08 , 0.65 ,0.03 ])
ax_slider6 = plt.axes([0.25, 0.045 , 0.65 ,0.03 ])

#  valinit=np.mean(limits)
Slider1 = Slider(ax=ax_slider1 , label="joint1" , valmin=limits[0][0] ,valmax=limits[0][1] , valinit=angle_trouve[0] )
Slider2 = Slider(ax=ax_slider2 , label="joint2" , valmin=limits[1][0] ,valmax=limits[1][1] , valinit=angle_trouve[1] )
Slider3 = Slider(ax=ax_slider3 , label="joint3" , valmin=limits[2][0] ,valmax=limits[2][1] , valinit=angle_trouve[2] )
Slider4 = Slider(ax=ax_slider4 , label="joint4" , valmin=limits[3][0] ,valmax=limits[3][1] , valinit=angle_trouve[3] )
Slider5 = Slider(ax=ax_slider5 , label="joint5" , valmin=limits[4][0] ,valmax=limits[4][1] , valinit=angle_trouve[4] )
Slider6 = Slider(ax=ax_slider6 , label="joint6" , valmin=limits[5][0] ,valmax=limits[5][1] , valinit=angle_trouve[5] )

Sliders = [Slider1 , Slider2, Slider3 , Slider4 , Slider5 , Slider6]








# test pour la cinématique inverse 


def init():
    for line in Line :
        line.set_data([] , [])
        line.set_3d_properties([])
        
    return Line

quiver_x = None
quiver_y = None
quiver_z = None

def update(frame):
    angles = [None] * nb_joint
    for k in range(nb_joint) :
        angles[k] = Sliders[k].val


    x_prev, y_prev , z_prev = 0, 0 , 0
    robot6DoF.current_angle = angles 
    
    for i in range(nb_joint):
        Target_i = LM.ForwardKinematic(robot6DoF , robot6DoF.current_angle , i)
        Pos = Target_i[:3 , 3]
        
        X = [x_prev , Pos[0]]
        Y = [y_prev , Pos[1]]
        Z = [z_prev , Pos[2]]
        
        Line[i].set_data(X , Y)
        Line[i].set_3d_properties(Z)
        
        x_prev , y_prev , z_prev = X[1] , Y[1] , Z[1]


    Line[nb_joint].set_data([x_prev , x_prev + Target_i[0][0]], [y_prev, y_prev + Target_i[1][0]])
    Line[nb_joint].set_3d_properties([z_prev, z_prev + Target_i[2][0]])

    Line[nb_joint + 1].set_data([x_prev , x_prev + Target_i[0][1]], [y_prev, y_prev + Target_i[1][1]])
    Line[nb_joint + 1].set_3d_properties([z_prev, z_prev + Target_i[2][1]])
    
    Line[nb_joint + 2].set_data([x_prev , x_prev + Target_i[0][2]], [y_prev, y_prev + Target_i[1][2]])
    Line[nb_joint + 2].set_3d_properties([z_prev, z_prev + Target_i[2][2]])

    return Line  
    
ani = FuncAnimation(fig , update , frames=200 , init_func=init ,blit=True , interval=50 )
plt.show()

