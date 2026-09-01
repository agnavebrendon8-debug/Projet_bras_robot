import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation 
from ikpy.chain import Chain 
from ikpy.link import OriginLink , URDFLink




            
fig = plt.figure()
ax = fig.add_subplot(111 , projection='3d')

ax.set_xlim(-100 , 100)
ax.set_ylim(-100 , 100)
ax.set_zlim(-100 , 100)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')


class motor():
    
    def __init__(self, name ,num_segment, default_angle , default_velocity):
        self.name = name
        self.num_segment = num_segment
        self.default_angle = default_angle
        self.default_speed = default_velocity
        self.angle = default_angle
        self.speed = default_velocity
        return
    
    def update_motor(self, new_amgle ):
        self.angle = new_amgle
    
    def reinitialiser(self):
        self.angle = self.default_angle
        self.speed = self.default_speed
        return 
    
    
class sensor():
    def __init__(self , tb_moteurs):
        self.tab_moteur = tb_moteurs
    
    def lire_info(self,num):
        moteur = self.tab_moteur[num]
        print("/==========================================================================/")
        print(f"name_motor : {moteur.name}")    
        print(f"angle : {moteur.angle}")    
        print(f"speed : {moteur.speed}")    

 
class segment():
    
    def __init__(self, num ,origine ,length):
        self.num = num 
        self.origine = origine
        self.length = length
        self.angle = 0                                  # angle du moteur
        self.target = [origine[0], self.length]
        
    def update_segment(self,origine ,  phi , teta ):    # teta autour de l'axe de rotation 
        self.origine = origine
        self.target = [self.origine[0] + self.length*np.cos(phi) , self.origine[1] + self.length*np.sin(teta) ]
        
    
 
class chaine():
    
    def __init__(self,origine ,nb_segment , tab , seg_tab ):
        
        self.origine = origine
        self.seg_nombre = nb_segment
        self.seg_length = tab
        self.segments = seg_tab 
        self.Lines = []
        self.target = []
        for i in range(self.seg_nombre):
            segment, = ax.plot([] , [] , [] , '-b' , linewidth=3)
            self.Lines.append(segment)
            
        self.robot_arm = Chain(name="robot_bras" , links=(
        
            #rotation : axe de rotation 
            URDFLink(name="joint1",origin_translation=[0 ,0,tab[0]], origin_orientation=[0 , 0 , 0] , rotation=[0 , 0 , 1]),
            URDFLink(name="joint2",origin_translation=[tab[1],0,0], origin_orientation=[np.pi/2 , 0 , 0] , rotation=[0 , 0 , 1]),
            URDFLink(name="joint3",origin_translation=[tab[2],0,0], origin_orientation=[0 , 0 , 0] , rotation=[0 , 0 , 1]),
            URDFLink(name="joint4",origin_translation=[tab[3],0,0], origin_orientation=[0 , 0 , 0] , rotation=[0 , 0 , 1])
        
        ))

        print(self.robot_arm)
        return 
    
    def get_segment_info(self):
        segments = self.segments
        
        for segment in segments :
            print(f"segment_num = {segment.num}")        
            print(f"angle : {segment.angle}")        
            print(f"target : {segment.target}")
            
        return        
    
    
    def inverseCinematic3(self , Pos , nb_segment ,teta = np.pi/2 ):
        X = Pos[0]
        Y = Pos[1]
        Z = Pos[2] - self.segments[3].length * np.sin(teta) - self.segments[0].length
        
        seg2 = self.segments[1].length
        seg3 = self.segments[2].length
        seg4 = self.segments[3].length
        
        r = np.sqrt(X**2 + Y**2 ) - self.segments[3].length * np.cos(teta) 
        R = np.sqrt(Z**2 + r**2 )

        teta1 = np.atan2(Y , X)
        teta2 = np.atan2( Z , r ) + np.acos(np.clip((seg2**2 + R**2 - seg3**2)/(2*seg2*R), -1 , 1) )
        teta3 = np.acos(np.clip((seg2**2 + seg3**2 - R**2)/(2*seg2*seg3), -1 , 1) ) - np.pi
        teta4 = teta - teta3 - teta2
        
        return [teta1 , teta2 , teta3 , teta4 ]
    
    
    def inverse_kinematic(self, tab):
        return self.robot_arm.inverse_kinematics(tab)
    
    
    def init_chaine(self):
        for line in self.Lines :
            line.set_data([] , [])
            line.set_3d_properties([])
        
        return tuple(self.Lines)
    
    def Update(self, frame , tab_angle , motors ): #tab_angle deux dimensions 
        
        segments = self.segments
        
        for segment in segments :
            segment.angle = tab_angle[frame][segment.num]
 
        for motor in motors :
            motor.update_motor(tab_angle[frame][motor.num_segment])
        
        x_prev , y_prev , z_prev = self.origine[0] , self.origine[1] ,self.origine[2]
        
        for i in range(self.seg_nombre) :
            phi = tab_angle[frame][0]
            teta = tab_angle[frame]
            
            if i == 0 :
                teta_total = 0
                X = [x_prev , x_prev]
                Y = [y_prev , y_prev]
                Z = [z_prev , z_prev + self.seg_length[i]]
                
                segments[i].origine = [X[0] , Y[0] , Z[0] ]
                segments[i].target = [X[1] , Y[1] , Z[1] ]
        
            else :
                teta_total += teta[i]
                X = [x_prev , x_prev + self.seg_length[i]*np.cos(teta_total)*np.cos(phi)]
                Y = [y_prev , y_prev + self.seg_length[i]*np.cos(teta_total)*np.sin(phi)]
                Z = [z_prev , z_prev + self.seg_length[i]*np.sin(teta_total)]
                
                segments[i].origine = [X[0] , Y[0] , Z[0] ]
                segments[i].target = [X[1] , Y[1] , Z[1] ]
                
                
            self.Lines[i].set_data(X , Y)
            self.Lines[i].set_3d_properties(Z)
            
            
            x_prev = X[1]
            y_prev = Y[1]
            z_prev = Z[1]
            
            
        self.target = [X[1] , Y[1] , Z[1] ]
        
        # if frame % 50 == 0 :
        #     print(f"Target = {self.target}")
        #     print()
        
        return tuple(self.Lines)
                
     
        


nb_segment = 4
tab_length = [20 , 40 ,40 , 20 ]
Frames = 200 

# tab_phi = np.linspace(np.pi/20,2*np.pi/3 , Frames )
# tab_teta1 = np.linspace(np.pi/10 ,2*np.pi/3 , Frames)
# tab_teta2 = np.linspace(np.pi/20 ,2*np.pi/4 , Frames)
# tab_teta3 = np.linspace(np.pi/15 ,2*np.pi/6 , Frames)

# tab_angle = []

# for i in range(Frames):
#     tab_angle.append([ tab_phi[i] , tab_teta1[i], tab_teta2[i] , tab_teta3[i] ])    



motors = []
motors2 = []

for i in range(nb_segment):
    m = motor(f"seg{i}" , i , 50 , np.pi/30 )
    motors.append(m)
    
for i in range(nb_segment):
    mm = motor(f"Seg_{i}", i , 30 , np.pi/30 )
    motors2.append(mm)

Sensor = sensor(motors)

# for i in range(nb_segment):
#     Sensor.lire_info(i)


seg1 = segment(0 , [0 , 0 , 0 ] , 20 )        
seg2 = segment(1 , [0 , 0 , 0 ] , 40 )        
seg3 = segment(2 , [0 , 0 , 0 ] , 40 )        
seg4 = segment(3 , [0 , 0 , 0 ] , 20 )

segments = []

segments.append(seg1)        
segments.append(seg2)        
segments.append(seg3)        
segments.append(seg4)        

Chaine1 = chaine([0 , 0 , 0 ] , nb_segment ,tab_length , segments )
Chaine2 = chaine([10 , 10 , 10 ] , nb_segment ,tab_length , segments )








tab_angle = []
target_x = list(np.linspace(30 , 40 , Frames))
target_y = list(np.linspace(20, 35 , Frames))
target_z = list(np.linspace(10, 30 , Frames))

points, = ax.plot([] , [] , [] , 'r-' , linewidth = 1)
pointx = []
pointy = []
pointz = []

# ax.plot([] , [] , [] , 'r-' , linewidth = 1)

def init():
    points.set_data([] , [])
    points.set_3d_properties([])
    
    return Chaine1.init_chaine() + (points,)

def update(frame):
    
    tab_angle.append(Chaine1.inverseCinematic3([target_x[frame] , target_y[frame] , target_z[frame] ] , 4 , 0))
    t1 = Chaine1.Update( frame , tab_angle , motors)
    
    
    if frame % 50 == 0 :
        print("=========================================")
        print(f" tab_angle = {tab_angle[frame]}")
        print("=========================================")
        print(f"inverse_calculate = {Chaine1.inverse_kinematic([target_x[frame] , target_y[frame] , target_z[frame] ])}")
        print(f"Forword_kinematics = {Chaine1.robot_arm.forward_kinematics(Chaine1.inverse_kinematic([target_x[frame] , target_y[frame] , target_z[frame] ]))}")
    
    pointx.append(target_x[frame])
    pointy.append(target_y[frame])
    pointz.append(target_z[frame])
    
    points.set_data(pointx, pointy)
    points.set_3d_properties(pointz)
    
    
    return tuple(t1) + (points,)




ani = FuncAnimation(fig , update , frames=Frames , init_func=init , interval=50 )

ani.save("class_botTest.gif" , writer="pillow" , fps=30 )
plt.show()