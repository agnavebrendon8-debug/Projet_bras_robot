import serial 
import time

from PyQt5.QtCore import QObject , pyqtSlot  


class RobotHardwareBridge(QObject):
    """Classe chargée de traduire les signaux IHM en commandes physiques (USB/Série)"""
    def __init__(self, gui , port="COM3", baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.connect_hardware()
        self.connect_gui_signal(gui)    
    

    def connect_hardware(self):
        """Initialise la connexion avec la carte microcontrôleur"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2) # Temps de sécurité pour le reboot de la carte (ex: Arduino)
            print(f"✅ Robot réel connecté sur le port {self.port}")
        except Exception as e:
            print(f"⚠️ Impossible de se connecter au robot réel : {e}")
            
    def connect_gui_signal(self , gui):
        
        gui.signals.angles_changed.connect(self.send_angles_to_motors)
        gui.signals.tool_command.connect(self.send_tool_command)
        gui.signals.emergency_stop.connect(self.process_emergency_stop)
        gui.signals.Redemarre.connect(self.process_begin)

    @pyqtSlot(list)
    def send_angles_to_motors(self, angles_deg):
        """Reçoit la liste des angles [A1, A2, A3, A4, A5] de l'IHM et l'envoie au robot"""
        if self.ser and self.ser.is_open:
            try:
                # Formatage de la trame : "A1.11,A2.22,A3.33,A4.44,A5.55\n"
                trame = ",".join([f"{angle:.2f}" for angle in angles_deg]) + "\n"
                
                # Envoi physique sur le port série (encodé en ASCII/Bytes)
                self.ser.write(trame.encode('ascii'))
            except Exception as e:
                print(f"Erreur d'envoi de la trame articulaire: {e}")

    @pyqtSlot(str, dict)
    def send_tool_command(self, tool_type, params):
        """Transmet l'état de l'effecteur (pince ou ventouse) au matériel"""
        if self.ser and self.ser.is_open:
            try:
                if tool_type == "Pince Électrique":
                    trame = f"TOOL:PINCE:{params['ouverture']}\n"
                else:
                    etat = 1 if params['aspiration'] else 0
                    trame = f"TOOL:VENTOUSE:{etat}\n"
                
                self.ser.write(trame.encode('ascii'))
            except Exception as e:
                print(f"Erreur d'envoi de la trame effecteur: {e}")

    @pyqtSlot()
    def process_emergency_stop(self):
        """Envoie l'ordre prioritaire de coupure immédiate de la puissance"""
        if self.ser and self.ser.is_open:
            try:
                # Trame prioritaire lue immédiatement par l'interruption du microcontrôleur
                self.ser.write(b"EMERGENCY_STOP\n")
                self.ser.flush() # Force l'envoi immédiat du buffer
                time.sleep(2)
                print("❌ Signal d'arrêt d'urgence transmis au matériel !")
            except Exception as e:
                print(f"Erreur d'envoi de l'arrêt d'urgence: {e}")
                
    @pyqtSlot()
    def process_begin(self) :
        if self.ser and self.ser.is_open :
            try :
                self.ser.write(b"Emergency_begin\n")
                self.flush()
                print("Signal de Redemarrage du robot ")
            except Exception as e :
                print(f"Erreur d'envoi de la commande de redemarrage : {e}")


    @pyqtSlot(dict)
    def send_cartesian_pos(position):
        pass 
    
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()



