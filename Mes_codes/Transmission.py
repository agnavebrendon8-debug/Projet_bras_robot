import serial 
import time

port = "COM3"
Baudrate = 115200 
timeout = 1
Write_timeout = 0

if __name__ == "__main__" :
    ser = serial.Serial(port=port , baudrate=Baudrate , timeout=timeout , write_timeout=Write_timeout)
    time.sleep(2)
    print(ser.get_settings())
    

def init_serial(port=port , baudrate=Baudrate , timeout=timeout , currente_pos=None, Write_timeout=Write_timeout , show_setting=True):
    ser = serial.Serial(port=port, baudrate=Baudrate ,timeout=timeout , write_timeout=Write_timeout)
    time.sleep(2)
    
    if show_setting :
        setting = ser.get_settings()
        print("Parametres de connnexion :", setting)
    
    if currente_pos is not None :    
        message = ",".join(str(angle) for angle in currente_pos) + "\n"
                
    ser.reset_output_buffer()
    ser.write(message.encode("utf-8"))
    time.sleep(1)
    print("Position initialisé avec succes ")
    
    return ser

def send(ser ,angles):
    message = ",".join(str(angle) for angle in angles) + "\n"
    ser.write(message.encode("utf-8"))
    return 

#strip(str) : permet de separer une chaine en une liste (parametre = caractere de separation )

def receive(ser):
    reponse_octet = ser.readline()

    if reponse_octet == b"":
        return None 
    
    try :
        message = reponse_octet.decode("utf-8").strip()
    
        if message.startswith("A:"):
            message = message[:2]
            morceau = message.split(",")
            angles = [float(angle) for angle in morceau ]
            return angles
    
        return message
    
    except ValueError :
        print(f"Erreur de conversion des angles du message : {message}")
        
    return None 

