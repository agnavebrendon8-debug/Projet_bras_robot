from gcodeparser import GcodeParser
import numpy as np

import Conversion_to_gcode as Cn

Cn.converter_to_gcode(r"C:\Users\BRENDON\projet_robot\Png_fichier\tete_mickey.svg", r"C:\Users\BRENDON\projet_robot\tete_mickey.nc" , 0 , 0 , 0 , True )

def to_target(File_gcode, length_max):
    
    with open(File_gcode, 'r') as f :
        gcode = GcodeParser(f.read())
    
    mickey_trajx = [None] * length_max
    mickey_trajy = [None] * length_max
    mickey_trajz = [None] * length_max

    x_prev = 10
    z_prev = 0
    y_prev = 5
    i= 0
    for line in gcode.lines :
        if i == length_max :
            break
        if line.command_str in ['G0' , 'G1'] :
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
            
        
        mickey_trajx[i] = x_prev/10
        mickey_trajy[i] = y_prev/10
        mickey_trajz[i] = z_prev
        i+=1
    return mickey_trajx , mickey_trajy , mickey_trajz