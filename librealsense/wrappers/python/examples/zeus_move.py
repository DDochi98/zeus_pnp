#!/usr/bin/python
# -*- coding: utf-8 -*-

from i611_MCS import *
from i611_extend import *
from i611_io import *

def DHA16_Close():
    dout(48,'001')


def DHA16_Open():
    dout(48,'100')


def main():
    rb = i611Robot()
    _BASE = Base()
    rb.open()
    IOinit( rb )

    #p2 = Position(99.81, 410.04, 406.83, 90.0, -2.66, -179.87 )
    p2 = Position(519.63, 219.720, 175.0, 0.0, 0.0, -180.0)
    m = MotionParam( jnt_speed=10, lin_speed=70 )
    rb.motionparam( m )

    while(1) :                                                  
        donjak = input('1 move_y 2 move_z 3 home 4 p2_home 5 break 6 open 7 close 8 getpos : ')
        if donjak == 1 :                         
                                              
            print('x distance base is 0')                  
            move_x = input('move x distance:')                  
                                                                                               
            print('y distance base is 785')      
            move_y = input('move y distance:')
                                                           
            p3 = p2.offset(dx=move_x ,dy=move_y)                
            rb.move(p3)                                                                        
                                                 
        elif donjak == 2 :                    
            print('camera 240 / greeper 160 / picking 185')
            move_z = input('move z distance:')                  
            p4 = p3.offset(dz=move_z)                                                          
            rb.move(p4)                          
                                                                
        elif donjak == 3 :                                 
            rb.home()                            
                                                                                               
        elif donjak == 4 :                 
            rb.move(p2)                       
                                                                                               
        elif donjak == 5 :                      
            break                     
                                            
        elif donjak == 6 :                                     
            DHA16_Open()                                   
            rb.sleep(0.5)                       
            dout(48,'000')                    
                                            
        elif donjak == 7 :                                     
            DHA16_Close()                                  
            rb.sleep(0.5)                     
            dout(48,'000')         
        else :                                  
            print('you only take step 1~8')
            pass                            
                                                               
    rb.close()                                             
                                                
if __name__ == '__main__':                    
    print('start')                          
    main()       