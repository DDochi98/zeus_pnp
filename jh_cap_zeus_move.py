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
    
    #x = 100.0
    #y = 400.0
    #z = 300.0
    #rz = 0.0

    m = MotionParam( jnt_speed=10, lin_speed=50 )
    rb.motionparam( m )

    try:
        while True:
            x = input('move_x : ')
            y = input('move_y : ')
            z = input('move_z : ')
            rz = input('angle : ')

            rb.sleep(0.5)
            DHA16_Open()
            dout(48,'100')
            rb.sleep(0.5)

            p1 = Position( x, y, 300, rz, 0.0, -180 )
            p2 = Position( x, y, z, rz, 0.0, -180 )
            p3 = Position( 100.0, 410.0, 400.0, 90.0, 0.0, -180 )
            p4 = Position( 100.0, 410.0, z, 90.0, 0.0, -180 )
           
            #move to goal
            rb.move( p1 )
            rb.sleep(1.0)
            
            #down to object
            rb.move( p2 )

            #pick
            DHA16_Close()
            dout(48,'001')
            rb.sleep(0.5)
     
            #up from object
            rb.move( p1 )

            #move to place
            rb.move( p3 )

            #down to place
            rb.move( p4 )

            #place object
            DHA16_Open()
            dout(48,'100')

    except KeyboardInterrupt:
        print "KeyboradInterrupt"
        rb.home()
    finally:
        print "finally"
        rb.close()

    rb.close()
if __name__ == '__main__':
    main()