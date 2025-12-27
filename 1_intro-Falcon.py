#!/usr/bin/env python3

from pyDhd import *

done = False
dhdOpen()

while(not done):
    ret, px,py,pz = dhdGetPosition()
    ret, vx, vy, vz = dhdGetLinearVelocity()
    print (ret,px,py,pz)
    done=dhdGetButton(0)
    dhdSetForce(10,0,0)
#    print(done)
    
dhdClose()
