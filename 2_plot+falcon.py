#!/usr/bin/env python3

from pyDhd import *
#bibliothèque numeric pour math
from utils import *

#bibliothèque plot etc.
#from pylab import *
done = False

dhdOpen()

k = 600

posx = list()
posy = list()
posz = list()


while(not done):
    
    ret, px,py,pz = dhdGetPosition()
    posx = posx + [px]
    posy = posy + [py]
    posz = posz + [pz]
    
    print(px,py,pz)
    dhdSetForce(0,0,0)    
   
    
    done=dhdGetButton(0)
    
dhdClose()


figure("my plot")
plot(posx,'red',posy,'blue',posz,'green')
show()  

print(max(posx)," et" ,min(posx))