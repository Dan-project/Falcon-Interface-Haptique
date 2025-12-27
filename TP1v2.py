# -*- coding: utf-8 -*-
"""
Created on Wed Nov 26 15:40:26 2025

@author: 21111315
"""


 # creation d'u ressort amortisseur
 
import time
from pyDhd import *
posx = list()
posy = list()
posz = list()

done = False
dhdOpen()
print(dhdOpen())
k=500
c=10
l=0.005
ti=time.time_ns()
t=list()
f_mur_set=list()

mur= False # on peut simuler un mur avec un ressort, ressort amortisseur, le but est de faire le mur le plus rigide possible
ressort= True
amortisseur= False
mur_incline=False

# implementation d'un mur en y <-0.02 m 

f_mur_get=list()


while(not done):
    ret, px,py,pz = dhdGetPosition()
    ret, vx, vy, vz = dhdGetLinearVelocity()
    posx = posx + [px]
    posy = posy + [py]
    posz = posz + [pz]
    
    if ressort:
        
        fx = k*(l-px)
        fy = k*(l-py)
        fz = k*(l-pz)

        dhdSetForce(fx,fy,fz)
        
    elif amortisseur:
        
        fx = -c*vx
        fy = -c*vy
        fz =- c*vz
        

        dhdSetForce(fx,fy,fz)

   
    
    elif mur:
        
        if py > -0.02:
            dhdSetForce(0,0,0)
            f_mur_set.append(0)
            f_mur_get.append(0)
        else:
            delta=(py+0.02)
            fy_temp=-1*1000*delta -c*vy
            print(fy_temp)
            dhdSetForce(0,fy_temp,0)
            _,_,fy_temp_get,_=dhdGetForce()
            f_mur_get.append(fy_temp_get)
            #print(f'force du mur {-1*1000*delta}')
            f_mur_set.append(-1*1000*delta)
            

        
    done=dhdGetButton(0)
    t.append(abs(ti-time.time_ns()))
        
tf= abs(ti-time.time_ns())
#print(tf)    
dhdClose()

import matplotlib.pyplot as plt
if ressort:
    plt.figure("my plot")
    plt.plot(t, posx, color='red', label='x')
    plt.plot(t, posy, color='blue', label='y')
    plt.plot(t, posz, color='green', label='z')
    
    import numpy as np
    periode=(1.5-0.1)/10
    wz= 2*np.pi*1/periode
    m=k/(wz*wz)
    c_intern=2*k*m/wz
    print(f"la masse de l'effecteur mobile est {m} kg")
    print(f"l'amortissement du falcon est {c_intern} kg/s")

if mur:
    plt.figure("my plot2")
    plt.figure("force renvoyé par le mur selon y")
    plt.plot(posy, f_mur_set, color='red', label='set')
    plt.plot(posy, f_mur_get, color='blue', label='get')

plt.show()  
