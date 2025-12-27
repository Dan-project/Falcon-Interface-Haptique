# -*- coding: utf-8 -*-
"""
Created on Wed Nov 26 15:40:26 2025

@author: 21111315 Dan CALAMIA
"""


# creation d'u ressort amortisseur

import time
from pyDhd import *
from utils import Vecteur3d


class Simulation():

    def __init__(self,mode):
        
        # init list
        self.posx = list()
        self.posy = list()
        self.posz = list()

        self.t=list()
        self.f_mur_set=list()
        self.f_mur_get=list()
        self.f_get= []

        # init system
        self.done = False
        dhdOpen()

        # init value
        self.k=500
        self.c=10
        self.l=0.005
        
        #mode
        self.mode=mode
        
        self.fx=0
        self.fy=0
        self.fz=0
  
        
        

    def update(self):

        self.ret, self.px, self.py, self.pz = dhdGetPosition()
        self.ret, self.vx, self.vy, self.vz = dhdGetLinearVelocity()
        self.posx = self.posx + [self.px]
        self.posy = self.posy + [self.py]
        self.posz = self.posz + [self.pz]


    def set_force(self,fx,fy,fz):

        dhdSetForce(fx,fy,fz)


    def ressort(self):
        """ 
        Ressort xyz
        """
        
        self.fx = self.k*(self.l-self.px)
        self.fy = self.k*(self.l-self.py)
        self.fz = self.k*(self.l-self.pz)
      

    def amortisseur(self):
        """ 
        Amortisseur xyz
        """

        self.fx = -self.c*self.vx
        self.fy = -self.c*self.vy
        self.fz =- self.c*self.vz


    def mur1(self,plan='xz',lim=-0.02):
        """ 
        Mur simple imité par un ressort
        """
        if plan=='xz':

            if self.py > lim:
                self.fx,self.fy,self.fz=0,0,0
                self.f_mur_set.append(0)


            else:
                delta=(self.py-lim)
                self.fy=-1*1000*delta
                self.f_mur_set.append(self.fy)
                self.fx,self.fz=0,0

                # recevoir la force réelle du robot
            _,_,fy_temp_get,_=dhdGetForce()
            self.f_mur_get.append(fy_temp_get)


    def mur2(self,plan='xz',lim=-0.02):
        """ 
        Mur simple imité par un ressort amortisseur
        """

        if plan=='xz':

            if self.py > lim:
                self.fx,self.fy,self.fz=0,0,0
                self.f_mur_set.append(0)
                self.f_mur_get.append(0)

            else:
                delta=(self.py-lim)
                self.fy=-1000*delta -self.c*self.vy
                self.f_mur_set.append(self.fy)
                self.fx,self.fz=0,0

                # recevoir la force réelle du robot
                _,_,fy_temp_get,_=dhdGetForce()
                self.f_mur_get.append(fy_temp_get)
        
        
        
    def mur_init(self, P0 = Vecteur3d(0, 0, 0), n  = Vecteur3d(0, 1, 1)):

        """ 
        Initialisation de mur ( n: normal au mur, p0: position appartenant au pla crée par le mur)
        """

        pos=Vecteur3d(self.px,self.py,self.pz)
        fvec=Vecteur3d(0,0,0)
        vitesse= Vecteur3d(self.vx,self.vy,self.vz) 
        distance= (pos-P0).dot(n) # ici on veut faire un produit sclaire entre deux vecteurs a voir dans utils
        
        if distance <=0:
            fvec= -1000*distance*n - 1* vitesse # on multiplie un scalaire par un vecteur 
         
        return fvec
    
    def mur_vib(self, P0 = Vecteur3d(0, 0, 0), n  = Vecteur3d(1, 0, 0)):
        """ 
        Initialisation de mur  texturé ( n: normal au mur, p0: position appartenant au pla crée par le mur)
        Vibrations liées au sinus 
        """
        
        import numpy as np
        pos=Vecteur3d(self.px,self.py,self.pz)
        fvec=Vecteur3d(0,0,0)
        vitesse= Vecteur3d(self.vx,self.vy,self.vz) 
        distance= (pos-P0).dot(n) # ici on veut faire un produit sclaire entre deux vecteurs a voir dans utils
        
        nx =Vecteur3d(1,0,0)
        ny= Vecteur3d(0,1,0)
        if distance <=0:
            fvec= -1000*distance*n - 1* vitesse # on multiplie un scalaire par un vecteur 
            fvec.z = fvec.z + 1*np.sin(1000*self.pz)
            fvec.y = fvec.y + 1*np.sin(1000*self.pz)
         
        return fvec
    

    
    
        
    
    def sphere(self,sens=-1, centre=Vecteur3d(0,0,0),r=0.03):
        """ 
        Initialisation d'une sphère ( sens: exterieu ou interieur, r: rayon, normal au mur, centre: position centre sphère )
        """
        import numpy as np
        pos=Vecteur3d(self.px,self.py,self.pz)
        fvec=Vecteur3d(0,0,0)
        vitesse= Vecteur3d(self.vx,self.vy,self.vz)
        n= Vecteur3d(pos.x-centre.x,pos.y-centre.y,pos.z-centre.z)
        distance= n.mod()
        
        if sens*(distance - r) <=0:
            fvec= -10000*(distance-r)*n - 1* vitesse
        
        return fvec
                
    
   
        
    def plot(self):
        """ 
        Fonction plot
        """

        import matplotlib.pyplot as plt

        if self.mode==1:
            plt.figure("Ressort xyz")
            plt.plot(self.t, self.posx, color='red', label='x')
            plt.plot(self.t, self.posy, color='blue', label='y')
            plt.plot(self.t, self.posz, color='green', label='z')
            plt.title("Système Ressort")
            plt.xlabel('Temps ns')
            plt.ylabel('Positions m')
            plt.legend()
            
            import numpy as np
            periode=(1.5-0.1)/10
            wz= 2*np.pi*1/periode
            m=self.k/(wz*wz)
            c_intern=2*self.k*m/wz
            print(f"la masse de l'effecteur mobile est {m} kg")
            print(f"l'amortissement du falcon est {c_intern} kg/s")

        if self.mode==3:
            plt.figure("Force renvoyé par le mur selon y")
            plt.plot(self.posy, self.f_mur_set, color='red', label='set')
            plt.plot(self.posy, self.f_mur_get, color='blue', label='get')
            
            plt.title("Mur n = y")
            plt.xlabel('Position en x m')
            plt.ylabel('Forces N')
            plt.legend()
            
        
        if self.mode in [2,3,4,5,6,7,8] :
            print('plot 3D')
        
            ax = plt.figure().add_subplot(projection='3d')
        
            x, y, z = self.posx, self.posy, self.posz
        
            # Récupération du premier vecteur (ou unique)
            u,v,w=[],[],[]
            for l in self.f_get:
                u.append(l[0])
                v.append(l[1])
                w.append(l[2])
        
            ax.quiver(
                x, y, z,
                u, v, w,
                length=0.00002,              # augmente la taille
                normalize=False,          # utilise la vraie taille du vecteur
                linewidth=1,              # épaissit la flèche
                arrow_length_ratio=0.2,   # agrandit la pointe
                color='blue'               # optionnel
            )
            
            
            
            ax.set_title('1 mur')
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            


    
        plt.show()



    def run(self):
        """ 
        RUNNNNN
        """

        ti=time.time_ns()
        dhdOpen()
        while(not self.done):

            self.update()
            
            if self.mode==0:
                print(self.pz)
                

            elif self.mode==1:
                self.ressort()
             
            elif self.mode==2:
                self.amortisseur()

            elif self.mode==3:
                self.mur1()
                
            elif self.mode==4:
                self.mur2()
                
            elif self.mode==5:
                # plusieurs murs, 2 mur penchés une arrete suivant y exemple
                n= [Vecteur3d(0, 1, 1),Vecteur3d(0,-1,1)]
                p0 = [Vecteur3d(0, 0, 0),Vecteur3d(0, 0,0)]
                for i in range(0,len(n)):
                    fvec=self.mur_init(p0[i],n[i])
               
                    self.fx=self.fx+ fvec.x
                    self.fy= self.fy + fvec.y
                    self.fz= self.fz + fvec.z
                    
            elif self.mode==6:
                
                    fvec=self.sphere()
               
                    self.fx=self.fx+ fvec.x
                    self.fy= self.fy + fvec.y
                    self.fz= self.fz + fvec.z
                    
            elif self.mode==7:
                n= [Vecteur3d(1, 0, 0),Vecteur3d(-1, 0, 0),Vecteur3d(0, 1, 0),Vecteur3d(0, -1, 0),Vecteur3d(0, 0, 1),Vecteur3d(0, 0, -1)]
                p0 = [Vecteur3d(-0.01, 0, 0),Vecteur3d(0.01, 0, 0),Vecteur3d(0, -0.01, 0),Vecteur3d(0, 0.01, 0),Vecteur3d(0, 0, 0),Vecteur3d(0, 0, 0.02)]
                for i in range(0,len(n)):
                    fvec=self.mur_init(p0[i],n[i])
               
                    self.fx=self.fx+ fvec.x
                    self.fy= self.fy + fvec.y
                    self.fz= self.fz + fvec.z
                    
            elif self.mode==8:
 
                n = [Vecteur3d(1,0,0)]
                p0 = [Vecteur3d(0,0,0)]
                
                for i in range(0,len(n)):
                    fvec=self.mur_vib(p0[i],n[i])
               
                    self.fx=self.fx+ fvec.x
                    self.fy= self.fy + fvec.y
                    self.fz= self.fz + fvec.z
                
                
                
            
         
                
                

            self.set_force(self.fx,self.fy,self.fz)
            self.f_get.append([self.fx,self.fy,self.fz])
            self.fx,self.fy,self.fz=0,0,0
          
            self.t.append(abs(ti-time.time_ns()))

            self.done=dhdGetButton(0)


        tf= abs(ti-time.time_ns())
        print(f"temps de la simulaiton : {tf} s")    
        dhdClose()


        



def main():
    
    mode=[0,1,2,3,4,5,6,7,8] # 0: libre, 1: ressort, 2: amortisseur, 3: mur (ressort), 4: mur(ressort+amortisseur) ,5: plusieurs murs, 6 sphere, 7 cube, 8 mur texturé
    print(mode[0])
    sim= Simulation(mode[8])
    sim.run()
    sim.plot()

if __name__ == '__main__':
    main()





