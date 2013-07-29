#visualization code by berthy424 and Marcello79

from Tkinter import *
from impl import *
#from particlefilters import *
#from math import *

#import random
from time import sleep
import numpy as np
import matplotlib.pyplot as plt
#***************************************
class DispParticleFilter(Tk):

    '''frameLength is the delay between two frames, i.e. two steps of the filter'''
    def __init__(self, motions, N=1000, N_random=500,frameLength = 0.1, displayRealRobot = True, displayGhost = False,flag=1,draw_flag=1 ):
        Tk.__init__(self)
        self.title( 'Diplay Particle Filter CS373-HW03.06')
        self.motions = motions        
        self.N = N
        self.N_random=N_random
        self.diff_p=[]
        self.diff_p_random=[]
        self.frameLength = frameLength
        self.displayRealRobot = displayRealRobot        
        self.displayGhost = displayGhost
        self.flag=flag
        self.draw_flag=draw_flag
        #init particle filter
        self.initFilter()
        # Drawing
        self.margin = 130                                  # margin
        self.zoom_factor = 2                                # zoom factor
        self.playing = False
        self.can = DisplayParticles ( self.margin, self.zoom_factor )
        self.can.configure(bg ='ivory', bd =2, relief=SUNKEN)
        self.can.pack(side =TOP, padx =5, pady =5)
    
        self.can.draw_all(self.p, self.robot, self.displayRealRobot, self.displayGhost)
        #Buttons
        self.buttonReset = Button(self, text ='Reset', command =self.resetFilter)
        self.buttonReset.pack(side =LEFT, padx =5, pady =5)
        self.buttonNext = Button(self, text ='Next step', command =self.nextStep)
        self.buttonNext.pack(side =LEFT, padx =5, pady =5)
        self.buttonPlay = Button(self, text ='Play', command =self.play)
        self.buttonPlay.pack(side =LEFT, padx =5, pady =5)
        self.buttonPause = Button(self, text ='Pause', command =self.pause)
        self.buttonPause.pack(side =LEFT, padx =5, pady =5)    
        self.buttonPause.configure(state=DISABLED)         
        #Label
        textLabel = 'Current state = ' + str(self.actualState+1) + '/' + str(len(motions))
        self.label = Label(self, text = textLabel )
        self.label.pack(side = BOTTOM, padx =5, pady =5)
        self.diff_kidnapp=[]
        self.diff_kidnapp_random=[]
        
        #self.sum_random_array=np.zeros((1,50))
        self.counter=0.0
    def resetFilter(self):
        self.pause()
        self.diff_p=[]
        self.diff_p_random=[]
        
        self.initFilter()
        #Replot all
        #self.can.draw_all(self.p, self.robot, self.displayRealRobot, self.displayGhost)

    def initFilter (self):

        #New Robot's position
        self.robot = robot(init=1)
        self.robot.set_noise(bearing_noise, steering_noise, distance_noise,turn_noise,forward_noise)

        # Make particles            
        self.p = []                  # p : particles set
        self.p_random=[]
        for i in range(self.N):
            r = robot()
            r.set_noise(bearing_noise, steering_noise, distance_noise,turn_noise,forward_noise)
            self.p.append(r)
        # --------------
        for i in range(self.N_random):
                    r = robot()
                    r.set_noise(bearing_noise, steering_noise, distance_noise,turn_noise,forward_noise)
                    self.p_random.append(r)        
        #self.p_random=self.p
        self.actualState = 0

    def nextStep (self, event=None):
        global counter
        #global counter
        global shifted_array
        #self.sum_shfited_array=np.zeros((1,50))
        global random_array  
        self.actualState = self.actualState + 1
        if self.actualState < len(self.motions):
            #Label
            stateString = 'Actual state = ' + smtr(self.actualState+1) + '/' + str(len(motions))
            self.label.configure( text = stateString )
            # motion update (prediction)
            #print self.actualState
            if self.actualState==50: #or self.actualState==75 or self.actualState==125 or self.actualState==175:
                #self.robot_past=self.robot
                self.robot=robot(init=1)
                #self.robot
                self.p_random=[]
                for i in range(self.N_random):
                            r = robot()
                            r.set_noise(bearing_noise, steering_noise, distance_noise,turn_noise,forward_noise)
                            self.p_random.append(r)                
                
                #if self.flag==1:
                #    self.robot=robot(init=1)
                #else:
                #    self.initFilter()
                print 'I am kidnapped'
                flag=1                
    
            #if(random.random() <= 0.90):
              
            else:
                self.robot = self.robot.move((self.motions[self.actualState]))
                flag=0                
                
                #self.robot = robot(init=1)
              
            if self.draw_flag==1:
                self.can.draw_all(self.p, self.robot, self.displayRealRobot, self.displayGhost)
            self.update()
            #print bearing_noise
            sleep(self.frameLength)
            p2 = []
            p2_random=[]
            for i in range(self.N):
                p2.append(self.p[i].move(self.motions[self.actualState]))
            for i in range(self.N_random):
                
                p2_random.append(self.p_random[i].move(self.motions[self.actualState]))
            self.p = p2
            self.p_random=p2_random
            if self.draw_flag==1:
                
                self.can.draw_all(self.p, self.robot, self.displayRealRobot, self.displayGhost)
            self.update()
            
            sleep(self.frameLength)
            # measurement update
            w = []
            w_random=[]
            Z = self.robot.sense()
            for i in range(self.N):
                w.append(self.p[i].measurement_prob(Z))
            for i in range(self.N_random):
                w_random.append(self.p_random[i].measurement_prob(Z))
            # resampling
            p3 = []
            p3_random=[]
            index = int(random.random() * self.N)
            index_random=int(random.random() * self.N_random)
            beta = 0.0
            beta_random=0.0
            mw = max(w)
            mw_random=max(w_random)
            if self.flag==1:
                for i in range(int(0.90 * self.N)):
                                beta += random.random() * 2.0 * mw
                                while beta > w[index]:
                                    beta -= w[index]
                                    index = (index + 1) % self.N
                                p3.append(self.p[index])
                for i in range(self.N - int(0.90 * self.N)):
                                r = robot()
                                r.set_noise(bearing_noise, steering_noise, distance_noise,turn_noise,forward_noise)
                                p3.append(r) 
                #print 'computing random'
                for i in range(self.N_random):
                                beta_random += random.random() * 2.0 * mw_random
                                #print index_random
                                #print w_random[index_random]
                                while beta_random > w_random[index_random]:
                                     beta_random -= w_random[index_random]
                                     index_random = (index_random + 1) % self.N_random
                                     #print 'inside while'
                                #print 'outside while'
                                p3_random.append(self.p_random[index_random])                                         
             
            #print 'done computing'                    
            '''                    
            else:
                    
                for i in range(self.N):
                    beta += random.random() * 2.0 * mw
                    while beta > w[index]:
                        beta -= w[index]
                        index = (index + 1) % self.N
                    p3.append(self.p[index])            
            '''        
            '''    
            for i in range(int(0.90 * self.N)):
                beta += random.random() * 2.0 * mw
                while beta > w[index]:
                    beta -= w[index]
                    index = (index + 1) % self.N
                p3.append(self.p[index])
            for i in range(self.N - int(0.90 * self.N)):
                r = robot()
                r.set_noise(bearing_noise, steering_noise, distance_noise)
                p3.append(r)
            '''
            self.p = p3
            self.p_random=p3_random
        
            position_p=get_position(self.p)
            position_p_random=get_position(self.p_random)
            position_p=np.array((position_p[0],position_p[1]))
            position_p_random=np.array((position_p_random[0],position_p_random[1]))
            poistion_robot=np.array((self.robot.x,self.robot.y))
            self.diff_p.append(np.linalg.norm(position_p-poistion_robot))
            
            self.diff_p_random.append(np.linalg.norm(position_p_random-poistion_robot))
            for j in range(len(self.diff_p_random)):
                shifted_array[counter][j]=self.diff_p[j]
                random_array[counter][j]=self.diff_p_random[j]
            #plt.ion()    
            #plt.figure(1)
            #plt.hold(1)    
            #plt.plot(shifted_array[0],'b-')
            
            #plt.plot(random_array[0],'r-')
            #plt.show()
            #    self.sum_random_array[0][j]=sum(self.shifted_array[:,j])/(self.counter+1)
            #    self.sum_shfited_array[0][j]=sum(self.shifted_array[:,j])/(self.counter+1)
                
            #print self.diff_p,'the new one is',self.diff_p_random
            #if self.counter==2:
    
                #plt.ion()
                #plt.figure(1)
                #plt.hold(1)
                #plt.plot(self.sum_random_array,'r-')
                #plt.plot(self.sum_shfited_array,'b-')
            #if flag==1:
                #print 'I am in the loop'
                #self.diff_kidnapp.append(self.diff_p[len(self.diff_p)-1]-self.diff_p[len(self.diff_p)-2])
                #self.diff_kidnapp_random.append(self.diff_p_random[len(self.diff_p_random)-1]-self.diff_p_random[len(self.diff_p_random)-2])
             #   plt.plot(len(self.diff_p)-1,10,'y*')
                #plt.figure(2)
                #plt.plot(self.diff_kidnapp_random,'r-')
                #plt.plot(self.diff_kidnapp,'b-')
            
            #Replot all
            #hold(1)
            
            if self.draw_flag==1:
                self.can.draw_all(self.p, self.robot, self.displayRealRobot, self.displayGhost)
            #print 'displaying othe ghost'
                self.can.draw_random_particle(self.p_random)
            return True
        
        else:
            return False

    def play (self, event=None):
        self.playing = True
        global counter
        self.buttonPause.configure(state=NORMAL)  
        self.buttonNext.configure(state=DISABLED) 
        self.buttonPlay.configure(state=DISABLED) 
        for i in range(10):
            print 'I am here'
            self.playing=True
            while self.playing:
                print counter
                
                        
                if self.nextStep() == False:
                    counter=counter+1
                    #self.actualState=0
                    self.resetFilter()
                    print('returned false')
                    self.pause(event)
                    #self.buttonPlay.configure(state=DISABLED)  
                    #self.buttonNext.configure(state=DISABLED) 
                    break;
            
                self.update()
                sleep(self.frameLength)
                

    def pause (self, event=None):
        self.playing = False
        self.buttonPause.configure(state=DISABLED)  
        self.buttonNext.configure(state=NORMAL) 
        self.buttonPlay.configure(state=NORMAL)

class DisplayParticles(Canvas):

    def __init__(self, margin, zoom_factor ):
        Canvas.__init__(self)
        #self.p = p
        self.margin = margin
        self.zoom_factor = zoom_factor
        self.larg = (2*margin + world_size) * zoom_factor
        self.haut = self.larg
        self.configure(width=self.larg, height=self.haut )
        self.larg, self.haut = (2*margin + world_size) * zoom_factor, (2*margin + world_size) * zoom_factor
        # Landmarks
        self.landmarks_radius = 2
        self.landmarks_color = 'green'
        # Particles
        self.particle_radius = 1
        self.particle_color = 'red'
        # Robot
        self.robot_radius = 4
        self.robot_color = 'blue'
        self.ghost_color = None
        self.ghost_color_random='yellow'
    def draw_random_particle(self,p_random):
        ghost_random=get_position(p_random)
        self.plot_robot( ghost_random[0], ghost_random[1], ghost_random[2], self.robot_radius, self.ghost_color_random)

    def draw_all(self, p, realRob, displayRealRobot, displayGhost):
        #print len(p)
        self.configure(bg ='ivory', bd =2, relief=SUNKEN)
        self.delete(ALL)
        self.p = p
        self.plot_particles()

        if displayGhost:
            ghost = get_position(self.p)
            self.plot_robot( ghost[0], ghost[1], ghost[2], self.robot_radius, self.ghost_color)
        self.plot_landmarks( landmarks, self.landmarks_radius, self.landmarks_color )

        if displayRealRobot:
            self.plot_robot( realRob.x, realRob.y, realRob.orientation, self.robot_radius, self.robot_color)

    def plot_landmarks(self, lms, radius, l_color ):
        for lm in lms:
            x0 = (self.margin + lm[1] - radius) * self.zoom_factor
            y0 = (self.margin + lm[0] - radius) * self.zoom_factor
            x1 = (self.margin + lm[1] + radius) * self.zoom_factor
            y1 = (self.margin + lm[0] + radius) * self.zoom_factor
            self.create_oval( x0, y0, x1, y1, fill = l_color )

    def plot_particles(self):
        for particle in self.p:
            self.draw_particle( particle, self.particle_radius, self.particle_color )

    def draw_particle(self, particle, radius, p_color):
        #x0 = (self.margin + particle.x - radius) * self.zoom_factor
        #y0 = (self.margin + particle.y - radius) * self.zoom_factor
        #x1 = (self.margin + particle.x + radius) * self.zoom_factor
        #y1 = (self.margin + particle.y + radius) * self.zoom_factor
        #self.create_oval( x0, y0, x1, y1, fill = p_color )
        x2 = (self.margin + particle.x) * self.zoom_factor
        y2 = (self.margin + particle.y) * self.zoom_factor
        x3 = (self.margin + particle.x + 2*radius*cos(particle.orientation)) * self.zoom_factor
        y3 = (self.margin + particle.y + 2*radius*sin(particle.orientation)) * self.zoom_factor
        self.create_line( x2, y2, x3, y3, fill = p_color, width =self.zoom_factor,
                          arrow=LAST, arrowshape=(2*self.zoom_factor,
                                                  3*self.zoom_factor,
                                                  1*self.zoom_factor) )

    def plot_robot(self, x,y, orientation, radius, r_color):
        x0 = (self.margin + x - radius) * self.zoom_factor
        y0 = (self.margin + y - radius) * self.zoom_factor
        x1 = (self.margin + x + radius) * self.zoom_factor
        y1 = (self.margin + y + radius) * self.zoom_factor
        self.create_oval( x0, y0, x1, y1, fill = r_color )
        x2 = (self.margin + x) * self.zoom_factor
        y2 = (self.margin + y) * self.zoom_factor
        x3 = (self.margin + x + 2*radius*cos(orientation)) * self.zoom_factor
        y3 = (self.margin + y + 2*radius*sin(orientation)) * self.zoom_factor
        self.create_line( x2, y2, x3, y3, fill = r_color, width =self.zoom_factor, arrow=LAST )

#**************************************************

if __name__ == "__main__":
    shifted_array=np.zeros((10,100))
            #self.sum_shfited_array=np.zeros((1,50))
    random_array=np.zeros((10,100))
    counter=0
    #motions  ( here copy of the dataset in hw3-6 )
    number_of_iterations = 100
    motions = [[2. * pi / 20, 12.] for row in range(number_of_iterations)]
    # 1 for uniform distribution
    # 0 for reseting the particle filter
    #Display window
    wind = DispParticleFilter ( motions, 500,500, displayRealRobot = True, displayGhost = True,flag=1,draw_flag=0)
    #wind1 = DispParticleFilter ( motions, 1000, displayRealRobot = True, displayGhost = True,flag=0 )
    wind.mainloop()
    
    #wind1.mainloop()