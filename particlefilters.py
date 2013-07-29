
# Now we want to simulate robot
# motion with our particles.
# Each particle should turn by 0.1
# and then move by 5. 
#
#
# Don't modify the code below. Please enter
# your code at the bottom.
from numpy import *

from pylab import *
from math import *
import random

landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0


class robot:
    def __init__(self):
        self.x = random.random() * world_size	
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;
    
    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError, 'X coordinate out of bound'
        if new_y < 0 or new_y >= world_size:
            raise ValueError, 'Y coordinate out of bound'
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
    
    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);
    
    
    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z
    
    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError, 'Robot cant move backwards'         
        
        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi
        
        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size    # cyclic truncate
        y %= world_size
        
        # set particle
        res = robot()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res
    
    def Gaussian(self, mu, sigma, x):
        
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
    
    
    def measurement_prob(self, measurement):
        
        # calculates how likely a measurement should be
        
        prob = 1.0;
        dist_sum=0.0;
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
            dist_sum+=dist
        return prob,dist_sum
    # so you actually move but the sensor reading would point otherwise
    def kidnapp(self):
        self.x = random.random() * world_size	
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi        
        
    
    
    
    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))



def eval(r, p):
    sum = 0.0;
    for i in range(len(p)): # calculate mean error
        dx = (p[i].x - r.x + (world_size/2.0)) % world_size - (world_size/2.0)
        dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
        err = sqrt(dx * dx + dy * dy)
        sum += err
    return sum / float(len(p))



def get_position(p):
    x = 0.0
    y = 0.0
    orientation = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (((p[i].orientation - p[0].orientation + pi) % (2.0 * pi)) 
                        + p[0].orientation - pi)
    return [x / len(p), y / len(p), orientation / len(p)]

#myrobot = robot()
#myrobot.set_noise(5.0, 0.1, 5.0)
#myrobot.set(30.0, 50.0, pi/2)
#myrobot = myrobot.move(-pi/2, 15.0)
#print myrobot.sense()
#myrobot = myrobot.move(-pi/2, 10.0)
#print myrobot.sense()

####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####

myrobot = robot()

N = 500
p = []
test=arange(1,10)
world=zeros((world_size,world_size))

for i in range(4):
    world[landmarks[i][0],landmarks[i][1]]=1
    #hold(1)    
    #plot(landmarks[i][0],landmarks[i][1],'bo')
    
    #plot(test)
    
    
#show()
#print landmarks[1][1]
print myrobot.x,myrobot.y

#show()
world[myrobot.x,myrobot.y]=2
#print  world[landmarks[i][0],landmarks[i][1]]
print world
ion()
for i in range(N):
        x = robot()
        #world[x.x,x.y]=3
        #raw_input("Press enter to see the robot move")
        #print world
        x.set_noise(0.05,0.05,5.0)
        p.append(x)
w_mean=[]
for t in range(10):
    
    clf()
    for i in range(4):
        hold(1)
        plot(landmarks[i][0],landmarks[i][1],'bo')
    '''
    if t==1:
        for i in range(N):
            
            world[p[i].x,p[i].y]=3
    '''    
    world[myrobot.x,myrobot.y]=0
    myrobot = myrobot.move(0.1,5.0) #turn,forward
    plot(myrobot.x,myrobot.y,'r^')
    world[myrobot.x,myrobot.y]=2
    #Z_before=myrobot.sense()
    #print world
    
    '''
    myrobot_before_x=myrobot.x
    myrobot_before_y=myrobot.y
    '''
    '''
    if t==3:
        print 'I am kidnapped'
        myrobot.kidnapp()
        plot(myrobot.x,myrobot.y,'rs')
    '''
    '''
        p=[]
        for i in range(N):
            
                x = robot()
                #world[x.x,x.y]=3
                #raw_input("Press enter to see the robot move")
                #print world
                x.set_noise(0.05,0.05,5.0)
                p.append(x)  
    '''
    #print myrobot,myrobot_before
    '''
    if myrobot_before_x==myrobot.x or myrobot_before_y==myrobot.y:
        print 'not kidnapp'
    else:
        print 'kidnapped'
    '''
    Z = myrobot.sense()
    '''
    if Z==Z_before:
        print 'not possible'
    else:
        print 'strange'
    '''
    #print p #PLEASE LEAVE THIS HERE FOR GRADING PURPOSES
    
    p2 =[]
    for i in range(N):
        p2.append(p[i].move(0.1,5.0))
            
        #raw_input("Press enter to see the robot move")	
        #world[p2[i].x,p2[i].y]=3
        #print world
    #print p2[1]
    #print p2[1].x
    '''
    x.set_noise(0.0,0.0,0.0)
    p4=[]
    
    for i in range(N):
        p4.append(p[i].move(0.1,5.0))
    diff=[]
    for i in range(N):
        diff.append(p2[i].x-p4[i].x) # this should give me the difference, I am not using abs as there should be some values less than or more 
    x.set_noise(0.05,0.05,5.0)
    # now if we set the noise and then move the particles then we can get the distribution of the noise
    '''
    p=p2
    
    
    w=[]
    
    
    for i in range(N):
        prob_sensor,dist_sensor=p[i].measurement_prob(Z)
        w.append(prob_sensor)
        #dist_w.append(dist_sensor)
    '''    
    w_mean.append(mean(w)) # we can check this with the current w and that will give us a idea if the robot is localized or not
    
    figure(2)
    clf()
    plot(w_mean)
    '''
    figure(1)
    #figure(1)
    #print w
    
    
    #resampling step
    
    p3=[]
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in range(int(N)):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index +1) % N
            
        p3.append(p[index])
        #print index
    #p=p3
    # you don't need this now because we are telling it that it is kidnapped
    
    '''
    for i in range(N-(int(0.9*N))):
        random_par = robot()
        random_par.set_noise(0.05,0.05,0.000011)
        p3.append(random_par)
    '''   
    p=p3
    
    #print p
    print len(p)
    #if t==9:
    '''   
    for i in range(N):
        plot(p[i].x,p[i].y,'yo')
        world[p[i].x,p[i].y]=3
    '''    
    print 'The actual location of the robot',myrobot
    particle_location=get_position(p)
    print 'The predicted location',particle_location    
    plot(particle_location[0],particle_location[1],'r*')
    raw_input("Press enter to see the robot move")
    #print world
    #print p
    #print "the actual location is"
    
    
    #show()
    #clf()
#print p
'''
print("the actual location is")
#print diff
mean=sum(diff)/N
var=sum(abs((diff-mean)))/N
print mean,var
'''
print myrobot
#print p


#print p