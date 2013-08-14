
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
from scipy import optimize

landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0


class robot(object):
    def __init__(self):
        self.x = random.random() * world_size	
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;
        self.weight=0.0
        self.forward_prob=0.0
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
            
            mean_dist_t=0.0
            var_dist_d=self.forward_noise
            var_dist_t=self.turn_noise
            mean_turn_d=0.0
            mean_turn_t=0.0
            var_turn_t=self.turn_noise
            var_turn_d=self.forward_noise
            var_trans_independent=self.forward_noise
            var_rot_independent=self.turn_noise
            dist=random.gauss(forward,forward**2*var_dist_d+turn**2*var_dist_t+var_trans_independent)
            
            # turn, and add randomness to the turning command
            turn_dist=random.gauss(turn,forward**2*var_turn_d+turn**2*var_turn_t+var_rot_independent)
            
            orientation_real = self.orientation + float(turn_dist) 
            orientation_real %= 2 * pi
         
            orientation_new=self.orientation+float(turn_dist)/2
            orientation_new %= 2*pi
            x=self.x+(cos(orientation_new)*dist)
    
            y = self.y + (sin(orientation_new) * dist)
            
            x %= world_size    # cyclic truncate
            y %= world_size        
            
            
            # set particle
            res = robot()
            res.set(x, y, orientation_real)
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




def smooth(p_history,time,weight_filtered,p_filtered):
    #print 'the history is',p_history[1][0].weight,p_history[1][0].forward_prob
    print 'the length of p_history is',len(p_history[0]) 
    traj=[]
    index = int(random.random() * N)
    beta = 0.0
    mw = max(weight_filtered)
    #print 'the mx is',mw
            
    for k in range(1):
        beta += random.random() * 2.0 * mw
        while beta > weight_filtered[index]:
            beta -= weight_filtered[index]
            index = (index +1) % N
                    
        traj.append((p_filtered[index].x,p_filtered[index].y,p_filtered[index].orientation))        
    for i in range(t-1,-1,-1):
        #print i
        weight=[]
        for j in range(N):
            p_history[j][i].weight=p_history[j][i].weight*p_history[j][i].forward_prob
            #weight.append(p_history[j][i].weight*p_history[j][i].forward_prob)
            weight.append(p_history[j][i].weight)
    
        index = int(random.random() * N)
        beta = 0.0
        mw = max(weight)
        #print 'the mx is',mw
        
        for k in range(1):
            beta += random.random() * 2.0 * mw
            while beta > weight[index]:
                beta -= weight[index]
                index = (index +1) % N
                
            traj.append((p_history[index][i].x,p_history[index][i].y,p_history[index][i].orientation))    
        
    return traj      


def error_calculation(traj,trans_readings,rotation_readings):
    error_trans=np.zeros((len(traj)))
    error_rotation=np.zeros((len(traj)))
    print len(traj)
    for i in range(len(traj)-1):
        error_rotation[i]=traj[i][2]-traj[i+1][2]-rotation_readings[i]
        error_trans[i]=((traj[i+1][0]-traj[i][0])*cos(traj[i][2]+(rotation_readings[i]+error_rotation[i])/2)+(traj[i+1][1]-traj[i][1])*sin(traj[i][2]+(rotation_readings[i]+error_rotation[i])/2))-trans_readings[i]
    print 'the errors are',error_rotation,error_trans
    return error_rotation,error_trans    


def f(x,*args):
    u, v,z = x
    #temp=0
    #for i in range(counter):
        #args = (param[i],3, 7, 8, 9, 10)
    a, b, c = args
    return sum(np.log(2*np.pi)+np.log(a[0:counter]*u + b[0:counter]*v + z)+((c[0:counter]**2)/a[0:counter]*u+b[0:counter]*v+z))*(-0.5)
    #temp=temp*(-1*0.5)    
    #return temp

def gradf(x,*args):
    u, v,z = x
    #print 'the constants are',u,v,z
    #tmp_gu=0
    #tmp_gv=0 
    #tmp_gz=0
    #for i in range(counter):
        #args = (param[i],3, 7, 8, 9, 10)
    a, b, c = args
    #print 'the args are',a[i],b[i],c[i]
    gu = sum((a[0:counter]/a[0:counter]*u+b[0:counter]*v+z)-((c[0:counter]**2)*a[0:counter]/((a[0:counter]*u+b[0:counter]*v+z)**2)))*(-0.5)    # u-component of the gradient
    gv = sum((b[0:counter]/a[0:counter]*u+b[0:counter]*v+z)-((c[0:counter]**2)*b[0:counter]/((a[0:counter]*u+b[0:counter]*v+z)**2)))*(-0.5)   # v-component of the gradient
    gz = sum((1/a[0:counter]*u+b[0:counter]*v+z)-((c[0:counter]**2)/((a[0:counter]*u+b[0:counter]*v+z)**2)))*(-0.5)    
    #tmp_gu+=gu
    #tmp_gv+=gv
    #tmp_gz+=gz
    #tmp_gu=tmp_gu*(-1*0.5)
    #tmp_gv=tmp_gv*(-1*0.5)
    #tmp_gz=tmp_gz*(-1*0.5)
    #print type(tmp_gv)
    return np.asarray((gu,gv,gz))



myrobot = robot()

N = 500
p = []
test=arange(1,10)
world=zeros((world_size,world_size))
distance_reported=np.zeros((10))

rotation_reported=np.zeros((10))

for i in range(4):
    world[landmarks[i][0],landmarks[i][1]]=1
    #hold(1)    
    #plot(landmarks[i][0],landmarks[i][1],'bo')
    
    #plot(test)
    
    
#show()
#print landmarks[1][1]
print myrobot.x,myrobot.y
p_history=np.ndarray((500,10),dtype=np.object)
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
counter=0
for t in range(10):
    
    clf()
    counter=t
    for i in range(4):
        hold(1)
        plot(landmarks[i][0],landmarks[i][1],'bo')
    world[myrobot.x,myrobot.y]=0
    myrobot = myrobot.move(0.1,5.0) #turn,forward
    plot(myrobot.x,myrobot.y,'r^')
    world[myrobot.x,myrobot.y]=2
    #Z_before=myrobot.sense()
    #print world
    
  
    #print p #PLEASE LEAVE THIS HERE FOR GRADING PURPOSES
    Z=myrobot.sense()
    p2 =[]
    for i in range(N):
        p2.append(p[i].move(0.1,5.0))
            
       
    p=p2
    
    
    w=[]
    
    
    for i in range(N):
        prob_sensor,dist_sensor=p[i].measurement_prob(Z)
        w.append(prob_sensor)
        p[i].weight=prob_sensor
        #dist_w.append(dist_sensor)
   
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

    p=p3
    for i in range(len(p)):
        
        p_history[i][t]=p[i]

    #print p
   
    print 'the history is',p_history[1][0].weight,p_history[1][0].forward_prob
    
    trajectory=smooth(p_history,t,w,p)
    
    distance_reported[t]=5
    rotation_reported[t]=0.1
    print' testing',trajectory
    if len(trajectory)>1:
        
        rotation_error,tran_error=error_calculation(trajectory,distance_reported,rotation_reported)
        print 'trajectory is',trajectory
    
        args = (distance_reported,rotation_reported,tran_error)
        x0=np.asarray((0.1,0.1,0.1))
    
        res1 = optimize.fmin_cg(f, x0, fprime=gradf, args=args)
        print 'res1=',res1    
    
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

print myrobot
#print p


#print p