
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
import sklearn.mixture

gmm=sklearn.mixture.GMM()
landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 400.0


class robot:
    def __init__(self):
        self.x = random.random() * world_size	
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.forward_noise_mean=0.0
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
    
    def move_real(self, turn, forward,forward_noise,turn_noise):
            if forward < 0:
                raise ValueError, 'Robot cant move backwards'         
            mean_dist_d=0.0
            mean_dist_t=0.0
            var_dist_d=forward_noise
            #print 'forward noise',forward_noise
            #print 'turn noise',turn_noise
            var_dist_t=turn_noise
            mean_turn_d=0.0
            mean_turn_t=0.0
            var_turn_t=turn_noise
            var_turn_d=forward_noise
            
            dist=random.gauss(forward*mean_dist_d+turn*mean_dist_t,forward**2*var_dist_d+turn**2*var_dist_t)
            #print 'distance is',dist
            # turn, and add randomness to the turning command
            turn_dist=random.gauss(forward*mean_turn_d+turn*mean_turn_t,forward**2*var_turn_d+turn**2*var_turn_t)
            #print 'turn is',turn_dist
            orientation = self.orientation + float(turn_dist) 
            orientation %= 2 * pi
            '''
            # move, and add randomness to the motion command
            
            x = self.x + (cos(orientation) * dist)
            y = self.y + (sin(orientation) * dist)
            x %= world_size    # cyclic truncate
            y %= world_size
            '''
            # now make a new motion model according to Elzar paper
            #dist=float(forward)+random.gauss(0,0,self.forward_noise)
            # according to Elzar paper dist comes from a distribtuion
            
            orientation_new=self.orientation+float(turn_dist)/2
            orientation_new %= 2*pi
            x=self.x+(cos(orientation_new)*dist)
    
            y = self.y + (sin(orientation_new) * dist)
            
            x %= world_size    # cyclic truncate
            y %= world_size        
            
            
            # set particle
            res = robot()
            res.set(x, y, orientation)
            res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
            return res    
    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError, 'Robot cant move backwards'         
        mean_dist_d=self.forward_noise_mean
        mean_dist_t=0.0
        var_dist_d=self.forward_noise
        var_dist_t=self.turn_noise
        mean_turn_d=0.0
        mean_turn_t=0.0
        var_turn_t=self.turn_noise
        var_turn_d=self.forward_noise
        dist=random.gauss(forward*mean_dist_d+turn*mean_dist_t,forward**2*var_dist_d+turn**2*var_dist_t)
        
        # turn, and add randomness to the turning command
        turn_dist=random.gauss(forward*mean_turn_d+turn*mean_turn_t,forward**2*var_turn_d+turn**2*var_turn_t)
        
        orientation_real = self.orientation + float(turn_dist) 
        orientation_real %= 2 * pi
        '''
        # move, and add randomness to the motion command
        
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size    # cyclic truncate
        y %= world_size
        '''
        # now make a new motion model according to Elzar paper
        #dist=float(forward)+random.gauss(0,0,self.forward_noise)
        # according to Elzar paper dist comes from a distribtuion
        
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
    def move_velocity(self, turn_velocity, forward_velocity):
        time=1.0
        if forward_velocity < 0:
            raise ValueError, 'Robot cant move backwards'   
        #print self.turn_noise
        orientation=self.orientation+turn_velocity*time+random.gauss(0.0,self.turn_noise)
        orientation %= 2 * pi
        forward_velocity=forward_velocity+random.gauss(0.0,self.forward_noise)
        x= self.x-(forward_velocity/turn_velocity)*sin(self.orientation)+(forward_velocity/turn_velocity)*sin(self.orientation+turn_velocity*time)
        y= self.y+(forward_velocity/turn_velocity)*cos(self.orientation)-(forward_velocity/turn_velocity)*cos(self.orientation+turn_velocity*time)
         # they also add a random term but I am not using it
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



####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####

myrobot = robot()

N = 500
p = []
p_original=[]
test=arange(1,10)
world=zeros((world_size,world_size))

for i in range(4):
    world[landmarks[i][0],landmarks[i][1]]=1

#print myrobot.x,myrobot.y

#show()
world[myrobot.x,myrobot.y]=2
#print  world[landmarks[i][0],landmarks[i][1]]
#print world
ion()
diff_position=[]
diff_position_original=[]
for i in range(N):
        x = robot()
    
        x.set_noise(0.05,0.05,5.0)
        p.append(x)
        #p_original.append(x)
w_mean=[]

for t in range(20):
    
    clf()
    
    for i in range(4):
        hold(1)
        plot(landmarks[i][0],landmarks[i][1],'bo')

    world[myrobot.x,myrobot.y]=0
    if t<=5:
        myrobot = myrobot.move_velocity(0.1,1.0) #turn,forward
    else:
        #print ' I am on a different terrain'
        myrobot=myrobot.move_velocity(0.1,1.0)
    #myrobot=myrobot.move_real(0.0,5.0,0.05,0.05)
    
    plot(myrobot.x,myrobot.y,'r^')
    world[myrobot.x,myrobot.y]=2

    
   
    Z=myrobot.sense()
    p2 =[]
    p2_original=[]
    for i in range(N):
        p2.append(p[i].move_velocity(0.1,1.0)) # turn,forward
        #p2_original.append(p_original[i].move_real(0.0,5.0,0.05,0.05))
            
      
    
    
    w=[]
    w_original=[]
    p_previous=p
    #p_previous_original=p_original
    p=p2
    p_original=p2_original
    # I think there is a missing p=p2 .. need to verify this
    for i in range(N):
        prob_sensor,dist_sensor=p[i].measurement_prob(Z)
        #prob_sensor_original,dist_sensor_original=p_original[i].measurement_prob(Z)
        
        #print prob_sensor
        w.append(prob_sensor)
        #w_original.append(prob_sensor_original)
        
        #dist_w.append(dist_sensor)
    #print 'the difference is',diff_odom
    #figure(1)
    
    figure(1)
    #print w
    
    
    #resampling step
    
    p3=[]
    p3_original=[]
    index = int(random.random() * N)
    index_original=int(random.random()*N)
    beta = 0.0
    beta_original=0.0
    mw = max(w)
    #mw_original=max(w_original)
    diff_odom_x=[]
    #diff_odom_x_original=[]
    
    for i in range(int(N)):
        beta += random.random() * 2.0 * mw
        #beta_original +=random.random() * 2.0 * mw_original
        while beta > w[index]:
            beta -= w[index]
            index = (index +1) % N
        #while beta_original > w_original[index_original]:
        #    beta_original -= w_original[index_original]
        #    index_original=(index_original+1)%N
            
        p3.append(p[index])
        #p3_original.append(p_original[index_original])
    #for i in range(int(N)):
     #   diff_odom_x.append(w[i]*(np.sqrt(((p[i].x-p_previous[i].x)**2)+((p[i].y-p_previous[i].y)**2))))
        #diff_odom_x_original.append(np.sqrt(((p_original[index].x-p_previous_original[index].x)**2)+((p_original[index].y-p_previous_original[index].y)**2)))
        
    
    
    
    #diff_odom=np.asarray(diff_odom_x)
    #diff_odom=diff_odom-5.0
    
    #r=gmm.fit(diff_odom)
    #print 'the mean with fit model is',r.means_[0,0],'and the variance is',r.covars_[0,0]
    #myrobot.forward_noise=r.covars_[0,0]#np.var(diff_odom)
    #myrobot.forward_noise_mean=r.means_[0,0]#np.mean(diff_odom)    
    #print 'the mean is',np.mean(diff_odom)
    #print 'the variance is',np.var(diff_odom)
    #print 'the difference in the x is',diff_odom_x
   
    
 
    p=p3
    #p_original=p3_original
    #print p
    #print len(p)
    
    print 'The actual location of the robot',myrobot
    particle_location=get_position(p)
    #particle_location_original=get_position(p_original)
    #diff_position.append(np.sqrt(((myrobot.x-particle_location[0])**2)+((myrobot.y-particle_location[1])**2)))
    #diff_position_original.append(np.sqrt(((myrobot.x-particle_location_original[0])**2)+((myrobot.y-particle_location_original[1])**2)))
    print 'The predicted location',particle_location    
    #print 'the original location is',particle_location_original
    #plot(particle_location[0],particle_location[1],'r*')
    #raw_input("Press enter to see the robot move")

print myrobot
#print diff_position
#print diff_position_original
#clf()
#plot(diff_position)

raw_input("Press enter to see the robot move")
#print p


#print p