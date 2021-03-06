
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
import scipy.spatial.distance as sp

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
        self.independent_noise_trans=0.0
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
    
    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise,new_t_inde_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);
        self.independent_noise_trans = float(new_t_inde_noise)
    
    def sense(self,sense_noise):
        Z = []
        #print self.sense_noise
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0,sense_noise)
            Z.append(dist)
        return Z
    
    def move_real_distribution(self, turn, forward,forward_noise,turn_noise,independent_noise_trans,forward_noise_turn,turn_noise_turn,independent_noise_turn):
                    if forward < 0:
                        raise ValueError, 'Robot cant move backwards'         
                    
                    mean_dist_t=0.0
                    var_dist_d=forward_noise
                    var_dist_t=turn_noise
                    mean_turn_d=0.0
                    mean_turn_t=0.0
                    var_turn_t=turn_noise_turn
                    var_turn_d=forward_noise_turn
                    var_trans_independent=independent_noise_trans
                    var_rot_independent=independent_noise_turn
                    inital_x=self.x
                    inital_y=self.y
                    inital_theta=self.orientation
                    dist=random.gauss(forward,forward**2*var_dist_d+turn**2*var_dist_t+var_trans_independent)
                    
                    # turn, and add randomness to the turning command
                    turn_dist=random.gauss(turn,forward**2*var_turn_d+turn**2*var_turn_t+var_rot_independent)
                    
                    orientation_real = self.orientation + float(turn_dist) 
                    orientation_real %= 2 * pi
                 
                    orientation_new=self.orientation+float(turn_dist)/2
                    orientation_new %= 2*pi
                    x=self.x+(cos(orientation_new)*dist)
            
                    y = self.y + (sin(orientation_new) * dist)
                    temp_array=np.zeros((2,2))
                    temp_array[0][0]=inital_x
                    temp_array[0][1]=inital_y
                    temp_array[1][0]=x
                    temp_array[1][1]=y
                    distance_motion=sp.pdist(temp_array,'euclidean')
                    
                    x %= world_size    # cyclic truncate
                    y %= world_size        
                    
                    
                    # set particle
                    res = robot()
                    res.set(x, y, orientation_real)
                    self.forward_prob=self.Gaussian(distance_motion,forward_noise,forward)
                    #print res.forward_prob
                    res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise,self.independent_noise_trans)
                    return res          
    def move_real(self, turn, forward,drift,forward_noise,turn_noise,independent_noise_trans,forward_noise_turn,turn_noise_turn,independent_noise_turn):
                if forward < 0:
                    raise ValueError, 'Robot cant move backwards'         
                
                mean_dist_t=0.0
                var_dist_d=forward_noise
                var_dist_t=turn_noise
                mean_turn_d=0.0
                mean_turn_t=0.0
                var_turn_t=turn_noise_turn
                var_turn_d=forward_noise_turn
                var_trans_independent=independent_noise_trans
                var_rot_independent=independent_noise_turn
                #inital_x=self.x
                #inital_y=self.y
                #inital_theta=self.orientation
                dist=random.gauss(forward-drift,forward**2*var_dist_d+turn**2*var_dist_t+var_trans_independent)
                
                # turn, and add randomness to the turning command
                turn_dist=random.gauss(turn,forward**2*var_turn_d+turn**2*var_turn_t+var_rot_independent)
                
                orientation_real = self.orientation + float(turn_dist) 
                orientation_real %= 2 * pi
             
                orientation_new=self.orientation+float(turn_dist)/2
                orientation_new %= 2*pi
                x=self.x+(cos(orientation_new)*dist)
        
                y = self.y + (sin(orientation_new) * dist)
                temp_array=np.zeros((2,2))
                #temp_array[0][0]=inital_x
                #temp_array[0][1]=inital_y
                #temp_array[1][0]=x
                #temp_array[1][1]=y
                #distance_motion=sp.pdist(temp_array,'euclidean')
                
                x %= world_size    # cyclic truncate
                y %= world_size        
                
                
                # set particle
                res = robot()
                res.set(x, y, orientation_real)
                #res.forward_prob=self.Gaussian(distance_motion,forward_noise,forward)
                #print res.forward_prob
                res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise,self.independent_noise_trans)
                return res       
    
    def move_velocity(self, turn_velocity, forward_velocity):
            time=1.0
            if forward_velocity < 0:
                raise ValueError, 'Robot cant move backwards'         
            x= self.x-(forward_velocity/turn_velocity)*sin(self.orientation)+(forward_velocity/turn_velocity)*sin(self.orientation+turn_velocity*time)
            y= self.y+(forward_velocity/turn_velocity)*cos(self.orientation)-(forward_velocity/turn_velocity)*cos(self.orientation+turn_velocity*time)
            orientation=self.orientation+turn_velocity*time # they also add a random term but I am not using it
            x %= world_size    # cyclic truncate
            y %= world_size        
            
            
            # set particle
            res = robot()
            res.set(x, y, orientation)
            res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise,self.independent_noise_trans)
            return res   
    
    def Gaussian(self, mu, sigma, x):
        
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
    
    
    def measurement_prob(self, measurement,sensor_noise):
        
        # calculates how likely a measurement should be
        
        prob = 1.0;
        dist_sum=0.0;
        #print sense_noise
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist,sensor_noise, measurement[i])
            dist_sum+=dist
        return prob,dist_sum
    # so you actually move but the sensor reading would point otherwise
    def kidnapp(self):
        self.x = random.random() * world_size	
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi        
        
    
    
    
    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))

def move_particle(particle, turn, forward,drift,forward_noise,turn_noise,independent_noise_trans,forward_noise_turn,turn_noise_turn,independent_noise_turn):
                if forward < 0:
                    raise ValueError, 'Robot cant move backwards'         
                
                mean_dist_t=0.0
                var_dist_d=forward_noise
                var_dist_t=turn_noise
                mean_turn_d=0.0
                mean_turn_t=0.0
                var_turn_t=turn_noise_turn
                var_turn_d=forward_noise_turn
                var_trans_independent=independent_noise_trans
                var_rot_independent=independent_noise_turn
                #inital_x=self.x
                #inital_y=self.y
                #inital_theta=self.orientation
                dist=random.gauss(forward-drift,forward**2*var_dist_d+turn**2*var_dist_t+var_trans_independent)
                
                # turn, and add randomness to the turning command
                turn_dist=random.gauss(turn,forward**2*var_turn_d+turn**2*var_turn_t+var_rot_independent)
                
                orientation_real = particle[2] + float(turn_dist) 
                orientation_real %= 2 * pi
             
                orientation_new=particle[2]+float(turn_dist)/2
                orientation_new %= 2*pi
                x=particle[0]+(cos(orientation_new)*dist)
        
                y = particle[1] + (sin(orientation_new) * dist)
                #temp_array=np.zeros((2,2))
                #temp_array[0][0]=inital_x
                #temp_array[0][1]=inital_y
                #temp_array[1][0]=x
                #temp_array[1][1]=y
                #distance_motion=sp.pdist(temp_array,'euclidean')
                
                x %= world_size    # cyclic truncate
                y %= world_size        
                
                
                # set particle
                #res = robot()
                #res.set(x, y, orientation_real)
                #res.forward_prob=self.Gaussian(distance_motion,forward_noise,forward)
                #print res.forward_prob
                #res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise,self.independent_noise_trans)
                return [x,y,orientation_real]   
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
    #print 'the length of p_history is',len(p_history[0]) 
    traj=[]
    index = int(random.random() * N)
    beta = 0.0
    mw = max(weight_filtered)
    #print 'the mx is',mw
    for g in range(1):
        print g
        traj.append([])
        for k in range(1):
            beta += random.random() * 2.0 * mw
            while beta > weight_filtered[index]:
                beta -= weight_filtered[index]
                index = (index +1) % N
                        
            traj[g].append((p_filtered[index].x,p_filtered[index].y,p_filtered[index].orientation))      
        print 'filteration done'
        for i in range(t-1,-1,-1):
            #print i
            weight=[]
            for j in range(N):
                p_history[j][i].weight=p_history[j][i].weight*p_history[j][i].forward_prob
                #print p_history[j][i].weight
                #weight.append(p_history[j][i].weight*p_history[j][i].forward_prob)
                weight.append(p_history[j][i].weight)
        
            index = int(random.random() * N)
            beta = 0.0
            mw = max(weight)
            #print 'the mx is',mw
            print 'smmothed weight calc'
            for k in range(1):
                beta += random.random() * 2.0 * mw
                while beta > weight[index]:
                    beta -= weight[index]
                    index = (index +1) % N
                    
                traj[g].append((p_history[index][i].x,p_history[index][i].y,p_history[index][i].orientation))    
            print 'trjectory append'
    return traj      


def error_calculation(traj,trans_readings,rotation_readings):
    error_trans=np.zeros((len(traj),len(traj[0])))
    error_rotation=np.zeros((len(traj),len(traj[0])))
    #print ' the lenght is',len(traj[0])
    for i in range(len(traj)):
        for j in range(len(traj[i])-1):
            error_rotation[i][j]=np.mod((traj[i][j+1][2]-traj[i][j][2]-rotation_readings[j]),2*np.pi)#traj[i][j+1][2]-traj[i][j][2]# 
            error_trans[i][j]=((min(abs(traj[i][j+1][0]-traj[i][j][0]),100-abs(traj[i][j+1][0]-traj[i][j][0])))*cos(traj[i][j][2]+(rotation_readings[j]+error_rotation[i][j])/2)+(min(abs(traj[i][j+1][1]-traj[i][j][1]),100-abs(traj[i][j+1][1]-traj[i][j][1])))*sin(traj[i][j][2]+(rotation_readings[j]+error_rotation[i][j])/2))
            #print 'the orientation is',traj[i][j][2]
            #print 'Actual movement',error_rotation,error_trans
            error_trans[i][j]=abs(error_trans[i][j])-trans_readings[j]
    return error_rotation,error_trans    


def f(x,*args):
    u, v,z = x
    #temp=0
    #for i in range(counter):
        #args = (param[i],3, 7, 8, 9, 10)
    a, b, c = args
    return sum(np.log(2*np.pi)+np.log((a[0:counter]**2)*u + (b[0:counter]**2)*v + z)+((c[:,0:counter]**2)/(a[0:counter]**2)*u+(b[0:counter]**2)*v+z))*(-0.5)
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
    gu = sum(((a[0:counter]**2)/((a[0:counter]**2)*u+(b[0:counter]**2)*v+z))-((c[:,0:counter]**2)*(a[0:counter]**2)/(((a[0:counter]**2)*u+(b[0:counter]**2)*v+z)**2)))*(-0.5)    # u-component of the gradient
    gv = sum(((b[0:counter]**2)/((a[0:counter]**2)*u+(b[0:counter]**2)*v+z))-((c[:,0:counter]**2)*(b[0:counter]**2)/(((a[0:counter]**2)*u+(b[0:counter]**2)*v+z)**2)))*(-0.5)   # v-component of the gradient
    gz = sum((1/((a[0:counter]**2)*u+(b[0:counter]**2)*v+z))-((c[:,0:counter]**2)/(((a[0:counter]**2)*u+(b[0:counter]**2)*v+z)**2)))*(-0.5)    
    #tmp_gu+=gu
    #tmp_gv+=gv
    #tmp_gz+=gz
    #tmp_gv=tmp_gv*(-1*0.5)
    #tmp_gz=tmp_gz*(-1*0.5)
    #tmp_gu=tmp_gu*(-1*0.5)
    #print type(tmp_gv)
    return np.asarray((gu,gv,gz))



myrobot = robot()

N = 500
p = []
p_original=[]
p_decay_original=[]

iterations=100
world=zeros((world_size,world_size))
distance_reported=np.zeros((iterations))

rotation_reported=np.zeros((iterations))

for i in range(4):
    world[landmarks[i][0],landmarks[i][1]]=1
    #hold(1)    
    #plot(landmarks[i][0],landmarks[i][1],'bo')
    
    #plot(test)
    
    
#show()
#print landmarks[1][1]
#print myrobot.x,myrobot.y
p_history=np.ndarray((500,iterations),dtype=np.object)
p_decay_history=np.ndarray((500,iterations),dtype=np.object)
#show()
world[myrobot.x,myrobot.y]=2
#print  world[landmarks[i][0],landmarks[i][1]]
#print world
#ion()
diff_position=[]
diff_position_original=[]
diff_position_original_decay=[]
sensor_noise=1.0
for i in range(N):
        x = robot()
        #world[x.x,x.y]=3
        #raw_input("Press enter to see the robot move")
        #print world
        x.set_noise(0.05,0.05,sensor_noise,0.05)
        p.append(x)
        p_original.append(x)
        p_decay_original.append(x)
w_mean=[]
counter=0
forw=0.05
turn=0.05
inde=0.05
forw_decay=forw
turn_decay=turn
inde_decay=inde
forw_turn=0.05
forw_turn_decay=forw_turn
turn_turn=0.05
turn_turn_decay=0.05

inde_turn=0.05
inde_turn_decay=0.05
move=5.0
rotate=0.0  #is this in radian or degrees???
change=20
parameter_forw=[]
parameter_forw_decay=[]
parameter_turn=[]
parameter_inde=[]
drift=0.0
trajectory_robot=[]
flag_paramter=0
for t in range(iterations):
    print t
    clf()
    counter=t
    #for i in range(4):
        #hold(1)
        #plot(landmarks[i][0],landmarks[i][1],'bo')
    world[myrobot.x,myrobot.y]=0
    if t<change:
        myrobot = myrobot.move_real(rotate,move,0.0,0.05,0.05,0.05,0.05,0.05,0.05) #turn,forward
    #elif t>=change and t<60:
        #myrobot=myrobot.move_real(rotate,move,0.5,0.05,0.05,0.05,0.05,0.05)
        
    else:
        #print 'I am on a different terrain'
        myrobot=myrobot.move_real(rotate,move,drift,0.5,0.05,0.05,0.05,0.05,0.05) # still have to deal with the move_real theing
    #plot(myrobot.x,myrobot.y,'r^')
    world[myrobot.x,myrobot.y]=2
    #Z_before=myrobot.sense()
    #print world
    
    trajectory_robot.append([myrobot.x,myrobot.y])
    #print p #PLEASE LEAVE THIS HERE FOR GRADING PURPOSES
    #if t>change:
    #    myrobot.sense_noise=7.0
    
    p2 =[]
    p2_original=[]
    p2_decay_original=[]
    if flag_paramter==0:
            
        for i in range(N):
            p2.append(p[i].move_real_distribution(rotate,move,forw,turn,inde,forw_turn,turn_turn,inde_turn))
            p2_original.append(p_original[i].move_real(rotate,move,0.0,0.05,0.05,0.05,0.05,0.05,0.05))
            p2_decay_original.append(p_decay_original[i].move_real_distribution(rotate,move,forw_decay,turn_decay,inde_decay,forw_turn_decay,turn_turn_decay,inde_turn_decay))
            #print p2_decay_original[i]
        
        
        print 'movement done'
        w=[]
        w_original=[]
        w_decay_original=[]
        #p_previous=p
        #p_decay_previous=p
        p=p2
        p_original=p2_original
        p_decay_original=p2_decay_original
        #print 'hello',p_decay_original
        
    
    if t>60:
        flag_paramter=1
    else:
        
        for i in range(N):
            #if t>60:
            #Z=myrobot.sense(20.0)
            #prob_sensor,dist_sensor=p[i].measurement_prob(Z,20.0)
            #flag_paramter=1
            #prob_sensor_original,dist_sensor_original=p_original[i].measurement_prob(Z,20.0)  
        
            flag_paramter=0
            Z=myrobot.sense(sensor_noise)
            #else:
            prob_sensor,dist_sensor=p[i].measurement_prob(Z,sensor_noise)
            prob_sensor_original,dist_sensor_original=p_original[i].measurement_prob(Z,sensor_noise)
            #print i
            #print p_decay_original[i]
            prob_sensor_original_decay,dist_sensor_original_decay=p_decay_original[i].measurement_prob(Z,sensor_noise)
            w.append(prob_sensor)
            #print w
            w_original.append(prob_sensor_original)
            w_decay_original.append(prob_sensor_original_decay)
            p[i].weight=prob_sensor
            p_decay_original[i].weight=prob_sensor_original_decay
            #dist_w.append(dist_sensor)
    #print w
    figure(1)
    #figure(1)
    #print w
    
    
    #resampling step
    if flag_paramter==0:
            
        p3=[]
        p3_original=[]
        p3_decay_original=[]
        index = int(random.random() * N)
        index_original=int(random.random()*N)
        index_decay_original=int(random.random()*N)
        beta = 0.0
        beta_original=0.0
        beta_decay_original=0.0
        mw = max(w)
        mw_original=max(w_original)
        mw_decay_original=max(w_decay_original)
        for i in range(int(N)):
            beta += random.random() * 2.0 * mw
            beta_original +=random.random() * 2.0 * mw_original
            beta_decay_original +=random.random() * 2.0 * mw_decay_original
            while beta > w[index]:
                beta -= w[index]
                index = (index +1) % N
            while beta_original > w_original[index_original]:
                beta_original -= w_original[index_original]
                index_original=(index_original+1)%N
            while beta_decay_original > w_decay_original[index_decay_original]:
                        beta_decay_original -= w_decay_original[index_decay_original]
                        index_decay_original=(index_decay_original+1)%N        
            p3.append(p[index])
            p3_original.append(p_original[index_original])
            p3_decay_original.append(p_decay_original[index_decay_original])
            #print index
        #p=p3
        # you don't need this now because we are telling it that it is kidnapped
    
        p=p3
        p_original=p3_original
   
        
        p_decay_original=p3_decay_original
        for i in range(len(p)):
        
            p_history[i][t]=p[i]
            p_decay_history[i][t]=p_decay_original[i]

        #print p
   
        #print 'the history is',p_history[1][t].weight,p_history[1][t].forward_prob
        print 'smoothing started'
        trajectory=smooth(p_history,t,w,p)
        print 'smoothing finished'
        trajectory_decay=smooth(p_decay_history,t,w_decay_original,p_decay_original)
        #print 'trajectory is',trajectory[0][0][0]
    
        distance_reported[t]=move
        rotation_reported[t]=0.0
        #tran_error_decay=np.zeros((len(trajectory_decay),len(trajectory_decay[0])))
        #print' testing',trajectory
        if len(trajectory[0])>1:
            
            rotation_error,tran_error=error_calculation(trajectory,distance_reported,rotation_reported)
            rotation_error_decay,tran_error_decay=error_calculation(trajectory_decay,distance_reported,rotation_reported)
            #tran_error_decay=tran_error
            #decay fucntion for the tran error#
            lamda=0.000003
            #for i in range(len(tran_error)):
            #print 'tran error before',tran_error
            
            #print 'time is',t
            for i in range(len(tran_error_decay)):
                    
                for j in range(len(tran_error_decay[0])-1):
                    
                    
                        print t-(j+1)
                        tran_error_decay[i][j]=math.copysign((abs((tran_error_decay[i][j]))*np.exp(-1*lamda*(t-(j)))),tran_error_decay[i][j])
                
            #tran_error[0][0]=1
            args = (distance_reported,rotation_reported,tran_error)
            args_decay=(distance_reported,rotation_reported,tran_error_decay)
            #print 'tran error is',tran_error_decay
            #print 'tran error decay is',tran_error_decay
            #print 'the rotation error is',rotation_error
            #print 'just checking',tran_error[0:counter] #when we did this the tran error were huge -> so check that code
            args1 = (distance_reported,rotation_reported,rotation_error)
            x0=np.asarray((0.05,0.05,0.05))
            x1=np.asarray((0.05,0.05,0.05))
            #optimize.mini
            #res1 = optimize.fmin_cg(f, x0,fprime=gradf, args=args,maxiter=50000,full_output=1)
            
                    
            res1 = optimize.fmin_ncg(f, x0,fprime=gradf,args=args,maxiter=1000,full_output=1)
            res2 = optimize.fmin_ncg(f,x1,fprime=gradf,args=args1,maxiter=1000,full_output=1)
            res3=optimize.fmin_ncg(f, x0,fprime=gradf,args=args_decay,maxiter=1000,full_output=1)
            #print 'res1=',res1
            #print 'res3=',res3
            #print res1[0][0]
            forw=res1[0][0]
            turn=res1[0][1]
            inde=res1[0][2]
            forw_decay=res3[0][0]
            turn_decay=res3[0][1]
            inde_decay=res3[0][2]
            forw_turn=res2[0][0]
            forw_turn_decay=res2[0][0]
            turn_turn=res2[0][1]
            turn_turn_decay=res2[0][1]
            inde_turn=res2[0][2]
            inde_turn_decay=res2[0][2]
    
    #print 'The actual location of the robot',myrobot
    print 'computed the paramters'
    parameter_forw.append(forw)
    
    parameter_turn.append(turn)
    parameter_inde.append(inde)
    parameter_forw_decay.append(forw_decay)
    if flag_paramter==0:
            
        particle_location=get_position(p)
        #print parameter_forw
        particle_location_original=get_position(p_original)
        particle_location_original_decay=get_position(p_decay_original)
    else:
        #particle, turn, forward,drift,forward_noise,turn_noise,independent_noise_trans,forward_noise_turn,turn_noise_turn,independent_noise_turn)
        particle_location=move_particle(particle_location,rotate,move,0.0,forw,turn,inde,forw_turn,turn_turn,inde_turn)
        particle_location_original=move_particle(particle_location_original,rotate,move,0.0,0.05,0.05,0.05,0.05,0.05,0.05)
        particle_location_original_decay=move_particle(particle_location_original_decay,rotate,move,0.0,forw_decay,turn_decay,inde_decay,forw_turn_decay,turn_turn_decay,inde_turn_decay)
    #print 'The predicted location',particle_location
    diff_position.append(np.sqrt((min((myrobot.x-particle_location[0]),100-(myrobot.x-particle_location[0]))**2)+(min((myrobot.y-particle_location[1]),100-(myrobot.y-particle_location[1])))**2))
    diff_position_original.append(np.sqrt((min((myrobot.x-particle_location_original[0]),100-(myrobot.x-particle_location_original[0]))**2)+(min((myrobot.y-particle_location_original[1]),100-(myrobot.y-particle_location_original[1])))**2))
    diff_position_original_decay.append(np.sqrt((min((myrobot.x-particle_location_original_decay[0]),100-(myrobot.x-particle_location_original_decay[0]))**2)+(min((myrobot.y-particle_location_original_decay[1]),100-(myrobot.y-particle_location_original_decay[1])))**2))
    #diff_position_original.append(np.sqrt(((myrobot.x-particle_location_original[0])**2)+((myrobot.y-particle_location_original[1])**2)))    
    #plot(particle_location[0],particle_location[1],'r*')
    #raw_input("Press enter to see the robot move")
    #print world
    #print p
    #print "the actual location is"
    
    #hold(1)
    #for i in range(len(trajectory[0])):
    
    #    plot(trajectory[0][i][0],trajectory[0][i][1])
    #show()
    #raw_input("press enter to see the robot move")
    print 'looping back again'
    #show()
    #clf()
#print p
figure(1)
plt.xticks(np.arange(0,101,5.0))
plt.xlabel('Timesteps')
plt.ylabel('Euclidean Error')
p1, =plot(diff_position)
p2, =plot(diff_position_original,'r')
p10, =plot(diff_position_original_decay,'g')
plt.legend([p2,p1],["Static Motion Model","Adaptive Motion Model"])
figure(2)
print 'parameter',parameter_forw
print 'paramter_decay',parameter_forw_decay
p3, =plot(parameter_forw,'r')
p4, =plot(parameter_inde,'g')
p5, =plot(parameter_turn,'y')
p6, =plot(parameter_forw_decay,'b')
plt.legend([p3,p4,p5],["sigma_d_d","sigma_d_1","sigma_d_t"])
#plot(diff_position_original_decay,'g')



    
#p6, =plot(trajectory)
#figure(3)


print trajectory
print trajectory_robot
#print myrobot
#print diff_position
#print diff_position_original
#print p
show()
#raw_input("Press enter to see the robot move")

#print p