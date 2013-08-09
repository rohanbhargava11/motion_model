import numpy as np

from scipy import optimize

args = (distance[i],rotation[i],error[i],2*np.pi)
def f(x,*args):
    
    u, v, z = x
    
    for i in range(len(time):
      a, b, c,d = args
      return -0.5*np.log(a*u + b*v + z)+(c**2/a*u+b*v+z) 
  
  
def gradf(x,*args):
    u, v, z = x
    a, b, c, d = args
    gu = -0.5*(a/a*u+b*v+z)-(c**2*a/(a*u+b*v+z)**2)     # u-component of the gradient
    gv = -0.5*(b/a*u+b*v+z)-(c**2*b/(a*u+b*v+z)**2)   # v-component of the gradient
    gz = -0.5*(1/a*u+b*v+z)-(c**2/(a*u+b*v+z)**2)
    return np.asarray((gu,gv,gz))


x0=np.asarray((0.5,0.5,0.5))
rotation=(np.pi/2,np.pi/2)
error=(1,1)
distance=(2,5)
#answer_temp=0
for i in range(len(rotation)):

res1=optimize.fmin_cg(f,x0,fprime=gradf,args=args,maxiter=1000)
    
print type(res1)
    
print res1