import numpy as np

from scipy import optimize
param_translation=np.ones((100))
param_rotation=np.zeros((100))
error_motion=np.zeros((100))
args = (param_translation,param_rotation,error_motion) # parameter values

def f(x,*args):
    u, v,z = x
    temp=0
    for i in range(len(param_translation)):
        #args = (param[i],3, 7, 8, 9, 10)
        a, b, c = args
        temp+=np.log(2*np.pi)+np.log(a[i]*u + b[i]*v + z)+((c[i]**2)/a[i]*u+b[i]*v+z)
    temp=temp*(-1*0.5)    
    return temp

def gradf(x,*args):
    u, v,z = x
    tmp_gu=0
    tmp_gv=0 
    tmp_gz=0
    for i in range(len(param_translation)):
        #args = (param[i],3, 7, 8, 9, 10)
        a, b, c = args
        gu = (a[i]/a[i]*u+b[i]*v+z)-((c[i]**2)*a[i]/((a[i]*u+b[i]*v+z)**2))     # u-component of the gradient
        gv = (b[i]/a[i]*u+b[i]*v+z)-((c[i]**2)*b[i]/((a[i]*u+b[i]*v+z)**2))   # v-component of the gradient
        gz = (1/a[i]*u+b[i]*v+z)-((c[i]**2)/((a[i]*u+b[i]*v+z)**2))        
        tmp_gu+=gu
        tmp_gv+=gv
        tmp_gz+=gz
    tmp_gu=tmp_gu*(-1*0.5)
    tmp_gv=tmp_gv*(-1*0.5)
    tmp_gz=tmp_gz*(-1*0.5)
    #print type(tmp_gv)
    return np.asarray((tmp_gu,tmp_gv,tmp_gz))


x0=np.asarray((0.5,0.5,0.5))

res1 = optimize.fmin_cg(f, x0, fprime=gradf, args=args)
#res1=optimize.minimize(f,x0,args=args,method='CG')
print 'res1=',res1



    
    