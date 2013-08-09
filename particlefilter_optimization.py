import numpy as np

from scipy import optimize
param=np.arange(1,10)
args = (param,3, 7, 8, 9, 10)  # parameter values

def f(x,*args):
    u, v = x
    temp=0
    for i in range(len(param)):
        #args = (param[i],3, 7, 8, 9, 10)
        a, b, c, d, e, f = args
        temp+=a[i]*u**2 + b*u*v + c*v**2 + d*u + e*v + f
    return temp

def gradf(x,*args):
    u, v = x
    tmp_gu=0
    tmp_gv=0    
    for i in range(len(param)):
        #args = (param[i],3, 7, 8, 9, 10)
        a, b, c, d, e, f = args
        gu = 2*a[i]*u + b*v + d     # u-component of the gradient
        gv = b*u + 2*c*v + e  
        tmp_gu+=gu
        tmp_gv+=gv
    return np.asarray((gu,gv))


x0=np.asarray((0,0))

res1 = optimize.fmin_cg(f, x0, fprime=gradf, args=args)

print 'res1=',res1



    
    