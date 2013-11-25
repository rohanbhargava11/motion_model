import numpy as np
from pylab import *

test=np.load('/home/rohan/Documents/motion_model/200_0.05_0.05_s_1.0_traj_3_drift_1.npz')
test.files
plt.xticks(np.arange(0,200,10.0))
plt.xlabel('Timesteps')
plt.ylabel('Euclidean Error')
    
    
p1, =plot(test['diff_position_plot'])
p2, =plot(test['diff_position_original_plot'],'r')

plt.legend([p2,p1],["Static Motion Model","Adaptive Motion Model"])

figure(2)
parameter_forw_plot=test['parameter_forw_plot']
parameter_inde_plot=test['parameter_inde_plot']
#print parameter_forw_plot.shape
np.putmask(parameter_forw_plot,parameter_forw_plot>=100,0.05)
print parameter_forw_plot[199]
print parameter_inde_plot[199]
np.putmask(parameter_inde_plot,parameter_inde_plot>=100,0.05)
plt.xlabel('Timesteps')
plt.ylabel('Value')
plt.xticks(np.arange(0,201,10.0))
p3, =plot(parameter_forw_plot,'r')
p4, =plot(parameter_inde_plot,'g')
p5, =plot(test['parameter_turn_plot'],'b')
plt.legend([p3,p4,p5],['sigma_d_d','sigma_d_1','sigma_d_t'])
figure(3)


plt.xticks(np.arange(0,201,10.0))
plt.yticks(np.arange(min(test['average_weight_plot']),max(test['average_weight_plot'])))
plt.xlabel('Timesteps')
plt.ylabel('Value')
p6, =plot(test['average_weight_plot'],'b')
plt.legend([p6],['weight of particles'])
show()
#plot(testdiff_position_original_decay,'g')