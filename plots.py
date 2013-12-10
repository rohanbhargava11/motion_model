import numpy as np
from pylab import *
dataset='200_0.05_0.5_s_1.0_10.0_100_traj_3_trial_1'

print dataset
test=np.load('/home/rohan/Documents/motion_model/final_datasets/'+dataset+'.npz')
test.files
dataset=dataset.replace('.',"")
#print test['diff_position_forward_plot']
plt.xticks(np.arange(0,200,10.0))
plt.yticks(np.arange(0,400,10.0))
plt.xlabel('Timesteps')
plt.ylabel('Euclidean Error')
    
    
p1, =plot(test['diff_position_plot'])
p2, =plot(test['diff_position_original_plot'],'r')
#p3, =plot(test['diff_position_forward_plot'],'y')
print test['total_time']
plt.legend([p2,p1],["Static Motion Model","Adaptive Motion Model"])
plt.savefig('/home/rohan/Documents/thesis_work/thesis/thesis_writeup/plots/'+dataset+'.pdf')

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
plt.savefig('/home/rohan/Documents/thesis_work/thesis/thesis_writeup/plots/'+dataset+'_motion_model_trans.pdf')
figure(3)
plt.xlabel('Timesteps')
plt.ylabel('Value')
plt.xticks(np.arange(0,201,10.0))
parameter_forw_turn_plot=test['parameter_forw_turn_plot']
np.putmask(parameter_forw_turn_plot,parameter_forw_turn_plot>=100,0.05)
parameter_turn_turn_plot=test['parameter_turn_turn_plot']
parameter_inde_turn_plot=test['parameter_inde_turn_plot']
print 'rotation'
print parameter_forw_turn_plot[199]
print parameter_inde_turn_plot[199]
np.putmask(parameter_inde_turn_plot,parameter_inde_turn_plot>=100,0.05)
p7, =plot(parameter_forw_turn_plot,'r')
p8, =plot(parameter_inde_turn_plot,'g')
p9, =plot(test['parameter_turn_turn_plot'],'b')
plt.legend([p7,p8,p9],['sigma_t_d','sigma_t_1','sigma_t_t'])
plt.savefig('/home/rohan/Documents/thesis_work/thesis/thesis_writeup/plots/'+dataset+'_motion_model_rotation.pdf')

figure(4)


plt.xticks(np.arange(0,201,10.0))
#plt.yticks(np.arange(min(test['average_weight_plot']),max(test['average_weight_plot'])))
plt.xlabel('Timesteps')
plt.ylabel('Value')
#p6, =plot(test['average_weight_plot'],'b')
p7, =plot(test['average_weight_original_plot'],'r')
plt.legend([p7],['weight of particles'])
plt.savefig('/home/rohan/Documents/thesis_work/thesis/thesis_writeup/plots/'+dataset+'_average_weight.pdf')
figure(5)
plot(test['above_average_original_plot'],'r')
plot(test['above_average_plot'],'b')
plt.savefig('/home/rohan/Documents/thesis_work/thesis/thesis_writeup/plots/'+dataset+'_max_weight.pdf')
figure(6)
#k=test['above_average_original_plot']
k=test['weight_smooth_history_average_plot']
p=k[101:]
o=test['above_average_plot']
b=o[101:]
plot(b,'b')
plot(p,'r')
figure(7)
plot(test['weight_smooth_history_average_plot'])
show()
#plot(testdiff_position_original_decay,'g')