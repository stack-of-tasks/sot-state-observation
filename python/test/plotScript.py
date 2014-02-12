import numpy as np
from numpy import linalg as LA
import matplotlib.pyplot as plt

def integrate(v):
    vi = np.array([])
    #m = np.mean(:,1:v.shape[1]], axis=0)
    #m=v[v.shape[0]-1,1:v.shape[1]]
    vi.resize([v.shape[0],v.shape[1]])
    x=np.array(v[0,1:v.shape[1]]);
    for i in range(1,v.shape[0]):
        vi[i,0]=v[i,0]
        vi[i,1:v.shape[1]]=x
        x=x+v[i,1:v.shape[1]]
    return vi

def derivate(v):
    vi = np.array([])
    #m = np.mean(:,1:v.shape[1]], axis=0)
    #m=v[v.shape[0]-1,1:v.shape[1]]
    vi.resize([v.shape[0]-1,v.shape[1]])
    for i in range(0,vi.shape[0]):
        vi[i,0]=v[i,0]
        vi[i,1:v.shape[1]]=v[i+1,1:v.shape[1]]-v[i,1:v.shape[1]]
    return vi

def norm(v):
    vi = np.array([])
    #m = np.mean(:,1:v.shape[1]], axis=0)
    #m=v[v.shape[0]-1,1:v.shape[1]]
    vi.resize([v.shape[0],1])
    for i in range(0,vi.shape[0]):
        vi[i]=LA.norm(v[i,:])
    return vi

def center(v,m):
    vi=v;
    for i in range(1,v.shape[0]):
        vi[i,1:v.shape[1]]=v[i,1:v.shape[1]]-m
    return vi

fl = np.genfromtxt ("/tmp/HRP2LAAS-forceLLEG.dat")
fr = np.genfromtxt ("/tmp/HRP2LAAS-forceRLEG.dat")
fr = fr.copy()
fr.resize(fl.shape[0],fr.shape[1])
f= fl + fr


flex = np.genfromtxt ("/tmp/flextimator-flexibility.dat")

acc = np.genfromtxt ("/tmp/HRP2LAAS-accelerometer.dat")
#acc_c=center(acc,acc[acc.shape[0]-1,1:acc.shape[1]])
#iacc_c=integrate(acc_c)
#iiacc_c = integrate(integrate(iacc_c))

gyr = np.genfromtxt ("/tmp/HRP2LAAS-gyrometer.dat")
#igyr = integrate(gyr[:,:])



sns = np.genfromtxt ("/tmp/flextimator-simulatedSensors.dat")

er = np.genfromtxt ("/tmp/hMhrefVector-sout.dat")
er2= np.genfromtxt ("/tmp/taskcompensateR-error.dat")

chest = np.genfromtxt ("/tmp/robot_dynamic-chest.dat")

pred = np.genfromtxt ("/tmp/flextimator-predictedSensors.dat")

refPos = np.genfromtxt ("/tmp/featurecompensateR_ref-position.dat")
refVel = np.genfromtxt ("/tmp/featurecompensateR_ref-velocity.dat")

state = np.genfromtxt ("/tmp/HRP2LAAS-state.dat")
robotstate = np.genfromtxt ("/tmp/HRP2LAAS-robotState.dat")


#er[:,1:er.shape[1]]=er[:,1:er.shape[1]]-er2[:,1:er.shape[1]]    

f1 = plt.figure()
f2 = plt.figure()
f3 = plt.figure()
f4 = plt.figure()
f5 = plt.figure()
f6 = plt.figure()
f7 = plt.figure()
f8 = plt.figure()
f9 = plt.figure()
f10 = plt.figure()

ax1 = f1.add_subplot(111)
ax2 = f2.add_subplot(111)
ax3 = f3.add_subplot(111)
ax4 = f4.add_subplot(111)
ax5 = f5.add_subplot(111)
ax6 = f6.add_subplot(111)
ax7 = f7.add_subplot(111)
ax8 = f8.add_subplot(111)
ax9 = f9.add_subplot(111)
ax10=f10.add_subplot(111)

forceplot = ax1.plot(f[:,0]/2, f[:,5], label='Feet Force')
flexplot  = ax1.plot(flex[:,0], -flex[:,4]*1000, label='Flexibility0')
flexplot  = ax1.plot(flex[:,0], -flex[:,5]*1000, label='Flexibility1')
flexplot  = ax1.plot(flex[:,0], -flex[:,6]*1000, label='Flexibility2')
accplot   = ax1.plot(acc[:,0], acc[:,1]*100, label='Acceleromter0')
simplot   = ax1.plot(sns[:,0], sns[:,1]*99.99, label='SimulatedAcc')

gyrplot = ax2.plot(gyr[:,0], 100*gyr[:,2], label='Gyrometer')
simplot = ax2.plot(sns[:,0], 100*sns[:,5], label='SimulatedGyr')


ploter1 = ax3.plot(er[:,0],er[:,1], label='Hand error x pos')
ploter2 = ax3.plot(er[:,0],er[:,2], label='Hand error y pos')
ploter3 = ax3.plot(er[:,0],er[:,3], label='Hand error z pos')
ploter4 = ax3.plot(er[:,0],er[:,4], label='Hand error x rot')
ploter5 = ax3.plot(er[:,0],er[:,5], label='Hand error y rot')
ploter6 = ax3.plot(er[:,0],er[:,6], label='Hand error z rot')

chestplot = ax4.plot(chest[:,0],chest[:,1:4])

#prederrplot = ax5.plot(pred[:,0],pred[:,1]-acc[:,1], label='Prediction error acc 0')
#prederrplot = ax5.plot(pred[:,0],pred[:,2]-acc[:,2], label='Prediction error acc 1')
#prederrplot = ax5.plot(pred[:,0],pred[:,3]-acc[:,3], label='Prediction error acc 2')
#prederrplot = ax6.plot(pred[:,0],pred[:,4]-gyr[:,1], label='Prediction error gyr 0')
#prederrplot = ax6.plot(pred[:,0],pred[:,5]-gyr[:,2], label='Prediction error gyr 1')
#prederrplot = ax6.plot(pred[:,0],pred[:,6]-gyr[:,3], label='Prediction error gyr 2')
##prederrplot = ax5.plot(pred[:,0],norm(pred[:,1:4]-acc[:,1:4]), label='Prediction error acc')
##prederrplot = ax5.plot(pred[:,0],norm(pred[:,4:7]-gyr[:,1:4]), label='Prediction error gyr')
#prederrplot = ax7.plot(pred[:,0],norm(pred[:, 7:10]), label='Prediction error foot1')
#prederrplot = ax7.plot(pred[:,0],norm(pred[:,10:13]), label='Prediction error foot2')


refvelplot = ax8.plot(refPos[:,0],refPos[:,4], label='RefPosx')
refvelplot = ax8.plot(refPos[:,0],refPos[:,8], label='RefPosy')
refvelplot = ax8.plot(refPos[:,0],refPos[:,12], label='RefPosz')

refvelplot = ax8.plot(refVel[:,0],refVel[:,1], label='RefVelx')
refvelplot = ax8.plot(refVel[:,0],refVel[:,2], label='RefVely')
refvelplot = ax8.plot(refVel[:,0],refVel[:,3], label='RefVelz')
#refvelplot = ax8.plot(refVel[:,0],refVel[:,4], label='RefVelrotx')
#refvelplot = ax8.plot(refVel[:,0],refVel[:,5], label='RefVelroty')
#refvelplot = ax8.plot(refVel[:,0],refVel[:,6], label='RefVelrotz')

#stateerrplot = ax9.plot(state[:,0],state[:,6:37]-robotstate[:,6:37])

ploter21 = ax10.plot(er[:,0],er[:,1], label='Hand error2 x pos')
ploter22 = ax10.plot(er[:,0],er[:,2], label='Hand error2 y pos')
ploter23 = ax10.plot(er[:,0],er[:,3], label='Hand error2 z pos')
ploter24 = ax10.plot(er[:,0],er[:,4], label='Hand error2 x rot')
ploter25 = ax10.plot(er[:,0],er[:,5], label='Hand error2 y rot')
ploter26 = ax10.plot(er[:,0],er[:,6], label='Hand error2 z rot')


handles, labels = ax1.get_legend_handles_labels()
ax1.legend(handles, labels)

handles, labels = ax2.get_legend_handles_labels()
ax2.legend(handles, labels)

handles, labels = ax3.get_legend_handles_labels()
ax3.legend(handles, labels)

#handles, labels = ax4.get_legend_handles_labels()
#ax4.legend(handles, labels)

#handles, labels = ax5.get_legend_handles_labels()
#ax5.legend(handles, labels)

#handles, labels = ax6.get_legend_handles_labels()
#ax6.legend(handles, labels)

#handles, labels = ax7.get_legend_handles_labels()
#ax7.legend(handles, labels)

handles, labels = ax8.get_legend_handles_labels()
ax8.legend(handles, labels)

handles, labels = ax10.get_legend_handles_labels()
ax10.legend(handles, labels)

plt.show()
