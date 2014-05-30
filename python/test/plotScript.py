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

path = 'logs/hrp-2/static-calibration/ds-frontal/simu/2014-05-30/'

fl = np.genfromtxt (path+"HRP2LAAS-forceLLEG.dat")
fr = np.genfromtxt (path+"HRP2LAAS-forceRLEG.dat")
fr = fr.copy()
fr.resize(fl.shape[0],fr.shape[1])
f= (fl + fr)/2




flex = np.genfromtxt (path+ "com-stabilizedEstimator-flexInversePoseThetaU.dat")
flexDot = np.genfromtxt (path+"com-stabilizedEstimator-flexInverseOmega.dat")
acc = np.genfromtxt (path+"HRP2LAAS-accelerometer.dat")
#acc_c=center(acc,acc[acc.shape[0]-1,1:acc.shape[1]])
#iacc_c=integrate(acc_c)
#iiacc_c = integrate(integrate(iacc_c))

gyr = np.genfromtxt (path+"HRP2LAAS-gyrometer.dat")
#igyr = integrate(gyr[:,:])

realCom = np.genfromtxt (path+"real-com-sout.dat")

#sns = np.genfromtxt (path+"flextimator-simulatedSensors.dat")

#er = np.genfromtxt (path+"hMhrefVector-sout.dat")

#chest = np.genfromtxt (path+"robot_dynamic-chest.dat")

#inputSignal = np.genfromtxt(path+"flextimator-input.dat")

#inputVel = np.genfromtxt(path+"inputVelocity-sout.dat")

#sotcontrol = np.genfromtxt(path+"solver-control.dat")

#devicestate = np.genfromtxt(path+"HRP2LAAS-state.dat")


#stabilizerRef = np.genfromtxt (path+"com-stabilized-comRef.dat")
stabilizer = np.genfromtxt (path+"com-stabilized-task.dat")
stabilizerError = np.genfromtxt (path+"com-stabilized-error.dat")
stabilizerDot = np.genfromtxt (path+"com-stabilized-d2com.dat")

flexThetaU = np.genfromtxt (path+"com-stabilizedEstimator-flexThetaU.dat")


#comRef = np.genfromtxt (path+"comref-sout.dat")

#er[:,1:er.shape[1]]=er[:,1:er.shape[1]]-er2[:,1:er.shape[1]]    

f1 = plt.figure();ax1 = f1.add_subplot(111)
f2 = plt.figure();ax2 = f2.add_subplot(111)
f3 = plt.figure();ax3 = f3.add_subplot(111)
#f4 = plt.figure();ax4 = f4.add_subplot(111)
#f5 = plt.figure();ax5 = f5.add_subplot(111)
#f6 = plt.figure();ax6 = f6.add_subplot(111)
#f7 = plt.figure();ax7 = f7.add_subplot(111)
#f8 = plt.figure();ax8 = f8.add_subplot(111)
#f9 = plt.figure();ax9 = f9.add_subplot(111)
#f10 = plt.figure();ax10=f10.add_subplot(111)


ax1.plot(f[:,0], f[:,1], label='Feet Force X')
#ax1.plot(f[:,0], f[:,2], label='Feet Force Y')
#ax1.plot(f[:,0], f[:,3], label='Feet Force Z')
#ax1.plot(f[:,0], f[:,4], label='Feet Force TX')
ax1.plot(f[:,0], f[:,5], label='Feet Force TY')
#ax1.plot(f[:,0], f[:,6], label='Feet Force TZ')

ax2.plot(realCom[:,0],realCom[:,1], label='CoM x')
ax2.plot(realCom[:,0],realCom[:,2], label='CoM y')
ax2.plot(realCom[:,0],realCom[:,3], label='CoM z')



#ax1.plot(flex[:,0], flex[:,5]*1000, label='Flexibility')
#ax1.plot(flexDot[:,0], flexDot[:,2]*1000, label='FlexiDot')
#ax1.plot(stabilizer[:,0], stabilizer[:,1]*1000, label='stabilizer')
#ax1.plot(stabilizerDot[:,0], stabilizerDot[:,1]*1, label='stabilizerDot')
#ax1.plot(stabilizerRef[:,0], stabilizerRef[:,2]*1000, label='stabilizerRefY')


ax3.plot(flexThetaU[:,0], flexThetaU[:,1]*1000, label='flexThetaU X')
ax3.plot(flexThetaU[:,0], flexThetaU[:,2]*1000, label='flexThetaU Y')
ax3.plot(flexThetaU[:,0], flexThetaU[:,3]*1000, label='flexThetaU Z')


#ax2.plot(comRef[:,1:3])

#ax2.plot(acc[:,0], acc[:,1]*100, label='Acceleromter0')
#ax2.plot(sns[:,0], sns[:,1]*99.99, label='SimulatedAcc')

#ax3.plot(gyr[:,0], 100*gyr[:,2], label='Gyrometer')
#ax3.plot(sns[:,0], 100*sns[:,5], label='SimulatedGyr')


#chestplot = ax4.plot(chest[:,0],chest[:,1:4])

#inputplot = ax5.plot(inputSignal[:,0],inputSignal[:,1:7])

#inputVel = ax6.plot(inputSignal[:,0],inputSignal[:,7:13])

#inputAcc = ax7.plot(inputSignal[:,0],inputSignal[:,13:19])
#control2 = ax6.plot(sotcontrol[:,0],sotcontrol[:,20:22])

#state = ax7.plot(devicestate[:,0],devicestate[:,1:7])
#state = ax7.plot(devicestate[:,0],devicestate[:,20:22])


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


#refvelplot = ax8.plot(refVel[:,0],refVel[:,4], label='RefVelrotx')
#refvelplot = ax8.plot(refVel[:,0],refVel[:,5], label='RefVelroty')
#refvelplot = ax8.plot(refVel[:,0],refVel[:,6], label='RefVelrotz')

#stateerrplot = ax9.plot(state[:,0],state[:,6:37]-robotstate[:,6:37])



#ploter21 = ax10.plot(er[:,0],er[:,1], label='Hand error2 x pos')
#ploter22 = ax10.plot(er[:,0],er[:,2], label='Hand error2 y pos')
#ploter23 = ax10.plot(er[:,0],er[:,3], label='Hand error2 z pos')
#ploter24 = ax10.plot(er[:,0],er[:,4], label='Hand error2 x rot')
#ploter25 = ax10.plot(er[:,0],er[:,5], label='Hand error2 y rot')
#ploter26 = ax10.plot(er[:,0],er[:,6], label='Hand error2 z rot')


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

#handles, labels = ax10.get_legend_handles_labels()
#ax10.legend(handles, labels)

plt.show()
