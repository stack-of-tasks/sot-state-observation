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

path = 'logs/hrp-2/static-calibration/ds-frontal/robot/2015-05-28/'
path = '/home/mbenalle/devel/ros/src/sot-stabilizer/logs/hrp-2/hand-compensator-oscillator/robot/2015-07-07/'

path = '/home/mbenalle/devel/ros/src/sot-stabilizer/logs//hrp-2/stabilizer/simu/mixed-stabilizers/stand-on-left-foot/2014-08-20/'
path = '/home/mbenalle/devel/ros/src/sot-stabilizer/logs//hrp-2/stabilizer/simu/stock-stabilizer/stand-on-left-foot/2014-08-20/'
path = '/home/mbenalle/devel/ros/src/sot-stabilizer/logs//hrp-2/stabilizer/simu/custom-stabilizer/stand-on-left-foot/2014-08-20'
path = '/home/mbenalle/devel/ros/src/sot-stabilizer/logs//hrp-2/stabilizer/simu/no-stabilizer/stand-on-left-foot/2014-08-20'


path = '/home/mbenalle/devel/ros/src/sot-stabilizer/logs//hrp-2/stabilizer/simu/mixed-stabilizers/walkfwd-shifted/2014-08-20'
#path = '/home/mbenalle/devel/ros/src/sot-stabilizer/logs//hrp-2/stabilizer/simu/stock-stabilizer/walkfwd-shifted/2014-08-20'
#path = '/home/mbenalle/devel/ros/src/sot-stabilizer/logs//hrp-2/stabilizer/simu/no-stabilizer/walkfwd-shifted/2014-08-20'
#path = '/home/mbenalle/devel/ros/src/sot-stabilizer/logs//hrp-2/stabilizer/simu/custom-stabilizer/walkfwd-shifted/2014-08-20'



#path = 'logs/hrp-2/static-calibration/ss-frontal/simu/2014-05-30/'
path = '/tmp/'

path=path+'/'

comheight = 0.804691781499
gravity = 9.81
dt = 0.005

on=True;
off=False;
only=True

seqplay = off
seqplayForce = off
seqplayZMP = off
zmpPlot =on
zmpfromcom =off
zmpfromflex =off
zmpfromEstimatedflex =on
comVelPlot= off
comAccPlot= on
comPlot = on
flexPlot=on
forcesPlot=on
compareforcesPlot=on
imuPlot=off
stabilizerDebug=on


if zmpPlot:
    zmpReal  = np.genfromtxt (path+"zmp-zmp.dat")
    fzmp = plt.figure(); axzmp = fzmp.add_subplot(111)
    axzmp.plot(zmpReal[:,0], zmpReal[:,1], label='zmp X')
    axzmp.plot(zmpReal[:,0], zmpReal[:,2], label='zmp Y')

    if seqplayForce:
        zmpRef = np.genfromtxt (path+"zmpRef-zmp.dat")
        axzmp.plot(zmpRef [:,0], zmpRef [:,1], label='zmpRef X')
        axzmp.plot(zmpRef [:,0], zmpRef [:,2], label='zmpRef Y')
    if seqplayZMP:
        zmpSeq = np.genfromtxt (path+"seqplay-zmp.dat")
        axzmp.plot(zmpSeq [:,0], zmpSeq [:,1], label='zmpSeq X')
        axzmp.plot(zmpSeq [:,0], zmpSeq [:,2], label='zmpSeq Y')
    if seqplay:
        lankleVelref = np.genfromtxt (path+"seqplay-leftAnkleVel.dat")
        rankleVelref = np.genfromtxt (path+"seqplay-rightAnkleVel.dat")
        #axzmp.plot(lankleVelref[:,0], lankleVelref[:,1], label='LeftAnkleVel')
        #axzmp.plot(rankleVelref[:,0], rankleVelref[:,2], label='RightAnkleVel')
    else:
        lanklePos = np.genfromtxt (path+"robot_feature_left-ankle-position.dat")
        ranklePos = np.genfromtxt (path+"robot_feature_right-ankle-position.dat")
        #axzmp.plot(lanklePos[:,0], lanklePos[:,4], label='LeftAnkle')
        #axzmp.plot(ranklePos[:,0], ranklePos[:,4], label='RightAnkle')
    
    if zmpfromcom:
        comSeq = np.genfromtxt (path+"seqplay-com.dat")
        comddotSeq = np.genfromtxt (path+"seqplay-comddot.dat")
        #axzmp.plot(comRec[:,0], comRec[:,1], label='comX')
        #axzmp.plot(comRec[:,0], comRec[:,2], label='comy')
        axzmp.plot(comSeq[:,0], comSeq[:,1]-comddotSeq[:,1]*comheight/gravity, label='zmpfromcomx')
        axzmp.plot(comSeq[:,0], comSeq[:,2]-comddotSeq[:,2]*comheight/gravity, label='zmpfromcomy')

    if zmpfromflex:
        comReal = np.genfromtxt (path+"comreal-gP0.dat")
        comddotReal = np.genfromtxt (path+"comreal-gA0.dat")
        axzmp.plot(comReal[:,0], comReal[:,1]-comddotReal[:,1]*comheight/gravity, label='zmpfromflexx')
        axzmp.plot(comReal[:,0], comReal[:,2]-comddotReal[:,2]*comheight/gravity, label='zmpfromflexy')

    if zmpfromEstimatedflex:
        zmpSeq = np.genfromtxt (path+"zmpestimated-zmp.dat")
        axzmp.plot(zmpSeq [:,0], zmpSeq [:,1], label='zmpEst X')
        axzmp.plot(zmpSeq [:,0], zmpSeq [:,2], label='zmpEst Y')
    
    handles, labels = axzmp.get_legend_handles_labels()
    axzmp.legend(handles, labels)

if comVelPlot:
    comVel = np.genfromtxt (path+"com-stabilized-task.dat")
    comVelRef = np.genfromtxt (path+"seqplay-comdot.dat")
    fcomvel = plt.figure(); axcomvel = fcomvel.add_subplot(111)
    axcomvel.plot(comVel[:,0], comVel[:,1], label='comvel-x')
    axcomvel.plot(comVel[:,0], comVel[:,2], label='comvel-y')
    axcomvel.plot(comVelRef[:,0], comVelRef[:,1], label='comvelref-x')
    axcomvel.plot(comVelRef[:,0], comVelRef[:,2], label='comvelref-y')
    
    handles, labels = axcomvel.get_legend_handles_labels()
    axcomvel.legend(handles, labels)

if comAccPlot:
    comAcc = np.genfromtxt (path+"seqplay-comddot.dat")
    comAccRef = np.genfromtxt (path+"com-stabilized-comddotRefOUT.dat")
    comAccError = np.genfromtxt (path+"com-stabilized-debug.dat")

    fcomAccx = plt.figure(); axcomAccx = fcomAccx.add_subplot(111)
    fcomAccy = plt.figure(); axcomAccy = fcomAccy.add_subplot(111)

    axcomAccx.plot(comAcc[:,0], comAcc[:,1], label='comRecacc-x')
    axcomAccy.plot(comAcc[:,0], comAcc[:,2], label='comRecacc-y')

    axcomAccx.plot(comAccRef[:,0], comAccRef[:,1], label='comrefacc-x')
    axcomAccy.plot(comAccRef[:,0], comAccRef[:,2], label='comrefacc-y')

    axcomAccx.plot(comAccError[:,0], comAccError[:,5]-comAccError[:,6]+comAccRef[:,1], label='comerroracc-x')
    axcomAccy.plot(comAccError[:,0], comAccError[:,12]-comAccError[:,13]+comAccRef[:,2], label='comerroracc-y')
    
    handles, labels = axcomAccx.get_legend_handles_labels()
    axcomAccx.legend(handles, labels)
    handles, labels = axcomAccy.get_legend_handles_labels()
    axcomAccy.legend(handles, labels)

if comPlot:
    com = np.genfromtxt (path+"com-stabilized-com.dat")
    comRef = np.genfromtxt (path+"seqplay-com.dat")
    comReal = np.genfromtxt (path+"comreal-gP0.dat")
    fcom = plt.figure(); axcom = fcom.add_subplot(111)
    axcom.plot(com[:,0], com[:,1], label='com-x')
    axcom.plot(com[:,0], com[:,2], label='com-y')
    axcom.plot(comRef[:,0], comRef[:,1], label='comref-x')
    axcom.plot(comRef[:,0], comRef[:,2], label='comref-y')

    axcom.plot(comReal[:,0], comReal[:,1], label='comreal-x')
    axcom.plot(comReal[:,0], comReal[:,2], label='comreal-y')
    
    handles, labels = axcom.get_legend_handles_labels()
    axcom.legend(handles, labels)

if flexPlot:
    #flex = np.genfromtxt (path+"flextimator-flexibility.dat")
    flex = np.genfromtxt (path+"com-stabilizedEstimator-flexibility.dat")
    
    fflex = plt.figure(); axflex = fflex.add_subplot(111)
    axflex.plot(flex[:,0], flex[:,4], label='flexibility-x')
    axflex.plot(flex[:,0], flex[:,5], label='flexibility-y')
    
    handles, labels = axflex.get_legend_handles_labels()
    axflex.legend(handles, labels)

if forcesPlot:
    forcesl = np.genfromtxt (path+"HRP2LAAS-forceLLEG.dat")
    
    fforces = plt.figure(); axforces = fforces.add_subplot(111)
    axforces.plot(forcesl[:,0], forcesl[:,1], label='force-l-x')
    axforces.plot(forcesl[:,0], forcesl[:,2], label='force-l-y')

    forcesr = np.genfromtxt (path+"HRP2LAAS-forceRLEG.dat")
    
    axforces.plot(forcesr[:,0], forcesr[:,1], label='force-r-x')
    axforces.plot(forcesr[:,0], forcesr[:,2], label='force-r-y')
    
    handles, labels = axforces.get_legend_handles_labels()
    axforces.legend(handles, labels)

if compareforcesPlot:
    forcesmeasured = np.genfromtxt (path+"com-stabilized-measuredForceTorque.dat")
    forcessimulated = np.genfromtxt (path+"com-stabilizedEstimator-forcesAndMoments.dat")
    
    fforcescompare1 = plt.figure(); axforcescompare1 = fforcescompare1.add_subplot(111)
    axforcescompare1.plot(forcesmeasured[:,0], forcesmeasured[:,1:4], label='forces1-meas')
    axforcescompare1.plot(forcessimulated[:,0], forcessimulated[:,1:4], label='force1-est')

    fforcescompare2 = plt.figure(); axforcescompare2 = fforcescompare2.add_subplot(111)
    axforcescompare2.plot(forcesmeasured[:,0], forcesmeasured[:,7:10], label='forces2-meas')
    axforcescompare2.plot(forcessimulated[:,0], forcessimulated[:,7:10], label='force2-est')
    
    ftorquescompare1 = plt.figure(); axtorquescompare1 = ftorquescompare1.add_subplot(111)
    axtorquescompare1.plot(forcesmeasured[:,0], forcesmeasured[:,4:7], label='torques1-meas')
    axtorquescompare1.plot(forcessimulated[:,0], forcessimulated[:,4:7], label='torques1-est')

    ftorquescompare2 = plt.figure(); axtorquescompare2 = ftorquescompare2.add_subplot(111)
    axtorquescompare2.plot(forcesmeasured[:,0], forcesmeasured[:,10:13], label='torques2-meas')
    axtorquescompare2.plot(forcessimulated[:,0], forcessimulated[:,10:13], label='torques2-est')
    
    handles, labels = axforcescompare1.get_legend_handles_labels()
    axforcescompare1.legend(handles, labels)
    handles, labels = axforcescompare2.get_legend_handles_labels()
    axforcescompare2.legend(handles, labels)
    handles, labels = axtorquescompare1.get_legend_handles_labels()
    axtorquescompare1.legend(handles, labels)
    handles, labels = axtorquescompare2.get_legend_handles_labels()
    axtorquescompare2.legend(handles, labels)

if imuPlot:

    acc = np.genfromtxt (path+"HRP2LAAS-accelerometer.dat")
    gyr = np.genfromtxt (path+"HRP2LAAS-gyrometer.dat")
    fimu = plt.figure(); aximu = fimu.add_subplot(111)

    aximu.plot(acc[:,0], acc[:,1]*0.1, label='acc-x')

    aximu.plot(gyr[:,0], gyr[:,2], label='gyr-y')

    
    handles, labels = aximu.get_legend_handles_labels()
    aximu.legend(handles, labels)

if stabilizerDebug:
    stab = np.genfromtxt (path+"com-stabilized-debug.dat")
    fstabx = plt.figure(); axstabx = fstabx.add_subplot(111)

    axstabx.plot(stab[:,0], stab[:,5], label='xddot')    
    axstabx.plot(stab[:,0], stab[:,7], label='xddot cmd')
    axstabx.plot(stab[:,0], stab[:,3], label='xdot')
    axstabx.plot(stab[:,0], stab[:,1], label='x')    
    axstabx.plot(stab[:,0], stab[:,2], label='flex-x')    


    handles, labels = axstabx.get_legend_handles_labels()
    axstabx.legend(handles, labels)

    fstaby = plt.figure(); axstaby = fstaby.add_subplot(111)

    axstaby.plot(stab[:,0], stab[:,12], label='yddot')    
    axstaby.plot(stab[:,0], stab[:,14], label='yddot cmd')
    axstaby.plot(stab[:,0], stab[:,10], label='ydot')
    axstaby.plot(stab[:,0], stab[:,8], label='y')    
    axstaby.plot(stab[:,0], stab[:,9], label='flex-y')    


    handles, labels = axstaby.get_legend_handles_labels()
    axstaby.legend(handles, labels)
    




fl = np.genfromtxt (path+"HRP2LAAS-forceLLEG.dat")
fr = np.genfromtxt (path+"HRP2LAAS-forceRLEG.dat")
fr = fr.copy()
fr.resize(fl.shape[0],fr.shape[1])
f= (fl + fr)/2




#flex = np.genfromtxt (path+ "com-stabilizedEstimator-flexInversePoseThetaU.dat")
#flexDot = np.genfromtxt (path+"com-stabilizedEstimator-flexInverseOmega.dat")
#acc = np.genfromtxt (path+"HRP2LAAS-accelerometer.dat")
#acc_c=center(acc,acc[acc.shape[0]-1,1:acc.shape[1]])
#iacc_c=integrate(acc_c)
#iiacc_c = integrate(integrate(iacc_c))

#gyr = np.genfromtxt (path+"HRP2LAAS-gyrometer.dat")
#igyr = integrate(gyr[:,:])

#realCom = np.genfromtxt (path+"real-com-sout.dat")

#sns = np.genfromtxt (path+"flextimator-simulatedSensors.dat")

#er = np.genfromtxt (path+"hMhrefVector-sout.dat")

#chest = np.genfromtxt (path+"robot_dynamic-chest.dat")

#inputSignal = np.genfromtxt(path+"flextimator-input.dat")

#inputVel = np.genfromtxt(path+"inputVelocity-sout.dat")

#sotcontrol = np.genfromtxt(path+"solver-control.dat")

#devicestate = np.genfromtxt(path+"HRP2LAAS-state.dat")


#stabilizerRef = np.genfromtxt (path+"com-stabilized-comRef.dat")
#stabilizer = np.genfromtxt (path+"com-stabilized-task.dat")
#stabilizerError = np.genfromtxt (path+"com-stabilized-error.dat")
#stabilizerDot = np.genfromtxt (path+"com-stabilized-d2com.dat")

#flexThetaU = np.genfromtxt (path+"com-stabilizedEstimator-flexThetaU.dat")


#comRef = np.genfromtxt (path+"comref-sout.dat")

#er[:,1:er.shape[1]]=er[:,1:er.shape[1]]-er2[:,1:er.shape[1]]    

#f1 = plt.figure();ax1 = f1.add_subplot(111)
#f2 = plt.figure();ax2 = f2.add_subplot(111)
#f3 = plt.figure();ax3 = f3.add_subplot(111)
#f4 = plt.figure();ax4 = f4.add_subplot(111)
#f5 = plt.figure();ax5 = f5.add_subplot(111)
#f6 = plt.figure();ax6 = f6.add_subplot(111)
#f7 = plt.figure();ax7 = f7.add_subplot(111)
#f8 = plt.figure();ax8 = f8.add_subplot(111)
#f9 = plt.figure();ax9 = f9.add_subplot(111)
#f10 = plt.figure();ax10=f10.add_subplot(111)


#ax1.plot(f[:,0], f[:,1], label='Feet Force X')
#ax1.plot(f[:,0], f[:,2], label='Feet Force Y')
#ax1.plot(f[:,0], f[:,3], label='Feet Force Z')
#ax1.plot(f[:,0], f[:,4], label='Feet Force TX')
#ax1.plot(f[:,0], f[:,5], label='Feet Force TY')
#ax1.plot(f[:,0], f[:,6], label='Feet Force TZ')

#ax2.plot(realCom[:,0],realCom[:,1], label='CoM x')
#ax2.plot(realCom[:,0],realCom[:,2], label='CoM y')
#ax2.plot(realCom[:,0],realCom[:,3], label='CoM z')

#ax2.plot(zmpRef [:,1],zmpRef [:,2])

#ax1.plot(flex[:,0], flex[:,5]*1000, label='Flexibility')
#ax1.plot(flexDot[:,0], flexDot[:,2]*1000, label='FlexiDot')
#ax1.plot(stabilizer[:,0], stabilizer[:,1]*1000, label='stabilizer')
#ax1.plot(stabilizerDot[:,0], stabilizerDot[:,1]*1, label='stabilizerDot')
#ax1.plot(stabilizerRef[:,0], stabilizerRef[:,2]*1000, label='stabilizerRefY')


#ax3.plot(flexThetaU[:,0], flexThetaU[:,1]*1000, label='flexThetaU X')
#ax3.plot(flexThetaU[:,0], flexThetaU[:,2]*1000, label='flexThetaU Y')
#ax3.plot(flexThetaU[:,0], flexThetaU[:,3]*1000, label='flexThetaU Z')


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


#handles, labels = ax1.get_legend_handles_labels()
#ax1.legend(handles, labels)

#handles, labels = ax2.get_legend_handles_labels()
#ax2.legend(handles, labels)

#handles, labels = ax3.get_legend_handles_labels()
#ax3.legend(handles, labels)

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
