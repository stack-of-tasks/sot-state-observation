import numpy as np
from numpy import linalg as LA
import matplotlib.pyplot as plt
from scipy import stats

#path = 'logs/hrp-2/static-calibration/ds-frontal/robot/2015-05-28/'
path = 'logs/hrp-2/static-calibration/ds-frontal/simu/2014-05-30/'


robotMass = 58.0
gravity = 9.8

fl = np.genfromtxt (path+"HRP2LAAS-forceLLEG.dat")
fr = np.genfromtxt (path+"HRP2LAAS-forceRLEG.dat")
fr = fr.copy()
fr.resize(fl.shape[0],fr.shape[1])
f= (fl + fr)

realCom = np.genfromtxt (path+"real-com-sout.dat")

flexThetaU = np.genfromtxt (path+"/com-stabilizedEstimator-flexThetaU.dat")

size = 81

samplef =   np.array((0.0,)*size)
sampleCom = np.array((0.0,)*size)
sampleTh =  np.array((0.0,)*size)

firstIndice = 6601-flexThetaU[0,0]
for i in range(0,size):
    indice       = firstIndice + 3200*i
    samplef[i]   = f[indice,5]
    sampleCom[i] = realCom[indice,1]*robotMass*gravity
    sampleTh[i]  = flexThetaU[indice,2]

slope, intercept, r_value, p_value, std_err = stats.linregress(samplef,sampleTh)
line = slope*samplef+intercept

#comRef = np.genfromtxt ("/tmp/comref-sout.dat")

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


ax1.plot(samplef , sampleCom , label='tau Vs CoM')


ax2.plot(samplef , sampleTh , label='tau Vs Th')
ax2.plot(samplef , line , label='linearRegression')

ax3.plot(sampleCom , sampleTh, label='CoM Vs Th')




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


