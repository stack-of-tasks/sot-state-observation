# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
import matplotlib.pyplot as pl
import dynamic_graph as dg
import dynamic_graph.signal_base as dgsb
import dynamic_graph.sot.core.sot_state_observation as sotso
from math import sin
from dynamic_graph.sot.core import Stack_of_vector

from dynamic_graph.sot.application.compensater import *


appli = CompensaterApplication(robot)
appli.withTraces()

#### Flexibility Estimator ##
est= sotso.DGIMUFlexibilityEstimation('flextimator')
est.setSamplingPeriod(0.005)

meas = est.signal('measurement')
inputs = est.signal('input')
contactNbr = est.signal('contactNbr')
contactNbr.value = 2

contact1 = est.signal('contact1')
contact1.value = (0,0,0)

contact2 = est.signal('contact2')
contact2.value = (0,0,0)

rFootPos = MatrixHomoToPose('rFootFrame')
lFootPos = MatrixHomoToPose('lFootFrame')

plug(robot.frames['rightFootForceSensor'].position,rFootPos.sin)
plug(robot.frames['leftFootForceSensor'].position,lFootPos.sin)

plug(rFootPos.sout,contact1)
plug(lFootPos.sout,contact2)

sensorStack = Stack_of_vector ('sv1')
plug(robot.device.accelerometer,sensorStack.sin1)
plug(robot.device.gyrometer,sensorStack.sin2)
sensorStack.selec1 (0, 3)
sensorStack.selec2 (0, 3)

plug(sensorStack.sout,meas);


imuPos = MatrixHomoToPose('imuPos')
imuOri = MatrixToUTheta('imuOri')
imuRotM = HomoToRotation('imuRot')

plug(robot.frames['accelerometer'].position,imuPos.sin)
plug(robot.frames['accelerometer'].position,imuRotM.sin)
plug(imuRotM.sout,imuOri.sin)


inputStack1 = Stack_of_vector ('sv2')
inputStack2 = Stack_of_vector ('sv3')
inputStack3 = Stack_of_vector ('sv4')

plug(imuPos.sout,inputStack1.sin1)
inputStack1.sin2.value =(0.0 ,  0.0 , 0.0 , 0.0 , 0.0 , 0.0 )
inputStack1.selec1(0,3)
inputStack1.selec2(0,6)

plug(imuOri.sout,inputStack2.sin1)
inputStack2.sin2.value =(0.0 ,  0.0 , 0.0 )
inputStack2.selec1(0,3)
inputStack2.selec2(0,3)

plug(inputStack1.sout,inputStack3.sin1)
plug(inputStack2.sout,inputStack3.sin2)
inputStack3.selec1(0,9)
inputStack3.selec2(0,6)

plug(inputStack3.sout,inputs)


flex=est.signal('flexPoseThetaU')
plug(flex,appli.ccMc.sin)

