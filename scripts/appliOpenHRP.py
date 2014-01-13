# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
import matplotlib.pyplot as pl
import dynamic_graph as dg
import dynamic_graph.signal_base as dgsb

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
imuOri = MatrixToUTheta('imuOriUTheta')
imuRotM = HomoToRotation('imuOriM')

plug(robot.frames['accelerometer'].position,imuPos.sin)
plug(robot.frames['accelerometer'].position,imuRotM.sin)
plug(imuRotM.sout,imuOri.sin)

inputStack1 = Stack_of_vector ('imuReferencePoseThetaU')
inputStack2 = Stack_of_vector ('estimatorInput')

plug(imuPos.sout,inputStack1.sin1)
plug(imuOri.sout,inputStack2.sin2)
inputStack1.selec1(0,3)
inputStack1.selec2(0,3)

plug(imuOri.sout,inputStack2.sin1)
inputStack2.sin2.value =( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 )
inputStack2.selec1(0,6)
inputStack2.selec2(0,9)

plug(inputStack2.sout,inputs)



flex=est.signal('flexMatrixInverse')
flexdot = est.signal('flexInverseVelocityVector')

appli.robot.tracer.add( est.name +'.flexInverseVelocityVector'  ,'flexV' )
appli.robot.tracer.add( est.name +'.flexibility'  ,'flex' )
appli.robot.tracer.add( robot.device.name + '.forceLLEG', 'forceLLEG')
appli.robot.tracer.add( robot.device.name + '.forceRLEG', 'forceRLEG')
appli.robot.tracer.add( robot.device.name + '.accelerometer', 'accelerometer')
appli.robot.tracer.add( robot.device.name + '.gyrometer', 'gyrometer')

appli.startTracer()

plug(flex,appli.ccMc)
plug(flexdot,appli.ccVc)

appli.nextStep()
