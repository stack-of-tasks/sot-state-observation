# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
import dynamic_graph as dg
import dynamic_graph.signal_base as dgsb

from math import sin
from dynamic_graph.sot.core import Stack_of_vector

from dynamic_graph.sot.application.stabilizer.compensater import *


appli = TwoHandsCompensater(robot)
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



rhMcc = Inverse_of_matrixHomo('hMcc')
plug(appli.robot.dynamic.rh,rhMcc.sin)
hMrhref = Multiply_of_matrixHomo('hMhref')
plug(rhMcc.sout    ,hMrhref.sin1)
plug(appli.ccMrhref,hMrhref.sin2)
hMrhrefVector = MatrixHomoToPoseUTheta('hMhrefVector')
plug(hMrhref.sout,hMrhrefVector.sin)




appli.robot.addTrace( est.name,'flexInverseVelocityVector' )
appli.robot.addTrace( est.name,'flexTransformationMatrix' )
appli.robot.addTrace( est.name,'flexVelocityVector')
appli.robot.addTrace( est.name,'flexibility'  )
appli.robot.addTrace( est.name, 'input')
appli.robot.addTrace( est.name, 'measurement')
appli.robot.addTrace( est.name, 'inovation')
appli.robot.addTrace( est.name, 'predictedSensors')
appli.robot.addTrace( est.name , 'simulatedSensors' )

appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name,  'gyrometer')

appli.robot.addTrace( hMrhrefVector.name,'sout')

appli.robot.addTrace( appli.taskCompensateR.task.name,'error')
appli.robot.addTrace( appli.taskCompensateR.featureDes.name, 'position')
appli.robot.addTrace( appli.taskCompensateR.featureDes.name, 'velocity')

appli.robot.addTrace( appli.transformerR.name, 'gM0')
appli.robot.addTrace( appli.transformerR.name, 'gV0')
appli.robot.addTrace( appli.transformerR.name, 'lM0')
appli.robot.addTrace( appli.transformerR.name, 'lV0')
appli.robot.addTrace( appli.transformerR.name, 'gMl')
appli.robot.addTrace( appli.transformerR.name, 'gVl')

appli.robot.addTrace( appli.robot.dynamic.name,'chest')
appli.robot.addTrace( appli.robot.device.name,'state')
appli.robot.addTrace( appli.robot.device.name,'robotState')

appli.startTracer()

plug(flex,appli.ccMc_R)
plug(flexdot,appli.ccVc_R)

#appli.rm(appli.taskPosture)

appli.nextStep()
