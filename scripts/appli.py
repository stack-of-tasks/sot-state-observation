# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
import matplotlib.pyplot as pl
import dynamic_graph as dg
import dynamic_graph.signal_base as dgsb

from math import sin

appli = CompensaterApplication(robot)
appli.withTraces()

if 'usingRobotViewer' in locals() and usingRobotViewer:    refreshList.append(lambda: appli.updateDisplay());    appli.initDisplay(); go()

# --- SHORTCUTS
rm           = appli.rm
push         = appli.push
dyn          = appli.robot.dynamic
sot          = appli.sot
contactLF    = appli.contactLF
contactRF    = appli.contactRF
taskLim      = appli.taskLim
taskCom      = appli.taskCom
taskChest    = appli.taskChest
taskGaze     = appli.taskGaze
taskPosture  = appli.taskPosture
taskRH       = appli.taskRH
taskR        = appli.taskCompensate
tr           = appli.robot.tracer
gopen        = optionalparentheses(appli.openGripper)
gclose       = optionalparentheses(appli.closeGripper)

t = optionalparentheses(appli.trace)
a = appli
s = optionalparentheses(appli.nextStep)

#### Flexibility Estimator ##
est= sotso.DGIMUFlexibilityEstimation('flextimator')
est.setSamplingPeriod(0.005)

meas = est.signal('measurement')
inputs = est.signal('input')
contactNbr = est.signal('contactNbr')
contactNbr.value = 1

contact1 = est.signal('contact1')
contact1.value = (0,0,0);

flex=est.signal('flexMatrixInverse')
flexdot = est.signal('flexVelocityVector')

plug(flex,appli.ccMc)
plug(flexdot,appli.ccVc)

meas.value = (0.0 , 0.0,  9.81 , 0.0 , 0.0 , 0.0)
inputs.value = (0.0, 0.0, 1.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

#stop()
