# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
import matplotlib.pyplot as pl
import dynamic_graph as dg
import dynamic_graph.signal_base as dgsb
import dynamic_graph.sot.core.sot_state_observation as sotso
from dynamic_graph.sot.application.compensater import *
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
a= sotso.DGIMUFlexibilityEstimation('flextimator')
a.setSamplingPeriod(0.005)

meas = a.signal('measurement')
inputs = a.signal('input')
contactNbr = a.signal('contactNbr')
contactNbr.value = 1

contact1 = a.signal('contact1')
contact1.value = (0,0,0);

flex=a.signal('flexPoseThetaU')


plug(flex,appli.ccMc.sin)

meas.value = (0.0 ,  9.8 , 0.0 , 0.0 , 0.0 , 0.0 )
inputs.value = (0.0, 0.0, 1.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)




#stop()
