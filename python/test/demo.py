import sys
import numpy as np
import matplotlib.pyplot as pl
import dynamic_graph as dg
import dynamic_graph.signal_base as dgsb
import dynamic_graph.sot.core.sot_state_observation as sotso
from math import sin

np.set_printoptions(threshold=sys.maxsize)

def interact():
    import readline # optional, will allow Up/Down/History in the console
    import code
    vars = globals().copy()
    vars.update(locals())
    shell = code.InteractiveConsole(vars)
    shell.interact()

a= sotso.DGIMUFlexibilityEstimation('flextimator')
a.setSamplingPeriod(0.005)

meas = a.signal('measurement')

inputs = a.signal('input')


contactNbr = a.signal('contactNbr')
contactNbr.value = 1

contact1 = a.signal('contact1')
contact1.value = (0,0,0);

flex=a.signal('flexibility')
flexptu = a.signal('flexPoseThetaU')

lastTime = 500

values = []

for x in xrange(lastTime):
    meas.value = (0.0 ,  9.81 , 0.0 , 1.0 , 0.0 , 0.0 )
    inputs.value = (0.0, 0.0, 1.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    flexptu.recompute(x)
    values.append(flexptu.value)




# Convert into numpy 
x = np.array(xrange(lastTime))
y = np.array(values).transpose()

fig  = pl.figure()
ax1 = fig.add_subplot(121)
ax2 = fig.add_subplot(122)



# plot configuration variables
#ax1.plot(x,y[0])
#ax1.plot(x,y[1])
#ax1.plot(x,y[2])

ax1.plot(x,y[0])
ax1.plot(x,y[1])
ax1.plot(x,y[2])

# plot velocity variables
ax2.plot(x,y[3])
ax2.plot(x,y[4])
ax2.plot(x,y[5])



leg = ax1.legend(("X position ", "Y position", "Z position"))
leg = ax2.legend(("X thetamu", "Y thetamu", "Z thetamu"))
pl.show()


