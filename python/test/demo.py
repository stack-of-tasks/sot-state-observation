import numpy as np
import matplotlib.pyplot as pl
import dynamic_graph as dg
import dynamic_graph.signal_base as dgsb
import dynamic_graph.sot.core.sot_state_observation as sotso
from math import sin

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
meas.value = (0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 )

inputs = a.signal('input')
inputs.value = (0.0, 0.0, 1.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

contactNbr = a.signal('contactNbr')
contactNbr.value = 1

contact1 = a.signal('contact1')
contact1.value = (0,0,0);

flex=a.signal('flexibility')

lastTime = 500

values = []
for x in xrange(lastTime):
    meas.value = (0.0 , 0.0 , 9.81 , 1.0 , 0.0 , 0.0 )
    values.append(flex.value)
    a.increment()



# Convert into numpy 
x = np.array(xrange(lastTime))
y = np.array(values).transpose()

fig  = pl.figure()
ax1 = fig.add_subplot(121)
ax2 = fig.add_subplot(122)



# plot configuration variables
ax1.plot(x,y[0])
ax1.plot(x,y[1])
ax1.plot(x,y[2])

# plot velocity variables
ax2.plot(x,y[6])
ax2.plot(x,y[7])
ax2.plot(x,y[8])

leg = ax1.legend(("X position ", "Y position", "Z position"))
leg = ax2.legend(("X thetamu", "Y thetamu", "Z thetamu"))
pl.show()
