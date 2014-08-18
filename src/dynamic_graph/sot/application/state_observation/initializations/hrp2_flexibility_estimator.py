# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb

from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector
from dynamic_graph.sot.application.state_observation import DGIMUFlexibilityEstimation, PositionStateReconstructor

from dynamic_graph.sot.core.matrix_util import matrixToTuple


class HRP2FlexibilityEstimator(DGIMUFlexibilityEstimation):
    def __init__(self, robot, name='flextimator'):
        DGIMUFlexibilityEstimation.__init__(self,name)
        self.setSamplingPeriod(0.005)
        self.robot = robot
        
        self.sensorStack = Stack_of_vector (name+'Sensors')
        plug(robot.device.accelerometer,self.sensorStack.sin1)
        plug(robot.device.gyrometer,self.sensorStack.sin2)
        self.sensorStack.selec1 (0, 3)
        self.sensorStack.selec2 (0, 3)

        plug(self.sensorStack.sout,self.measurement);

        self.inputPos = MatrixHomoToPoseUTheta(name+'InputPosition')

        plug(robot.frames['accelerometer'].position,self.inputPos.sin)

        robot.dynamic.createJacobian('ChestJ_OpPoint','chest')
        self.imuOpPoint = OpPointModifier('IMU_oppoint')
        self.imuOpPoint.setEndEffector(False)
       
        self.imuOpPoint.setTransformation(matrixToTuple(np.linalg.inv(np.matrix(self.robot.dynamic.chest.value))*np.matrix(self.robot.frames['accelerometer'].position.value)))

        plug (robot.dynamic.chest,self.imuOpPoint.positionIN)
        plug (robot.dynamic.signal('ChestJ_OpPoint'),self.imuOpPoint.jacobianIN)

        self.inputVel = Multiply_matrix_vector(name+'InputVelocity')
        plug(self.imuOpPoint.jacobian,self.inputVel.sin1)
        plug(robot.device.velocity,self.inputVel.sin2)

        self.inputPosVel = Stack_of_vector (name+'InputPosVel')
        plug(self.inputPos.sout,self.inputPosVel.sin1)
        plug(self.inputVel.sout,self.inputPosVel.sin2)
        self.inputPosVel.selec1 (0, 6)
        self.inputPosVel.selec2 (0, 6)

        self.inputVector = PositionStateReconstructor (name+'EstimatorInput')
        plug(self.inputPosVel.sout,self.inputVector.sin)
        self.inputVector.inputFormat.value  = '001111'
        self.inputVector.outputFormat.value = '011111'
        self.inputVector.setFiniteDifferencesInterval(2)

        plug(self.inputVector.sout,self.input)

        robot.flextimator = self
