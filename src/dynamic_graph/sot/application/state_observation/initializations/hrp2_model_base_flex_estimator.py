# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from numpy import *
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb

from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector, MatrixHomoToPose
from dynamic_graph.sot.application.state_observation import DGIMUModelBaseFlexEstimation, PositionStateReconstructor, InputReconstructor

from dynamic_graph.sot.core.derivator import Derivator_of_Vector

from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.application.state_observation import Calibrate


class HRP2ModelBaseFlexEstimator(DGIMUModelBaseFlexEstimation):
    def __init__(self, robot, name='flextimator2'):
        DGIMUModelBaseFlexEstimation.__init__(self,name)
        
        self.setSamplingPeriod(0.005)  
        self.robot = robot

        # Definition of IMU vector
        self.sensorStack = Stack_of_vector (name+'Sensors')
        plug(self.robot.device.accelerometer,self.sensorStack.sin1)
        plug(self.robot.device.gyrometer,self.sensorStack.sin2)
        self.sensorStack.selec1 (0, 3)
        self.sensorStack.selec2 (0, 3)

	# Calibration
	self.calibration= Calibrate('calibration')
	plug(self.sensorStack.sout,self.calibration.imuIn)
	plug(self.robot.dynamic.com,self.calibration.comIn)
        plug(self.calibration.imuOut,self.measurement)

        self.inputPos = MatrixHomoToPoseUTheta(name+'InputPosition')
        plug(robot.frames['accelerometer'].position,self.inputPos.sin)
        self.robot.dynamic.createJacobian('ChestJ_OpPoint','chest')
        self.imuOpPoint = OpPointModifier('IMU_oppoint')
        self.imuOpPoint.setEndEffector(False)
        self.imuOpPoint.setTransformation(matrixToTuple(np.linalg.inv(np.matrix(self.robot.dynamic.chest.value))*np.matrix(self.robot.frames['accelerometer'].position.value)))
        plug (self.robot.dynamic.chest,self.imuOpPoint.positionIN)
        plug (self.robot.dynamic.signal('ChestJ_OpPoint'),self.imuOpPoint.jacobianIN)
        self.inputVel = Multiply_matrix_vector(name+'InputVelocity')
        plug(self.imuOpPoint.jacobian,self.inputVel.sin1)
        plug(self.robot.device.velocity,self.inputVel.sin2)
        self.inputPosVel = Stack_of_vector (name+'InputPosVel')
        plug(self.inputPos.sout,self.inputPosVel.sin1)
        plug(self.inputVel.sout,self.inputPosVel.sin2)
        self.inputPosVel.selec1 (0, 6)
        self.inputPosVel.selec2 (0, 6)
        self.IMUVector = PositionStateReconstructor (name+'EstimatorInput')
        plug(self.inputPosVel.sout,self.IMUVector.sin)
        self.IMUVector.inputFormat.value  = '001111'
        self.IMUVector.outputFormat.value = '011111'
        self.IMUVector.setFiniteDifferencesInterval(2)


        # Definition of inertia, angular momentum and derivatives
        self.robot.dynamic.inertia.recompute(0)
        self.inertia=self.robot.dynamic.inertia #(48.2378,48.2378,2.87339,0,0,0) #
        self.dotInertia=(0,0,0,0,0,0)
        self.zeroMomentum=(0,0,0,0,0,0)
        
        # Waist position
        self.robot.dynamic.waist.recompute(0)
        #self.robot.dynamic.chest.recompute(1)
        #self.robot.dynamic.com.recompute(1)
        self.positionWaist=self.robot.dynamic.waist


        # Definition of com and derivatives
        self.com=self.calibration.comOut #self.robot.dynamic.com
        self.DCom = Multiply_matrix_vector(name+'DCom')
	self.robot.dynamic.Jcom.recompute(0)
        plug(self.robot.dynamic.Jcom,self.DCom.sin1)
        plug(self.robot.device.velocity,self.DCom.sin2)
        self.comVectorIn = Stack_of_vector (name+'ComVectorIn')
        plug(self.calibration.comOut,self.comVectorIn.sin1)
        plug(self.DCom.sout,self.comVectorIn.sin2)
        self.comVectorIn.selec1 (0, 3)
        self.comVectorIn.selec2 (0, 3)
        self.comVector = PositionStateReconstructor (name+'ComVector')
        plug(self.comVectorIn.sout,self.comVector.sin)
        self.comVector.inputFormat.value  = '000101'
        self.comVector.outputFormat.value = '010101'
      
             
        
        # Concatenate with InputReconstructor entity
        self.inputVector=InputReconstructor(name+'inputVector')
        plug(self.comVector.sout,self.inputVector.comVector)
        plug(self.inertia,self.inputVector.inertia)
        self.inputVector.dinertia.value=self.dotInertia
        plug(self.positionWaist,self.inputVector.positionWaist)
        self.inputVector.setSamplingPeriod(robot.timeStep)
        self.inputVector.setFDInertiaDot(True)

        plug(self.robot.dynamic.angularmomentum,self.inputVector.angMomentum)
        self.angMomDerivator = Derivator_of_Vector('angMomDerivator')
        plug(self.robot.dynamic.angularmomentum,self.angMomDerivator.sin)
        self.angMomDerivator.dt.value = self.robot.timeStep
        plug(self.angMomDerivator.sout,self.inputVector.dangMomentum)
        #self.inputVector.dangMomentum.value = self.zeroMomentum

        plug(self.IMUVector.sout,self.inputVector.imuVector)
        plug(self.contactNbr,self.inputVector.nbContacts)
        # plug(self.contacts,self.inputVector.contactsPosition)
        
       
        plug(self.inputVector.input,self.input)
        self.robot.flextimator = self

        kfe=40000
        kfv=600
        kte=600
        ktv=60
        self.setKfe(matrixToTuple(np.diag((kfe,kfe,kfe))))
        self.setKfv(matrixToTuple(np.diag((kfv,kfv,kfv))))
        self.setKte(matrixToTuple(np.diag((kte,kte,kte))))
        self.setKtv(matrixToTuple(np.diag((ktv,ktv,ktv))))
