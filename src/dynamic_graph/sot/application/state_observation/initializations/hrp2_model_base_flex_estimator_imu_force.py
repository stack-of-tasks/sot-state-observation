# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from numpy import *
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb

from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector, MatrixHomoToPose
from dynamic_graph.sot.application.state_observation import DGIMUModelBaseFlexEstimation, PositionStateReconstructor, InputReconstructor, EstimatorInterface, DriftFromMocap

from dynamic_graph.sot.core.derivator import Derivator_of_Vector

from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.application.state_observation import Calibrate


class HRP2ModelBaseFlexEstimatorIMUForce(DGIMUModelBaseFlexEstimation):
    def __init__(self, robot, name='flextimator2'):

        DGIMUModelBaseFlexEstimation.__init__(self,name)
        self.setSamplingPeriod(0.005)  
        self.robot = robot
	self.setContactModel(1)

	self.setWithForceSensors(True)
	self.setForceVariance(1e-4)
	self.setWithComBias(False)

	self.setProcessNoiseCovariance(matrixToTuple(np.diag((1e-8,)*12+(1e-4,)*6+(1.e-13,)*2+(1.e-10,)*6)))
	self.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-3,)*3+(1e-6,)*3))) 

        self.setKfe(matrixToTuple(np.diag((40000,40000,40000))))
        self.setKfv(matrixToTuple(np.diag((600,600,600))))
        self.setKte(matrixToTuple(np.diag((600,600,600))))
        self.setKtv(matrixToTuple(np.diag((60,60,60))))

	#Estimator interface
	self.interface=EstimatorInterface("EstimatorInterface")
	self.interface.setLeftHandSensorTransformation((-2.,1.,3.))
	self.interface.setRightHandSensorTransformation((-2.,1.,3.))
        self.interface.setFDInertiaDot(True)  

	# Contacts forces anf positions
	plug (self.robot.device.forceLLEG,self.interface.force_lf)
	plug (self.robot.device.forceRLEG,self.interface.force_rf)
	plug (self.robot.frames['leftFootForceSensor'].position,self.interface.position_lf)
	plug (self.robot.frames['rightFootForceSensor'].position,self.interface.position_rf)
	plug (self.robot.device.forceLARM,self.interface.force_lh)
	plug (self.robot.device.forceRARM,self.interface.force_rh)
	plug (self.robot.dynamic.signal('right-wrist'),self.interface.position_lh)
	plug (self.robot.dynamic.signal('left-wrist'),self.interface.position_rh)

        # Drift
        self.drift = DriftFromMocap(name+'Drift')

	# Meausrement reconstruction
	plug(self.robot.device.accelerometer,self.interface.accelerometer)
	plug(self.robot.device.gyrometer,self.interface.gyrometer)
	# plug(self.drift,)
	plug(self.interface.measurement,self.measurement);
   
        # Input reconstruction

		# IMU Vector
        self.inputPos = MatrixHomoToPoseUTheta(name+'InputPosition')
        plug(robot.frames['accelerometer'].position,self.inputPos.sin)
        self.robot.dynamic.createJacobian(name+'ChestJ_OpPoint','chest')	
        self.imuOpPoint = OpPointModifier(name+'IMU_oppoint')
        self.imuOpPoint.setEndEffector(False)
        self.imuOpPoint.setTransformation(matrixToTuple(np.linalg.inv(np.matrix(self.robot.dynamic.chest.value))*np.matrix(self.robot.frames['accelerometer'].position.value)))
        plug (self.robot.dynamic.chest,self.imuOpPoint.positionIN)			
        plug (self.robot.dynamic.signal(name+'ChestJ_OpPoint'),self.imuOpPoint.jacobianIN)
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
	self.inputPosVel.sout.recompute(0)
	self.IMUVector.setLastVector(self.inputPosVel.sout.value+(0.,)*6)

        	# CoM and derivatives
        self.com=self.robot.dynamic.com
        self.DCom = Multiply_matrix_vector(name+'DCom')
        plug(self.robot.dynamic.Jcom,self.DCom.sin1)
        plug(self.robot.device.velocity,self.DCom.sin2)
        self.comVectorIn = Stack_of_vector (name+'ComVectorIn')
        plug(self.com,self.comVectorIn.sin1)
        plug(self.DCom.sout,self.comVectorIn.sin2)
        self.comVectorIn.selec1 (0, 3)
        self.comVectorIn.selec2 (0, 3)
        self.comVector = PositionStateReconstructor (name+'ComVector')
        plug(self.comVectorIn.sout,self.comVector.sin)
        self.comVector.inputFormat.value  = '000101'
        self.comVector.outputFormat.value = '010101'  
	self.comVector.setFiniteDifferencesInterval(20)
	self.DCom.sout.recompute(0)
	self.comVector.setLastVector(self.com.value+(0.,)*15)#(0.,)*3+self.DCom.sout.value+(0.,)*9)

		# Compute derivative of Angular Momentum
        self.angMomDerivator = Derivator_of_Vector('angMomDerivator')
        plug(self.robot.dynamic.angularmomentum,self.angMomDerivator.sin)
        self.angMomDerivator.dt.value = self.robot.timeStep 

#        self.angMomDerivator = PositionStateReconstructor (name+'angMomDerivator')
#        plug(self.robot.dynamic.angularmomentum,self.angMomDerivator.sin)
#        self.angMomDerivator.inputFormat.value  = '000001'
#        self.angMomDerivator.outputFormat.value = '000100'  
#	 self.angMomDerivator.setFiniteDifferencesInterval(2)
#	 self.robot.dynamic.angularmomentum.recompute(0)
#	 self.angMomDerivator.setLastVector(self.robot.dynamic.angularmomentum.value+(0.,)*15)       
        
        	# Concatenate with interface estimator
	plug(self.comVector.sout,self.interface.comVector)
	plug(self.robot.dynamic.inertia,self.interface.inertia)
	self.interface.dinertia.value=(0,0,0,0,0,0)
	plug(self.robot.dynamic.angularmomentum,self.interface.angMomentum)
	plug(self.angMomDerivator.sout,self.interface.dangMomentum)
	plug(self.robot.dynamic.waist,self.interface.positionWaist)
	plug(self.IMUVector.sout,self.interface.imuVector)
 
        plug(self.interface.input,self.input)
	plug (self.interface.modeledContactsNbr,self.contactNbr)

        self.robot.flextimator = self


