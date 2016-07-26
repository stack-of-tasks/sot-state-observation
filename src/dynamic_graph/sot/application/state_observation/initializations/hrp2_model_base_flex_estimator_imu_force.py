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

from dynamic_graph.sot.application.state_observation.initializations.hrp2_mocap_to_sot import HRP2MocapToSOT 


class HRP2ModelBaseFlexEstimatorIMUForce(DGIMUModelBaseFlexEstimation):

    def __init__(self, robot, name='flextimator'):
        DGIMUModelBaseFlexEstimation.__init__(self,name)
        self.setSamplingPeriod(0.005)  
        self.robot = robot

	# Covariances
	self.setProcessNoiseCovariance(matrixToTuple(np.diag((1e-8,)*12+(1e-4,)*3+(1e-4,)*3+(1e-4,)*3+(1e-4,)*3+(1.e-2,)*6+(1e-15,)*2+(1.e-8,)*3)))
	self.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-3,)*3+(1e-6,)*3))) 
	self.setUnmodeledForceVariance(1e-13)
	self.setForceVariance(1e-4)
	self.setAbsolutePosVariance(1e-4)

	# Contact model definition
        self.setKfe(matrixToTuple(np.diag((40000,40000,40000))))
        self.setKfv(matrixToTuple(np.diag((600,600,600))))
        self.setKte(matrixToTuple(np.diag((600,600,600))))
        self.setKtv(matrixToTuple(np.diag((60,60,60))))

	# Estimator interface
	self.interface=EstimatorInterface(name+"EstimatorInterface")
	self.interface.setLeftHandSensorTransformation((0.,0.,1.57))
	self.interface.setRightHandSensorTransformation((0.,0.,1.57))

	# State and measurement definition
	self.interface.setWithUnmodeledMeasurements(False)
	self.interface.setWithModeledForces(True)
	self.interface.setWithAbsolutePose(False)
	self.setWithComBias(False)

	# Contacts velocities
	self.leftFootVelocity = Multiply_matrix_vector ('leftFootVelocity')
	plug(self.robot.frames['leftFootForceSensor'].jacobian,self.leftFootVelocity.sin1)
	plug(self.robot.device.velocity,self.leftFootVelocity.sin2)
	self.rightFootVelocity = Multiply_matrix_vector ('rightFootVelocity')
	plug(self.robot.frames['rightFootForceSensor'].jacobian,self.rightFootVelocity.sin1)
	plug(self.robot.device.velocity,self.rightFootVelocity.sin2)

        self.interface.setFDInertiaDot(True)  

	# Contacts forces, positions and velocities
		# Feet
	plug (self.robot.device.forceLLEG,self.interface.force_lf)
	plug (self.robot.device.forceRLEG,self.interface.force_rf)
	plug (self.robot.frames['leftFootForceSensor'].position,self.interface.position_lf)
	plug (self.robot.frames['rightFootForceSensor'].position,self.interface.position_rf)
	plug (self.leftFootVelocity.sout,self.interface.velocity_lf)
	plug (self.rightFootVelocity.sout,self.interface.velocity_rf)
		# Hands
	plug (self.robot.device.forceLARM,self.interface.force_lh)
	plug (self.robot.device.forceRARM,self.interface.force_rh)
	plug (self.robot.dynamic.signal('right-wrist'),self.interface.position_lh)
	plug (self.robot.dynamic.signal('left-wrist'),self.interface.position_rh)
		# Strings
	self.Peg = (0,0,4.60) # Position of the anchorage in the global frame
	self.Prl1 = np.matrix([[1,0,0,0-3.19997004e-02],[0,1,0,0.15-0],[0,0,1,1.28-1],[0,0,0,1]]) # Positions of the contacts on the robot (in the local frame) with respect to the chest
	self.Prl2 = np.matrix([[1,0,0,0-3.19997004e-02],[0,1,0,-0.15-0],[0,0,1,1.28-1],[0,0,0,1]])
	(self.contact1OpPoint,self.contact1Pos,self.contact1)=self.createContact('contact1', self.Prl1,self.Peg)
	(self.contact2OpPoint,self.contact2Pos,self.contact2)=self.createContact('contact2', self.Prl2,self.Peg)
	plug(self.contact1.sout,self.interface.position_ls)
	plug(self.contact2.sout,self.interface.position_rs)

	# Contacts model and config
	plug(self.interface.contactsModel,self.contactsModel)
	self.setWithConfigSignal(True)
	plug(self.interface.config,self.config)
	
        # Drift
	self.mocap = HRP2MocapToSOT(self.robot)
        self.drift = DriftFromMocap(name+'Drift')
	self.mocap.initialize()
	plug(self.mocap.robotPositionInMocap.sout,self.drift.limbGlobal)
	plug(self.mocap.robotPositionISot.sout,self.drift.limbLocal)

	# Measurement reconstruction
	plug(self.robot.device.accelerometer,self.interface.accelerometer)
	plug(self.robot.device.gyrometer,self.interface.gyrometer)
	plug(self.drift.driftVector,self.interface.drift)
	plug(self.interface.measurement,self.measurement)
   
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
        self.IMUVector.setFiniteDifferencesInterval(1)
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
	self.comVector.setFiniteDifferencesInterval(1)
	self.DCom.sout.recompute(0)
	self.comVector.setLastVector(self.com.value+(0.,)*15)#(0.,)*3+self.DCom.sout.value+(0.,)*9)

		# Compute derivative of Angular Momentum
        self.angMomDerivator = Derivator_of_Vector(name+'angMomDerivator')
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

    def initAbsolutePoses(self):
	self.drift.init()


    def createContact(self,name, prl,peg):
	self.contactOpPoint = OpPointModifier(name+'_opPoint')
	self.contactOpPoint.setEndEffector(False)
	self.contactOpPoint.setTransformation(matrixToTuple(prl))
	plug (self.robot.dynamic.chest,self.contactOpPoint.positionIN)

	self.contactPos = MatrixHomoToPose(name+'_pos')
	plug(self.contactOpPoint.position, self.contactPos.sin)

	self.contact = Stack_of_vector (name)
	self.contact.sin1.value = peg
	plug(self.contactPos.sout,self.contact.sin2)
	self.contact.selec1 (0, 3)
	self.contact.selec2 (0, 3)

	return (self.contactOpPoint,self.contactPos,self.contact)


