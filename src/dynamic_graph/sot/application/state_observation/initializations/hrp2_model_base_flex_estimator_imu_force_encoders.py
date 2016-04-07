# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from numpy import *
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb

from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector, MatrixHomoToPose, Selec_of_vector, Inverse_of_matrixHomo, Multiply_of_matrixHomo, MatrixHomoToPoseRollPitchYaw
from dynamic_graph.sot.application.state_observation import DGIMUModelBaseFlexEstimation, PositionStateReconstructor, Odometry, Filter, EstimatorInterface

from dynamic_graph.sot.core.derivator import Derivator_of_Vector

from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.application.state_observation import Calibrate

from dynamic_graph.sot.hrp2.dynamic_hrp2_14 import DynamicHrp2_14

class HRP2ModelBaseFlexEstimatorIMUForceEncoders(DGIMUModelBaseFlexEstimation):
    def __init__(self, robot, name='flextimator2'):
        DGIMUModelBaseFlexEstimation.__init__(self,name)
        
        self.robot = robot
	self.setSamplingPeriod(self.robot.timeStep)  
	self.setContactModel(1)
	self.setKfe(matrixToTuple(np.diag((40000,40000,40000))))
	self.setKfv(matrixToTuple(np.diag((600,600,600))))
	self.setKte(matrixToTuple(np.diag((600,600,600))))
	self.setKtv(matrixToTuple(np.diag((60,60,60))))

	self.setWithForceSensors(True)
	self.setForceVariance(1e-4)
	self.setProcessNoiseCovariance(matrixToTuple(np.diag((1e-8,)*12+(1e-4,)*6+(0e0,)*6+(1.e-13,)*2+(1.e-4,)*6)))
	self.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-3,)*3+(1e-6,)*3+(1e-13,)*6))) 

	#Estimator interface
	self.interface=EstimatorInterface("EstimatorInterface")
	self.interface.setLeftHandSensorTransformation((0.,0.,1.57))
	self.interface.setRightHandSensorTransformation((0.,0.,1.57))
        self.interface.setFDInertiaDot(True)  

	# Contacts forces
	plug (self.robot.device.forceLLEG,self.interface.force_lf)
	plug (self.robot.device.forceRLEG,self.interface.force_rf)
	plug (self.robot.device.forceLARM,self.interface.force_lh)
	plug (self.robot.device.forceRARM,self.interface.force_rh)

	# Selecting robotState
	self.robot.device.robotState.value=46*(0.,)
	self.robotState = Selec_of_vector('robotState')
	plug(self.robot.device.robotState,self.robotState.sin)
	self.robotState.selec(0,36)

	# Reconstruction of the position of the free flyer from encoders

        	# Create dynamic with the free flyer at the origin of the control frame
	self.robot.dynamicOdo=self.createDynamic(self.robotState.sout,'_dynamicOdo')
        self.robot.dynamicOdo.inertia.recompute(1)
        self.robot.dynamicOdo.waist.recompute(1)

		# Reconstruction of the position of the contacts in dynamicOdo
	self.leftFootPosOdo=Multiply_of_matrixHomo("leftFootPosOdo")
	plug(self.robot.dynamicOdo.signal('left-ankle'),self.leftFootPosOdo.sin1)
	self.leftFootPosOdo.sin2.value=self.robot.forceSensorInLeftAnkle
	self.rightFootPosOdo=Multiply_of_matrixHomo("rightFootPosOdo")
	plug(self.robot.dynamicOdo.signal('right-ankle'),self.rightFootPosOdo.sin1)
	self.rightFootPosOdo.sin2.value=self.robot.forceSensorInRightAnkle

		# Odometry
        self.odometry=Odometry ('odometry')
	plug (self.robot.frames['leftFootForceSensor'].position,self.odometry.leftFootPositionRef)
	plug (self.robot.frames['rightFootForceSensor'].position,self.odometry.rightFootPositionRef)
        plug (self.rightFootPosOdo.sout,self.odometry.rightFootPositionIn)
        plug (self.leftFootPosOdo.sout,self.odometry.leftFootPositionIn)
	plug (self.robot.device.forceLLEG,self.odometry.force_lf)
        plug (self.robot.device.forceRLEG,self.odometry.force_rf)
	self.odometry.setLeftFootPosition(self.robot.frames['leftFootForceSensor'].position.value)
	self.odometry.setRightFootPosition(self.robot.frames['rightFootForceSensor'].position.value)
	plug(self.interface.stackOfSupportContacts,self.odometry.stackOfSupportContacts)

	# Create dynamicEncoders
	self.robot.dynamicEncoders=self.createDynamic(self.robotState.sout,'_dynamicEncoders')
	plug(self.odometry.freeFlyer,self.robot.dynamicEncoders.ffposition)

	# Reconstruction of the position of the contacts in dynamicEncoders
#	self.leftFootPos=Multiply_of_matrixHomo("leftFootPos")
#	plug(self.robot.dynamicEncoders.signal('left-ankle'),self.leftFootPos.sin1)
#	self.leftFootPos.sin2.value=self.robot.forceSensorInLeftAnkle
#	self.rightFootPos=Multiply_of_matrixHomo("rightFootPos")
#	plug(self.robot.dynamicEncoders.signal('right-ankle'),self.rightFootPos.sin1)
#	self.rightFootPos.sin2.value=self.robot.forceSensorInRightAnkle

	# Contacts positions
#	plug (self.leftFootPos.sout,self.interface.position_lf)
#	plug (self.rightFootPos.sout,self.interface.position_rf)
	plug (self.odometry.leftFootPositionOut,self.interface.position_lf)
	plug (self.odometry.rightFootPositionOut,self.interface.position_rf)
	plug (self.robot.dynamicEncoders.signal('right-wrist'),self.interface.position_lh)
	plug (self.robot.dynamicEncoders.signal('left-wrist'),self.interface.position_rh)

	# Compute contacts number
	plug (self.interface.supportContactsNbr,self.contactNbr)

        # Compute measurement vector
        plug(self.robot.device.accelerometer,self.interface.accelerometer)
        plug(self.robot.device.gyrometer,self.interface.gyrometer)
	plug(self.interface.measurement,self.measurement)
    
        # Input reconstruction

		# IMU Vector
			# Creating an operational point for the IMU
        self.robot.dynamicEncoders.createJacobian(name+'ChestJ_OpPoint','chest')
        self.imuOpPoint = OpPointModifier(name+'IMU_oppoint')
        self.imuOpPoint.setTransformation(matrixToTuple(np.linalg.inv(np.matrix(self.robot.dynamicEncoders.chest.value))*np.matrix(self.robot.frames['accelerometer'].position.value)))
        self.imuOpPoint.setEndEffector(False)
        plug (self.robot.dynamicEncoders.chest,self.imuOpPoint.positionIN)	
        plug (self.robot.dynamicEncoders.signal(name+'ChestJ_OpPoint'),self.imuOpPoint.jacobianIN)
			# IMU position
	self.PosAccelerometer=Multiply_of_matrixHomo("PosAccelerometer")
	plug(self.robot.dynamicEncoders.chest,self.PosAccelerometer.sin1)
	self.PosAccelerometer.sin2.value=matrixToTuple(self.robot.accelerometerPosition)
        self.inputPos = MatrixHomoToPoseUTheta(name+'InputPosition')
        plug(self.PosAccelerometer.sout,self.inputPos.sin)
			# IMU velocity
        self.inputVel = Multiply_matrix_vector(name+'InputVelocity')
        plug(self.imuOpPoint.jacobian,self.inputVel.sin1)
        plug(self.robot.dynamicEncoders.velocity,self.inputVel.sin2)
			# Concatenate
        self.inputPosVel = Stack_of_vector (name+'InputPosVel')
        plug(self.inputPos.sout,self.inputPosVel.sin1)
        plug(self.inputVel.sout,self.inputPosVel.sin2)
        self.inputPosVel.selec1 (0, 6)
        self.inputPosVel.selec2 (0, 6)
			# IMU Vector
        self.IMUVector = PositionStateReconstructor (name+'EstimatorInput')
        plug(self.inputPosVel.sout,self.IMUVector.sin)
        self.IMUVector.inputFormat.value  = '001111'
        self.IMUVector.outputFormat.value = '011111'
        self.IMUVector.setFiniteDifferencesInterval(2)
        	# CoM and derivatives
        self.comIn=self.robot.dynamicEncoders.com
        self.comVector = PositionStateReconstructor (name+'ComVector')
        plug(self.comIn,self.comVector.sin)
        self.comVector.inputFormat.value  = '000001'
        self.comVector.outputFormat.value = '010101'  
	self.comVector.setFiniteDifferencesInterval(20)
		# Compute derivative of Angular Momentum
        self.angMomDerivator = Derivator_of_Vector('angMomDerivator')
        plug(self.robot.dynamicEncoders.angularmomentum,self.angMomDerivator.sin)
        self.angMomDerivator.dt.value = self.robot.timeStep          
        	# Concatenate with InputReconstructor entity
        plug(self.comVector.sout,self.interface.comVector)
        plug(self.robot.dynamicEncoders.inertia,self.interface.inertia)
	plug(self.robot.dynamicEncoders.angularmomentum,self.interface.angMomentum)
	plug(self.angMomDerivator.sout,self.interface.dangMomentum)
        self.interface.dinertia.value=(0,0,0,0,0,0)
        plug(self.robot.dynamicEncoders.waist,self.interface.positionWaist)
        plug(self.IMUVector.sout,self.interface.imuVector)
 
        plug(self.interface.input,self.input)

        self.robot.flextimator = self

    # Create a dynamic ######################################

    def createDynamic(self,state,name) :
	# Create dynamic
        dynamicTmp = self.robot.loadModelFromJrlDynamics(
                              self.robot.name + name, 
                              self.robot.modelDir, 
                              self.robot.modelName,
                              self.robot.specificitiesPath,
                              self.robot.jointRankPath,
                              DynamicHrp2_14)
        dynamicTmp.dimension = dynamicTmp.getDimension()
        if dynamicTmp.dimension != len(self.robot.halfSitting):
            raise RuntimeError("Dimension of half-sitting: {0} differs from dimension of robot: {1}".format (len(self.halfSitting), dynamicTmp.dimension))

	# Pluging position
	plug(state, dynamicTmp.position)

	self.derivative=True

	# Pluging velocity
	self.robot.enableVelocityDerivator = self.derivative
 	if self.robot.enableVelocityDerivator:
            dynamicTmp.velocityDerivator = Derivator_of_Vector('velocityDerivator')
            dynamicTmp.velocityDerivator.dt.value = self.robot.timeStep
            plug(state, dynamicTmp.velocityDerivator.sin)
            plug(dynamicTmp.velocityDerivator.sout, dynamicTmp.velocity)
        else:
            dynamicTmp.velocity.value = dynamicTmp.dimension*(0.,)

	# Pluging acceleration
	self.robot.enableAccelerationDerivator = self.derivative
        if self.robot.enableAccelerationDerivator:
            dynamicTmp.accelerationDerivator = Derivator_of_Vector('accelerationDerivator')
            dynamicTmp.accelerationDerivator.dt.value = self.robot.timeStep
            plug(dynamicTmp.velocityDerivator.sout,
                 dynamicTmp.accelerationDerivator.sin)
            plug(dynamicTmp.accelerationDerivator.sout, dynamicTmp.acceleration)
        else:
            dynamicTmp.acceleration.value = dynamicTmp.dimension*(0.,)

        for dynamicTmp.op in self.robot.OperationalPoints:
            dynamicTmp.createOpPoint(dynamicTmp.op, dynamicTmp.op)

	return dynamicTmp

