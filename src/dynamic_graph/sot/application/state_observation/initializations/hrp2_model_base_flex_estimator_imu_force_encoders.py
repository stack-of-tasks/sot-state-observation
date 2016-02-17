# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from numpy import *
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb

from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector, MatrixHomoToPose, Selec_of_vector, Inverse_of_matrixHomo, Multiply_of_matrixHomo, MatrixHomoToPoseRollPitchYaw
from dynamic_graph.sot.application.state_observation import DGIMUModelBaseFlexEstimation, PositionStateReconstructor, InputReconstructor, Odometry, Filter

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
#	self.setWithComBias(False)
	self.setProcessNoiseCovariance(matrixToTuple(np.diag((1e-8,)*12+(1e-4,)*6+(1.e-13,)*2)))
	self.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-3,)*3+(1e-6,)*3))) 

	self.leftFootPos=Multiply_of_matrixHomo("leftFootPos")
	plug(self.robot.dynamic.signal('left-ankle'),self.leftFootPos.sin1)
	self.leftFootPos.sin2.value=self.robot.forceSensorInLeftAnkle
	self.rightFootPos=Multiply_of_matrixHomo("rightFootPos")
	plug(self.robot.dynamic.signal('right-ankle'),self.rightFootPos.sin1)
	self.rightFootPos.sin2.value=self.robot.forceSensorInRightAnkle

	# Reconstruction of the position of the free flyer from encoders

        	# Create dynamic with the free flyer at the origin of the control frame
	self.robot.device.robotState.value=46*(0.,)
	self.robotState = Selec_of_vector('robotState')
	plug(self.robot.device.robotState,self.robotState.sin)
	self.robotState.selec(0,36)
	self.robot.dynamicFF=self.createDynamic(self.robotState.sout,'_dynamicFF')
        self.robot.dynamicFF.inertia.recompute(1)
        self.robot.dynamicFF.waist.recompute(1)

		# Reconstruction of the position of the contacts in dynamicFF
	self.leftFootPosFF=Multiply_of_matrixHomo("leftFootPosFF")
	plug(self.robot.dynamicFF.signal('left-ankle'),self.leftFootPosFF.sin1)
	self.leftFootPosFF.sin2.value=self.robot.forceSensorInLeftAnkle
	self.rightFootPosFF=Multiply_of_matrixHomo("rightFootPosFF")
	plug(self.robot.dynamicFF.signal('right-ankle'),self.rightFootPosFF.sin1)
	self.rightFootPosFF.sin2.value=self.robot.forceSensorInRightAnkle

		# OdometryFF
        self.odometryFF=Odometry ('OdometryFF')
	plug (self.robot.device.robotState,self.odometryFF.robotStateIn)
	plug(self.robot.frames['leftFootForceSensor'].position,self.odometryFF.leftFootPositionRef)
	plug(self.robot.frames['rightFootForceSensor'].position,self.odometryFF.rightFootPositionRef)
	plug (self.robot.device.forceLLEG,self.odometryFF.force_lf)
        plug (self.robot.device.forceRLEG,self.odometryFF.force_rf)
        plug (self.rightFootPosFF.sout,self.odometryFF.rightFootPosition)
        plug (self.leftFootPosFF.sout,self.odometryFF.leftFootPosition)
	plug (self.odometryFF.nbSupport,self.contactNbr)
	self.odometryFF.setLeftFootPosition(self.robot.frames['leftFootForceSensor'].position.value)
	self.odometryFF.setRightFootPosition(self.robot.frames['rightFootForceSensor'].position.value)

	# Create dynamicEncoders
	self.robot.dynamicEncoders=self.createDynamic(self.odometryFF.robotStateOut,'_dynamicEncoders')

        # Stack of sensors

		# IMU
        self.sensorStackimu = Stack_of_vector (name+'SensorsIMU')
        plug(self.robot.device.accelerometer,self.sensorStackimu.sin1)
        plug(self.robot.device.gyrometer,self.sensorStackimu.sin2)
        self.sensorStackimu.selec1 (0, 3)
        self.sensorStackimu.selec2 (0, 3)

                # Contacts position
        self.contactsPos = Stack_of_vector ('contactsPOS')
        plug(self.odometryFF.supportPos1,self.contactsPos.sin1)
        plug(self.odometryFF.supportPos2,self.contactsPos.sin2)
        self.contactsPos.selec1 (0, 6)
        self.contactsPos.selec2 (0, 6)

		# Contacts forces
	self.sensorStackforce = Stack_of_vector ('SensorsFORCE')
        plug(self.odometryFF.forceSupport1,self.sensorStackforce.sin1)
        plug(self.odometryFF.forceSupport2,self.sensorStackforce.sin2)
        self.sensorStackforce.selec1 (0, 6)
        self.sensorStackforce.selec2 (0, 6)

		# Calibration
	self.calibration= Calibrate('calibration')
        plug(self.odometryFF.nbSupport,self.calibration.contactsNbr)
	plug(self.sensorStackimu.sout,self.calibration.imuIn)
        plug(self.contactsPos.sout,self.calibration.contactsPositionIn)
	plug(self.robot.dynamicEncoders.com,self.calibration.comIn)

		# Concatenate
        self.sensorStack = Stack_of_vector (name+'Sensors')
        plug(self.calibration.imuOut,self.sensorStack.sin1)
        plug(self.sensorStackforce.sout,self.sensorStack.sin2)
        self.contactForces = self.sensorStack.sin2
        self.sensorStack.selec1 (0, 6)
        self.sensorStack.selec2 (0, 12)

	plug(self.sensorStack.sout,self.measurement)
    
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
        self.inputVector=InputReconstructor(name+'inputVector')
        plug(self.odometryFF.nbSupport,self.inputVector.nbContacts)
        plug(self.comVector.sout,self.inputVector.comVector)
        plug(self.robot.dynamicEncoders.inertia,self.inputVector.inertia)
	plug(self.robot.dynamicEncoders.angularmomentum,self.inputVector.angMomentum)
	plug(self.angMomDerivator.sout,self.inputVector.dangMomentum)
        self.inputVector.dinertia.value=(0,0,0,0,0,0)
        plug(self.robot.dynamicEncoders.waist,self.inputVector.positionWaist)
        plug(self.IMUVector.sout,self.inputVector.imuVector)
        plug(self.calibration.contactsPositionOut,self.inputVector.contactsPosition)

        self.inputVector.setSamplingPeriod(robot.timeStep)
        self.inputVector.setFDInertiaDot(True)     
        plug(self.inputVector.input,self.input)

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

