# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from numpy import *
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb

from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector, MatrixHomoToPose, Selec_of_vector, Inverse_of_matrixHomo, Multiply_of_matrixHomo, MatrixHomoToPoseRollPitchYaw
from dynamic_graph.sot.application.state_observation import DGIMUModelBaseFlexEstimation, PositionStateReconstructor, Odometry, Filter, EstimatorInterface, DriftFromMocap

from dynamic_graph.sot.core.derivator import Derivator_of_Vector

from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.application.state_observation import Calibrate

from dynamic_graph.sot.hrp2.dynamic_hrp2_14 import DynamicHrp2_14
from dynamic_graph.sot.core.feature_position import FeaturePosition
from dynamic_graph.sot.core import Task, FeatureGeneric
from dynamic_graph.sot.core import GainAdaptive

class HRP2ModelBaseFlexEstimatorIMUForceEncoders(DGIMUModelBaseFlexEstimation):
    def __init__(self, robot, name='flextimatorEncoders'):
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
	self.setContactModel(1)
        self.setKfe(matrixToTuple(np.diag((40000,40000,40000))))
        self.setKfv(matrixToTuple(np.diag((600,600,600))))
        self.setKte(matrixToTuple(np.diag((600,600,600))))
        self.setKtv(matrixToTuple(np.diag((60,60,60))))

	#Estimator interface
	self.interface=EstimatorInterface(name+"EstimatorInterface")
	self.interface.setLeftHandSensorTransformation((0.,0.,1.57))
	self.interface.setRightHandSensorTransformation((0.,0.,1.57))
        self.interface.setFDInertiaDot(True)  

	# State and measurement definition
	self.interface.setWithUnmodeledMeasurements(False)
	self.interface.setWithModeledForces(True)
	self.interface.setWithAbsolutePose(False)
	self.setWithComBias(False)

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
	self.leftFootPosOdo=Multiply_of_matrixHomo(name+"leftFootPosOdo")
	plug(self.robot.dynamicOdo.signal('left-ankle'),self.leftFootPosOdo.sin1)
	self.leftFootPosOdo.sin2.value=self.robot.forceSensorInLeftAnkle
	self.rightFootPosOdo=Multiply_of_matrixHomo(name+"rightFootPosOdo")
	plug(self.robot.dynamicOdo.signal('right-ankle'),self.rightFootPosOdo.sin1)
	self.rightFootPosOdo.sin2.value=self.robot.forceSensorInRightAnkle

		# Odometry
        self.odometry=Odometry (name+'odometry')
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
	self.robotStateWoFF = Selec_of_vector('robotStateWoFF')
	plug(self.robot.device.robotState,self.robotStateWoFF.sin)
	self.robotStateWoFF.selec(6,36)
        self.stateEncoders = Stack_of_vector (name+'stateEncoders')
        plug(self.odometry.freeFlyer,self.stateEncoders.sin1)
        plug(self.robotStateWoFF.sout,self.stateEncoders.sin2)
        self.stateEncoders.selec1 (0, 6)
        self.stateEncoders.selec2 (0, 30)
	self.robot.dynamicEncoders=self.createDynamic(self.stateEncoders.sout,'_dynamicEncoders')
#	self.robot.dynamicEncoders=self.createDynamic(self.robotState.sout,'_dynamicEncoders')
#	plug(self.odometry.freeFlyer,self.robot.dynamicEncoders.ffposition)
#	self.robot.dynamicEncoders=self.createDynamic(self.robot.device.state,'_dynamicEncoders')

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

	# Contacts velocities
	self.leftFootVelocity = Multiply_matrix_vector ('leftFootVelocity')
	plug(self.robot.frames['leftFootForceSensor'].jacobian,self.leftFootVelocity.sin1)
	plug(self.robot.dynamicEncoders.velocity,self.leftFootVelocity.sin2)
	self.rightFootVelocity = Multiply_matrix_vector ('rightFootVelocity')
	plug(self.robot.frames['rightFootForceSensor'].jacobian,self.rightFootVelocity.sin1)
	plug(self.robot.dynamicEncoders.velocity,self.rightFootVelocity.sin2)

	# Compute contacts number
	plug (self.interface.supportContactsNbr,self.contactNbr)

	# Contacts model and config
	plug(self.interface.contactsModel,self.contactsModel)
	self.setWithConfigSignal(True)
	plug(self.interface.config,self.config)

        # Drift
        self.drift = DriftFromMocap(name+'Drift')

        # Compute measurement vector
        plug(self.robot.device.accelerometer,self.interface.accelerometer)
        plug(self.robot.device.gyrometer,self.interface.gyrometer)
	plug(self.drift.driftVector,self.interface.drift)
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
	self.PosAccelerometer=Multiply_of_matrixHomo(name+"PosAccelerometer")
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
        self.angMomDerivator = Derivator_of_Vector(name+'angMomDerivator')
        plug(self.robot.dynamicEncoders.angularmomentum,self.angMomDerivator.sin)
        self.angMomDerivator.dt.value = self.robot.timeStep          
        	# Concatenate with interace estimator
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

    def createCenterOfMassFeatureAndTask(self,
    				         dynamicTmp,
                                         featureName, featureDesName,
                                         taskName,
                                         selec = '111',
                                         ingain = 1.):
        dynamicTmp.com.recompute(0)
        dynamicTmp.Jcom.recompute(0)
    
        featureCom = FeatureGeneric(featureName)
        plug(dynamicTmp.com, featureCom.errorIN)
        plug(dynamicTmp.Jcom, featureCom.jacobianIN)
        featureCom.selec.value = selec
        featureComDes = FeatureGeneric(featureDesName)
        featureComDes.errorIN.value = dynamicTmp.com.value
        featureCom.setReference(featureComDes.name)
        taskCom = Task(taskName)
        taskCom.add(featureName)
        gainCom = GainAdaptive('gain'+taskName)
        gainCom.setConstant(ingain)
        plug(gainCom.gain, taskCom.controlGain)
        plug(taskCom.error, gainCom.error)    
        return (featureCom, featureComDes, taskCom, gainCom)

    def createOperationalPointFeatureAndTask(self,
					 dynamicTmp,
	                                 operationalPointName,
	                                 featureName,
	                                 taskName,
	                                 ingain = .2):

        jacobianName = 'J{0}'.format(operationalPointName)
        dynamicTmp.signal(operationalPointName).recompute(0)
        dynamicTmp.signal(jacobianName).recompute(0)
        feature = \
           FeaturePosition(featureName,
	                   dynamicTmp.signal(operationalPointName),
	                   dynamicTmp.signal(jacobianName),
     	                   dynamicTmp.signal(operationalPointName).value)
        task = Task(taskName)
        task.add(featureName)
        gain = GainAdaptive('gain'+taskName)
        gain.setConstant(ingain)
        plug(gain.gain, task.controlGain)
        plug(task.error, gain.error)  
        return (feature, task, gain)

    def createDynamic(self,state,name) :
	# Create dynamic
        self.dynamicTmp = self.robot.loadModelFromJrlDynamics(
                              self.robot.name + name, 
                              self.robot.modelDir, 
                              self.robot.modelName,
                              self.robot.specificitiesPath,
                              self.robot.jointRankPath,
                              DynamicHrp2_14)
        self.dynamicTmp.dimension = self.dynamicTmp.getDimension()
        if self.dynamicTmp.dimension != len(self.robot.halfSitting):
            raise RuntimeError("Dimension of half-sitting: {0} differs from dimension of robot: {1}".format (len(self.halfSitting), self.dynamicTmp.dimension))

	# Pluging position
	plug(state, self.dynamicTmp.position)

	self.derivative=True

	# Pluging velocity
	self.robot.enableVelocityDerivator = self.derivative
 	if self.robot.enableVelocityDerivator:
            self.dynamicTmp.velocityDerivator = Derivator_of_Vector('velocityDerivator')
            self.dynamicTmp.velocityDerivator.dt.value = self.robot.timeStep
            plug(state, self.dynamicTmp.velocityDerivator.sin)
            plug(self.dynamicTmp.velocityDerivator.sout, self.dynamicTmp.velocity)
        else:
            self.dynamicTmp.velocity.value = self.dynamicTmp.dimension*(0.,)

	# Pluging acceleration
	self.robot.enableAccelerationDerivator = self.derivative
        if self.robot.enableAccelerationDerivator:
            self.dynamicTmp.accelerationDerivator = Derivator_of_Vector('accelerationDerivator')
            self.dynamicTmp.accelerationDerivator.dt.value = self.robot.timeStep
            plug(self.dynamicTmp.velocityDerivator.sout, self.dynamicTmp.accelerationDerivator.sin)
            plug(self.dynamicTmp.accelerationDerivator.sout, self.dynamicTmp.acceleration)
        else:
            self.dynamicTmp.acceleration.value = self.dynamicTmp.dimension*(0.,)

#        # --- center of mass ------------
#        (self.featureCom, self.featureComDes, self.taskCom, self.gainCom) = \
#            self.createCenterOfMassFeatureAndTask\
#            (self.dynamicTmp, '{0}_feature_com'.format(self.robot.name),
#             '{0}_feature_ref_com'.format(self.robot.name),
#             '{0}_task_com'.format(self.robot.name))

        # --- operational points tasks -----
        self.robot.features = dict()
        self.robot.tasks = dict()
        self.robot.gains = dict()
        for op in self.robot.OperationalPoints:
	    opName= op + name
            self.dynamicTmp.createOpPoint(op, op)
	    (self.robot.features[opName], self.robot.tasks[opName], self.robot.gains[opName]) = \
                self.createOperationalPointFeatureAndTask(self.dynamicTmp, op, 
		    '{0}_feature_{1}'.format(self.robot.name, opName),
                    '{0}_task_{1}'.format(self.robot.name, opName))
            # define a member for each operational point
            w = op.split('-')
            memberName = w[0]
            for i in w[1:]:
                memberName += i.capitalize()
            setattr(self, memberName, self.robot.features[opName])

#        self.robot.tasks ['com'] = self.taskCom
#        self.robot.features ['com']  = self.featureCom
#        self.robot.gains['com'] = self.gainCom
      
        self.robot.features['waist'+name].selec.value = '011100'

	return self.dynamicTmp

