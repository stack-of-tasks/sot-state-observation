# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from numpy import *
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb

from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector, MatrixHomoToPose, Selec_of_vector 
from dynamic_graph.sot.application.state_observation import DGIMUModelBaseFlexEstimation, PositionStateReconstructor, InputReconstructor

from dynamic_graph.sot.core.derivator import Derivator_of_Vector

from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.application.state_observation import Calibrate

from dynamic_graph.sot.hrp2.dynamic_hrp2_14 import DynamicHrp2_14

class HRP2ModelBaseFlexEstimatorIMUForceEncoders(DGIMUModelBaseFlexEstimation):
    def __init__(self, robot, name='flextimator2'):
        DGIMUModelBaseFlexEstimation.__init__(self,name)
        
        self.setSamplingPeriod(0.005)  
        self.robot = robot

        self.createDynamicEncoders()

        self.robot.dynamicEncoders.inertia.recompute(1)					      
        self.robot.dynamicEncoders.waist.recompute(1)	

        # Sensors stack
        self.sensorStackimu = Stack_of_vector (name+'SensorsIMU')
        plug(self.robot.device.accelerometer,self.sensorStackimu.sin1)
        plug(self.robot.device.gyrometer,self.sensorStackimu.sin2)
        self.sensorStackimu.selec1 (0, 3)
        self.sensorStackimu.selec2 (0, 3)
	self.calibration= Calibrate('calibration')
	plug(self.sensorStackimu.sout,self.calibration.imuIn)
	plug(self.robot.dynamicEncoders.com,self.calibration.comIn)			

        self.sensorStackforce = Stack_of_vector (name+'SensorsFORCE')
        plug(self.robot.device.forceLLEG,self.sensorStackforce.sin1)
        plug(self.robot.device.forceRLEG,self.sensorStackforce.sin2)
        self.sensorStackforce.selec1 (0, 6)
        self.sensorStackforce.selec2 (0, 6)

        self.sensorStack = Stack_of_vector (name+'Sensors')
        plug(self.calibration.imuOut,self.sensorStack.sin1)
        plug(self.sensorStackforce.sout,self.sensorStack.sin2)
        self.sensorStack.selec1 (0, 6)
        self.sensorStack.selec2 (0, 12)

	plug(self.sensorStack.sout,self.measurement);
        self.contactForces = self.sensorStack.sin2
      
        # Input reconstruction

		# IMU Vector
        self.inputPos = MatrixHomoToPoseUTheta(name+'InputPosition')
        plug(robot.frames['accelerometer'].position,self.inputPos.sin)
        self.robot.dynamicEncoders.createJacobian(name+'ChestJ_OpPoint','chest')	
        self.imuOpPoint = OpPointModifier(name+'IMU_oppoint')
        self.imuOpPoint.setEndEffector(False)
        self.imuOpPoint.setTransformation(matrixToTuple(np.linalg.inv(np.matrix(self.robot.dynamicEncoders.chest.value))*np.matrix(self.robot.frames['accelerometer'].position.value)))
        plug (self.robot.dynamicEncoders.chest,self.imuOpPoint.positionIN)			
        plug (self.robot.dynamicEncoders.signal(name+'ChestJ_OpPoint'),self.imuOpPoint.jacobianIN)
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

        	# CoM and derivatives
        self.com=self.robot.dynamicEncoders.com
        self.DCom = Multiply_matrix_vector(name+'DCom')
        plug(self.robot.dynamicEncoders.Jcom,self.DCom.sin1)
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

		# Compute derivative of Angular Momentum
        self.angMomDerivator = Derivator_of_Vector('angMomDerivator')
        plug(self.robot.dynamicEncoders.angularmomentum,self.angMomDerivator.sin)
        self.angMomDerivator.dt.value = self.robot.timeStep          
        
        	# Concatenate with InputReconstructor entity
        self.inputVector=InputReconstructor(name+'inputVector')
        plug(self.comVector.sout,self.inputVector.comVector)
        plug(self.robot.dynamicEncoders.inertia,self.inputVector.inertia)
	plug(self.robot.dynamicEncoders.angularmomentum,self.inputVector.angMomentum)
	plug(self.angMomDerivator.sout,self.inputVector.dangMomentum)
        self.inputVector.dinertia.value=(0,0,0,0,0,0)
        plug(self.robot.dynamicEncoders.waist,self.inputVector.positionWaist)
        plug(self.IMUVector.sout,self.inputVector.imuVector)
        plug(self.contactNbr,self.inputVector.nbContacts)

        self.inputVector.setSamplingPeriod(robot.timeStep)
        self.inputVector.setFDInertiaDot(True)     
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


    # Robot real dynamics ######################################

    def initializeOpPoints(self, model):
        for op in self.robot.OperationalPoints:
            model.createOpPoint(op, op)

    def createDynamicEncoders(self):
	# Create dynamic
        self.robot.dynamicEncoders = self.robot.loadModelFromJrlDynamics(
                              self.robot.name + '_dynamicEncoders', 
                              self.robot.modelDir, 
                              self.robot.modelName,
                              self.robot.specificitiesPath,
                              self.robot.jointRankPath,
                              DynamicHrp2_14)
        self.dimension = self.robot.dynamicEncoders.getDimension()
        if self.dimension != len(self.robot.halfSitting):
            raise RuntimeError("Dimension of half-sitting: {0} differs from dimension of robot: {1}".format (len(self.halfSitting), self.dimension))

	# Pluging position
	self.robot.device.robotState.value=46*(0.,)
	self.encodersState = Selec_of_vector('encodersState')
	self.encodersState.selec(0,36)
	plug(self.robot.device.robotState,self.encodersState.sin)
        plug(self.encodersState.sout, self.robot.dynamicEncoders.position)

	# Pluging velocity
 	if self.robot.enableVelocityDerivator:
            self.velocityDerivator = Derivator_of_Vector('velocityDerivator')
            self.velocityDerivator.dt.value = self.robot.timeStep
            plug(self.robot.device.robotState, self.velocityDerivator.sin)
            plug(self.velocityDerivator.sout, self.robot.dynamicEncoders.velocity)
        else:
            self.robot.dynamicEncoders.velocity.value = self.dimension*(0.,)

	# Pluging acceleration
        if self.robot.enableAccelerationDerivator:
            self.accelerationDerivator = \
                Derivator_of_Vector('accelerationDerivator')
            self.accelerationDerivator.dt.value = self.timeStep
            plug(self.velocityDerivator.sout,
                 self.accelerationDerivator.sin)
            plug(self.accelerationDerivator.sout, self.robot.dynamicEncoders.acceleration)
        else:
            self.robot.dynamicEncoders.acceleration.value = self.dimension*(0.,)

	self.initializeOpPoints(self.robot.dynamicEncoders)



