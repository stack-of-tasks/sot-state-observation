# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from numpy import *
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb

from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector, MatrixHomoToPose
from dynamic_graph.sot.application.state_observation import DGIMUModelBaseFlexEstimation, PositionStateReconstructor, InputReconstructor, EstimatorInterface, DriftFromMocap, FromLocalToGlobalFrame

from dynamic_graph.sot.core.derivator import Derivator_of_Vector

from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.application.state_observation import Calibrate

from dynamic_graph.ros import RosExport

def initDevice(robot):
	robot.device.state.value=robot.halfSitting
	robot.device.velocity.value=(0.,)*36
	robot.device.forceLLEG.value=(1.8349814919184242, -7.4412430930486302, 256.06853454222494, -0.035428813912447302, 2.0798475785647286, 0.14169701504511384)
	robot.device.forceRLEG.value=(2.4303733340459406, 11.156361786170869, 285.59013529212666, -0.69957871247984049, 2.0516111892090887, -0.22872430884228223)
	robot.device.accelerometer.value=(0.12527866711822, 0.089756740219665537, 9.8059788372472152)
	robot.device.gyrometer.value=(-0.0029326257213877862, 0.007655425240526083, -8.001571126249214e-05)
	robot.device.forceLARM.value=(-2.04071, 3.25524, -5.89116, 0.0814646, 0.0779619, 0.0190317)
	robot.device.forceRARM.value=(2.07224, -9.57906, 0.69111, -0.415802, 0.289026, 0.00621748)

def computeDynamic(robot,i):
	robot.dynamic.chest.recompute(i)
	robot.dynamic.com.recompute(i)
	robot.dynamic.Jcom.recompute(i)
	robot.dynamic.angularmomentum.recompute(i)
	robot.dynamic.inertia.recompute(i)
	robot.dynamic.waist.recompute(i)

class FromLocalToGLobalFrame(FromLocalToGlobalFrame):
    def __init__(self, estimator, name='FromLocalToGLobalFrame'):
	FromLocalToGlobalFrame.__init__(self, name)
	self.estimator = estimator
	plug(self.estimator.state, self.flexState)	

class HRP2ModelBaseFlexEstimatorIMUForce(DGIMUModelBaseFlexEstimation):

    def __init__(self, robot, name='flextimator', useMocap=True, dt=0.005):
        DGIMUModelBaseFlexEstimation.__init__(self,name)
        self.setSamplingPeriod(dt)  
        self.robot = robot

	initDevice(self.robot)
	computeDynamic(self.robot,0)

        # Covariances
        self.setProcessNoiseCovariance(matrixToTuple(np.diag((1e-8,)*12+(1e-4,)*3+(1e-4,)*3+(1e-4,)*3+(1e-4,)*3+(1.e-2,)*6+(1e-15,)*2+(1.e-8,)*3)))
        self.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-3,)*3+(1e-6,)*3))) 
        self.setUnmodeledForceVariance(1e-13)
        self.setForceVariance(1e-4)
        self.setAbsolutePosVariance(1e-4)

        # Contact model definition
        # self.setKfe(matrixToTuple(np.diag((40000,40000,40000))))
        self.setKfe(matrixToTuple(np.diag((150000,150000,150000))))
        self.setKfv(matrixToTuple(np.diag((600,600,600))))
        self.setKte(matrixToTuple(np.diag((600,600,600))))
        self.setKtv(matrixToTuple(np.diag((10,10,10))))

        self.setKfeRopes(matrixToTuple(np.diag((10000,10000,10000))))
        self.setKfvRopes(matrixToTuple(np.diag((300,300,800))))
        self.setKteRopes(matrixToTuple(np.diag((600,600,600))))
        self.setKtvRopes(matrixToTuple(np.diag((60,60,60))))

        # Estimator interface
        self.interface=EstimatorInterface(name+"EstimatorInterface")
	self.interface.setSamplingPeriod(dt)
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
	self.setPe(self.Peg)
        self.Prl1 = np.matrix([[1,0,0,0-3.19997004e-02],[0,1,0,0.15-0],[0,0,1,1.28-1],[0,0,0,1]]) # Positions of the contacts on the robot (in the local frame) with respect to the chest
        self.Prl2 = np.matrix([[1,0,0,0-3.19997004e-02],[0,1,0,-0.15-0],[0,0,1,1.28-1],[0,0,0,1]])
        (self.contact1OpPoint,self.contact1Pos,self.contact1)=self.createContact('contact1', self.Prl1)
        (self.contact2OpPoint,self.contact2Pos,self.contact2)=self.createContact('contact2', self.Prl2)
        plug(self.contact1.sout,self.interface.position_ls)
        plug(self.contact2.sout,self.interface.position_rs)

        # Contacts model and config
        plug(self.interface.contactsModel,self.contactsModel)
        self.setWithConfigSignal(True)
        plug(self.interface.config,self.config)

        if(useMocap):     
            # Mocap signal
            self.ros = RosExport('rosExportMocap')
            self.ros.add('matrixHomoStamped', "chest", "/evart/hrp2_14_head/hrp2_14_head")
            # Filtering
            from dynamic_graph.sot.tools import MocapDataFilter
            self.mocapFilter = MocapDataFilter('MocapDataFilter')
            plug(self.ros.signal('chest'),self.mocapFilter.sin)
            self.mocapSignal =  self.mocapFilter.sout
            
            # Drift
            self.drift = DriftFromMocap(name+'Drift')
            plug(self.mocapSignal,self.drift.limbGlobal)
            plug(self.robot.dynamic.chest,self.drift.limbLocal)
            self.drift.init()
            plug(self.drift.driftInvVector,self.interface.drift)

        # Measurement reconstruction
        plug(self.robot.device.accelerometer,self.interface.accelerometer)
        plug(self.robot.device.gyrometer,self.interface.gyrometer)
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
	self.IMUVector.setSamplingPeriod(dt)
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
	self.comVector.setSamplingPeriod(dt)
        plug(self.comVectorIn.sout,self.comVector.sin)
        self.comVector.inputFormat.value  = '000101'
        self.comVector.outputFormat.value = '010101'  
        self.comVector.setFiniteDifferencesInterval(1)
        self.DCom.sout.recompute(0)
        self.comVector.setLastVector(self.com.value+(0.,)*15)#(0.,)*3+self.DCom.sout.value+(0.,)*9)

        # Compute derivative of Angular Momentum
        self.angMomDerivator = Derivator_of_Vector(name+'angMomDerivator')
        plug(self.robot.dynamic.angularmomentum,self.angMomDerivator.sin)
        self.angMomDerivator.dt.value = dt

#        self.angMomDerivator = PositionStateReconstructor (name+'angMomDerivator')
#	 self.angMomDerivator.setSamplingPeriod(dt)
#        plug(self.robot.dynamic.angularmomentum,self.angMomDerivator.sin)
#        self.angMomDerivator.inputFormat.value  = '000001'
#        self.angMomDerivator.outputFormat.value = '000100'  
#     self.angMomDerivator.setFiniteDifferencesInterval(2)
#     self.robot.dynamic.angularmomentum.recompute(0)
#     self.angMomDerivator.setLastVector(self.robot.dynamic.angularmomentum.value+(0.,)*15)       
        
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


    def createContact(self,name, prl):
        self.contactOpPoint = OpPointModifier(name+'_opPoint')
        self.contactOpPoint.setEndEffector(False)
        self.contactOpPoint.setTransformation(matrixToTuple(prl))
        plug (self.robot.dynamic.chest,self.contactOpPoint.positionIN)
    
        self.contactPos = MatrixHomoToPose(name+'_pos')
        plug(self.contactOpPoint.position, self.contactPos.sin)
    
        self.contact = Stack_of_vector (name)
        plug(self.contactPos.sout,self.contact.sin1)
        self.contact.sin2.value = (0,0,0)
        self.contact.selec1 (0, 3)
        self.contact.selec2 (0, 3)

        return (self.contactOpPoint,self.contactPos,self.contact)


