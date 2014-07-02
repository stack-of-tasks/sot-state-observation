# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
#from numpy import *
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb

from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector, MatrixHomoToPose
from dynamic_graph.sot.application.state_observation import DGIMUModelBaseFlexEstimation, PositionStateReconstructor, InputReconstructor

from dynamic_graph.sot.core.matrix_util import matrixToTuple


class HRP2ModelBaseFlexEstimator(DGIMUModelBaseFlexEstimation):
    def __init__(self, robot, name='flextimator1'):   
        DGIMUModelBaseFlexEstimation.__init__(self,name)
        self.setSamplingPeriod(0.005)  
        self.robot = robot

        # Definition of IMU vector
        self.sensorStack = Stack_of_vector (name+'Sensors')
        plug(self.robot.device.accelerometer,self.sensorStack.sin1)
        plug(self.robot.device.gyrometer,self.sensorStack.sin2)
        self.sensorStack.selec1 (0, 3)
        self.sensorStack.selec2 (0, 3)
        plug(self.sensorStack.sout,self.measurement);       
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
        self.robot.dynamic.inertia.recompute(1)
        self.inertia=self.robot.dynamic.inertia
        self.dotInertia=(0,0,0,0,0,0)
        self.angMomentum=(0,0,0)
        self.dotAngMomentum=(0,0,0) 


        # Definition of com and derivatives
        self.com=self.robot.dynamic.com#(0,0,0.75) # /!\ In the local frame!
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
        self.comVector.inputFormat.value  = '000011'
        self.comVector.outputFormat.value = '000111'
        self.inputReconstruction=InputReconstructor(name+'InputReconstruction')
        plug(self.comVector.sout,self.inputReconstruction.comVector)
        plug(self.inertia,self.inputReconstruction.inertia)
        self.inputReconstruction.dinertia.value=self.dotInertia
        self.inputReconstruction.angMomentum.value=self.angMomentum
        self.inputReconstruction.dangMomentum.value=self.dotAngMomentum
        plug(self.IMUVector.sout,self.inputReconstruction.imuVector)


        # Definition of contacts position
#        self.rFootPos = MatrixHomoToPose('rFootFrame')
#        self.lFootPos = MatrixHomoToPose('lFootFrame')
#        plug(self.robot.frames['rightFootForceSensor'].position,self.rFootPos.sin)
#        plug(self.robot.frames['leftFootForceSensor'].position,self.lFootPos.sin)
#        self.a=float('NaN')
#        self.contactn3=(0,0,0)
#        self.contactn4=(0,0,0)
#        self.contacts = Stack_of_vector (name+'contacts')
#        plug(self.rFootPos.sout,self.contacts.sin1)
#        plug(self.lFootPos.sout,self.contacts.sin2)
#        self.contacts.selec1 (0, 3)
#        self.contacts.selec2 (0, 3)
#        self.otherContacts=self.contactn3+self.contactn4 
#        self.allcontacts = Stack_of_vector (name+'allcontacts')
#        plug(self.contacts.sout,self.allcontacts.sin1)
#        self.allcontacts.sin2.value=self.otherContacts
#        self.allcontacts.selec1 (0, 6)
#        self.allcontacts.selec2 (0, 6)
        
     
        # Construction of the input vector       
        self.inputVector = Stack_of_vector (name+'inputVector')
        plug(self.inputReconstruction.input,self.inputVector.sin1)
        #plug(self.allcontacts.sout,self.inputVector.sin2)
        self.inputVector.selec1 (0, 42)
        #self.inputVector.selec2 (0, 12)  
        
        
        #plug(self.inputVector.sout,self.input)
        self.robot.flextimator = self
