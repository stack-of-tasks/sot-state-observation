from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint
from dynamic_graph.sot.core.meta_tasks_kine import *
from numpy import *
from numpy.linalg import inv,pinv,norm
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate,matrixToRPY
import dynamic_graph.sot.core.matrix_util 
rpyToMatrix = matrix_util.RPYToMatrix
from dynamic_graph.sot.core.meta_tasks_kine_relative import gotoNdRel, MetaTaskKine6dRel
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.feature_vector3 import FeatureVector3
from dynamic_graph.sot.dyninv import SolverKine
from dynamic_graph.tracer_real_time import *
SolverKine.toList = lambda sot: map(lambda x: x[1:-1],sot.dispStack().split('|')[1:])


def rtToHomo(R,T=None):
    M=eye(4)
    M[0:3,0:3]=R
    if T!=None: M[0:3,3]=T
    return matrix(M)

class CompensaterApplication:

    threePhaseScrew = True
    tracesRealTime = True

    def __init__(self,robot):

        self.robot = robot
        plug(self.robot.dynamic.com,self.robot.device.zmp)

        self.initGeneralApplication()
        self.sot = self.solver.sot

        self.createTasks()
        self.initTasks()
        self.initTaskGains()

        self.initSolver()
        self.initialStack()

    def initGeneralApplication(self):
        from dynamic_graph.sot.application.velocity.precomputed_meta_tasks import initialize
        self.solver = initialize(self.robot)


    # --- TASKS --------------------------------------------------------------------
    # --- TASKS --------------------------------------------------------------------
    # --- TASKS --------------------------------------------------------------------
    def createTasks(self):

        self.taskGaze    = MetaTaskVisualPoint('gaze',self.robot.dynamic,'head','gaze')
        self.contactRF   = self.robot.contactRF
        self.contactLF   = self.robot.contactLF
        self.taskCom     = self.robot.mTasks['com']
        self.taskLim     = self.robot.taskLim
        self.taskRH      = self.robot.mTasks['rh']
        self.taskLH      = self.robot.mTasks['lh']
        self.taskChest   = self.robot.mTasks['chest']
        self.taskPosture = self.robot.mTasks['posture']
        self.taskGripper = MetaTaskKinePosture(self.robot.dynamic,'gripper')
        self.taskHalfStitting = MetaTaskKinePosture(self.robot.dynamic,'halfsitting')
        self.taskCompensate = MetaTaskKine6d('compensate',self.robot.dynamic,'rh','right-wrist')

    def openGripper(self):
        self.taskGripper.gotoq(None,rhand=(self.gripperOpen,),lhand=(self.gripperOpen,))

    def closeGripper(self):
        self.taskGripper.gotoq(None,rhand=(self.gripperClose,),lhand=(self.gripperClose,))

    #initialization is separated from the creation of the tasks because if we want to switch
    #to second order controlm the initialization will remain while the creation is 
    #changed

    def initTasks(self):
        self.initTaskContact()
        self.initTaskBalance()
        self.initTaskPosture()
        self.initTaskHalfSitting()
        self.initTaskCompensate()

    def initTaskHalfSitting(self):
        self.taskHalfStitting.gotoq(None,self.robot.halfSitting)

    def initTaskContact(self):
        # --- CONTACTS
        self.contactRF.feature.frame('desired')
        self.contactLF.feature.frame('desired')

    def initTaskBalance(self):
        # --- BALANCE ---
        self.taskChest.feature.frame('desired')
        self.taskChest.feature.selec.value = '011000'
        
        ljl = matrix(self.robot.dynamic.lowerJl.value).T
        ujl = matrix(self.robot.dynamic.upperJl.value).T
        ljl[9] = 0.65
        ljl[15] = 0.65
        self.taskLim.referenceInf.value = vectorToTuple(ljl)
        self.taskLim.referenceSup.value = vectorToTuple(ujl)

    def initTaskPosture(self):
        # --- LEAST NORM
        weight_ff        = 0
        weight_leg       = 3
        weight_knee      = 5
        weight_chest     = 1
        weight_chesttilt = 10
        weight_head      = 0.3
        weight_arm       = 1

        weight = diag( (weight_ff,)*6 + (weight_leg,)*12 + (weight_chest,)*2 + (weight_head,)*2 + (weight_arm,)*14)
        weight[9,9] = weight_knee
        weight[15,15] = weight_knee
        weight[19,19] = weight_chesttilt
        weight = weight[6:,:]

        self.taskPosture.task.jacobian.value = matrixToTuple(weight)
        self.taskPosture.task.task.value = (0,)*weight.shape[0]

    def initTaskGains(self, setup = "medium"):
        if setup == "medium":
            self.contactRF.gain.setConstant(10)
            self.contactLF.gain.setConstant(10)
            self.taskChest.gain.setConstant(10)
            self.taskRH.gain.setByPoint(2,0.8,0.01,0.8)
            self.taskCompensate.gain.setByPoint(2,0.8,0.01,0.8)
            self.taskHalfStitting.gain.setByPoint(2,0.8,0.01,0.8)
         
    # --- SOLVER ----------------------------------------------------------------

    def initSolver(self):
        plug(self.sot.control,self.robot.device.control)

    def push(self,task,keep=False):
        if isinstance(task,str): taskName=task
        elif "task" in task.__dict__:  taskName=task.task.name
        else: taskName=task.name
        if taskName not in self.sot.toList():
            self.sot.push(taskName)
        if taskName!="taskposture" and "taskposture" in self.sot.toList():
            self.sot.down("taskposture")
        if keep: task.keep()

    def rm(self,task):
        if isinstance(task,str): taskName=task
        elif "task" in task.__dict__:  taskName=task.task.name
        else: taskName=task.name
        if taskName in self.sot.toList(): self.sot.rm(taskName)

    # --- DISPLAY -------------------------------------------------------------
    def initDisplay(self):
        self.robot.device.viewer.updateElementConfig('red',[0,0,-1,0,0,0])
        self.robot.device.viewer.updateElementConfig('yellow',[0,0,-1,0,0,0])

    def updateDisplay(self):
        '''Display the various objects corresponding to the calcul graph. '''
        None

    # --- TRACES -----------------------------------------------------------
    def withTraces(self):
        if self.tracesRealTime:
            self.robot.initializeTracer()
        else:
            self.robot.tracer = Tracer('trace')
            self.robot.device.after.addSignal('{0}.triger'.format(self.robot.tracer.name))
        self.robot.tracer.open('/tmp/','','.dat')
        #self.robot.tracer.add( self.taskRH.task.name+'.error','erh' )
        self.robot.startTracer()
        
    def stopTraces(self):
        self.robot.stopTracer()

    def trace(self):
        self.robot.tracer.dump()

    # --- RUN --------------------------------------------------------------
    def initialStack(self):
        self.sot.clear()
        self.push(self.contactLF,True)
        self.push(self.contactRF,True)
        self.push(self.taskLim)
        self.push(self.taskCom)
        self.push(self.taskChest,True)
        self.push(self.taskPosture)

    def moveToInit(self):
        '''Go to initial pose.'''
        gotoNd(self.taskRH,(0.3,-0.2,1.1,0,-pi/2,0),'111001')
        self.push(self.taskRH)
        None

    


    def goHalfSitting(self):
        '''End of application, go to final pose.'''
        self.sot.clear()
        self.push(self.contactLF,True)
        self.push(self.contactRF,True)
        self.push(self.taskLim)
        self.push(self.taskHalfStitting)

    # --- SEQUENCER ---
    seqstep = 0
    def nextStep(self,step=None):
        if step!=None: self.seqstep = step
        if self.seqstep==0:
            self.moveToInit()
        elif self.seqstep==1:
            self.startCompensate()
        elif self.seqstep==2:
            self.goHalfSitting()
        self.seqstep += 1
        
    def __add__(self,i):
        self.nextStep()


    # COMPENATION ######################################

    def initTaskCompensate(self):
        # The constraint is:
        #    cMhref !!=!! cMh = cMcc ccMh
        # or written in ccMh
        #    ccMh !!=!! ccMc cMhref

        # c : central frame of the robot
        # cc : central frame for the controller  (without the flexibility)
        # cMcc= flexibility
        # ccMc= flexibility inverted

        # href : the reference 'desired' position of the hand
        self.ccMcBox = MatrixToHomo ("ccMc")
        self.ccMc = self.ccMcBox.sin
        
        self.ccMhref = Multiply_of_matrixHomo('ccMhref')
        plug(self.ccMcBox.sout, self.ccMhref.sin1)

        self.cMhref = self.ccMhref.sin2
        # You need to set up the inverted flexibility : plug( ..., self.ccMc)
        # You need to set up a reference value here: plug( ... ,self.cMhref)

        

        plug(self.ccMhref.sout,self.taskCompensate.featureDes.position)

        self.taskCompensate.feature.frame('desired')

    def startCompensate(self):
        '''Start to compensate for the hand movements.'''
        
     
        self.cMhref.value = self.robot.dynamic.rh.value
        print matrix(self.cMhref.value)
        
        self.rm(self.taskRH)
        self.push(self.taskCompensate)

   




