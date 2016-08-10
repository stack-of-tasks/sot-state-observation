# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from numpy import *
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb

from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector, MatrixHomoToPose
from dynamic_graph.sot.application.state_observation import DGIMUModelBaseFlexEstimation, PositionStateReconstructor, InputReconstructor

from dynamic_graph.sot.core.matrix_util import matrixToTuple

from dynamic_graph.sot.core import Multiply_of_matrixHomo, Inverse_of_matrixHomo

from dynamic_graph.ros import RosExport

from dynamic_graph.sot.tools import MocapDataFilter

class HRP2MocapToSOT:
  def __init__(self,robot):
    self.robot = robot
    self.ros = RosExport('rosExportMocap')
    self.ros.add('matrixHomoStamped', "chest", "/evart/hrp2_head_sf/hrp2_head_sf")

    self.mocapFilter = MocapDataFilter('MocapDataFilter')
    plug(self.ros.signal('chest'),self.mocapFilter.sin)
    self.mocapSignal =  self.mocapFilter.sout
  
  def initialize(self):
    
    m = np.matrix([[-0.8530, 0.5208,  -0.0360,  0.0049],[ -0.5206, -0.8537,  -0.0146,  -0.0078],[  -0.0384,   0.0063,    0.9993 ,   0.2158],
         [0     ,    0      ,   0  ,  1.0000]])#transformation from the chest-markers frame to the chest frame
    
    self.chestMarkersSot = OpPointModifier('chestMarkersSot')# gets the position of the chest-markers frame in sot frame
    self.chestMarkersSot.setEndEffector(False)
    self.chestMarkersSot.setTransformation(matrixToTuple(m))
    plug (self.robot.dynamic.chest, self.chestMarkersSot.positionIN)
      
    self.chestMarkersSot.position.recompute(1)
    self.mchestsot = np.matrix(self.chestMarkersSot.position.value)
    self.mchestmocap = np.matrix(self.ros.signal('chest').value)
    self.mtransformMocap2ISot = np.linalg.inv(self.mchestsot) * self.mchestmocap #transforms the mocap frame to the initial-sot frame
    
    self.chestMarkerMocapInISot = Multiply_of_matrixHomo("chestMarkerMocap")#gets position of chest-markers frame in sot-initial frame
    self.chestMarkerMocapInISot.sin1.value = matrixToTuple(self.mtransformMocap2ISot)
    plug(self.mocapSignal,self.chestMarkerMocapInISot.sin2)
    
    self.chestMarkersSotInverse = Inverse_of_matrixHomo ("chestMarkersSotInverse")#inverse of the homomatrix for position of markers in local sot
    plug(self.chestMarkersSot.position, self.chestMarkersSotInverse.sin)
    
    self.robotPositionISot = Multiply_of_matrixHomo("robotPositionInSot")
    plug(self.chestMarkerMocapInISot.sout,self.robotPositionISot.sin1)
    plug(self.chestMarkersSotInverse.sout,self.robotPositionISot.sin2)
    
    self.robotPositionInMocap = Multiply_of_matrixHomo("robotPositionInMocap")
    plug(self.mocapSignal,self.robotPositionInMocap.sin1)
    plug(self.chestMarkersSotInverse.sout,self.robotPositionInMocap.sin2)
    
    
    
