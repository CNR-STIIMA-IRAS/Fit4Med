# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '.\MovementWindow.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from json import load
import os
import sys
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QMessageBox
from .MovementWindow import Ui_MovementWindow
import yaml
from yaml.loader import SafeLoader
import numpy as np
from scipy import interpolate
from copy import deepcopy
#import FMRRMainProgram as FMRRMain

#MC Classes/methods
from . import MC_Tools

#ROS
from builtin_interfaces.msg import Duration
#import rospkg
from geometry_msgs.msg import Point
#from StringIO import StringIO


class FMRR_Ui_MovementWindow(Ui_MovementWindow):
    def __init__(self) -> None:
        super().__init__()

    
##############################################################################################################
#####                                                                                                    #####  
#####                                        CREATE TRAJECTORY                                           ##### 
#####                                                                                                    #####
#####                                                                                                    #####
##############################################################################################################            
#
    def clbk_BtnCreateMovementData(self):
        
        # Create a Dialogn Button to ask to manually guide the robot to end position
        MyString = "Go to the desired final position"
        question = QMessageBox(QMessageBox.Warning, "Manual Guidance", MyString)
        stop_button = question.addButton("Stop", QMessageBox.RejectRole)
        start_button = question.addButton("Start", QMessageBox.AcceptRole)
        decision = question.exec_()
        # You can check which button was pressed:
        if question.clickedButton() == start_button:
            # Start Admittance controller
            pass
        elif question.clickedButton() == stop_button:
            # Stop admittance and load scaled
            pass        
        
        # Get the end position of the handle
        _toolPosCovFact = self.ui_FMRRMainWindow._toolPosCovFact        
        HandlePosition = self.ui_FMRRMainWindow.HandlePosition
        RobotJointPosition = self.ui_FMRRMainWindow.RobotJointPosition# 
        
        # save data to create movement
        self.End_HandlePosition = deepcopy( self.ui_FMRRMainWindow.HandlePosition )
        self.End_RobotJointPosition = RobotJointPosition
        
        # visualize data            
        HandlePosition[:] = [iPosition * _toolPosCovFact for iPosition in HandlePosition]
        self.lcdNumber_EndPos_X.display( int( HandlePosition[0] ) ) 
        self.lcdNumber_EndPos_Y.display( int( HandlePosition[1] ) ) 
        self.lcdNumber_EndPos_Z.display( int( HandlePosition[2] ) )
        self.pushButton_CREATEMovement.enablePushButton(1)
        
        # create trajectory
        MovementsPath = self.ui_FMRRMainWindow.FMRR_Paths['Movements']
        
        if self.radioButton_SideLeft.isChecked() == True:
            self.SideOfMovement = 1
        elif self.radioButton_SideRight.isChecked() == True :
            self.SideOfMovement = 2
        else:
            self.SideOfMovement = 0
              
        if self.radioButton_TypeOfExercise_Reaching.isChecked() == True:
            self.TypeOfMovement = 1
        elif self.radioButton_TypeOfExercise_HandtoMouth.isChecked() == True:
            self.TypeOfMovement = 2
        else:
            self.TypeOfMovement = 0

        if self.radioButton_VelocityProfileConstant.isChecked() == True:
            self.vel_profile = 1    
            print('1')
        elif self.radioButton_VelocityProfileBellshaped.isChecked() == True:
            self.vel_profile = 2
            print('2')
        else:
            self.vel_profile = 0
            print('0')
           
        # Set start and end positions 
        x_begin = 0   
        y_begin = 0
        z_begin = 0
        x_end = deepcopy(self.End_HandlePosition[0])
        y_end = deepcopy(self.End_HandlePosition[1])
        z_end = deepcopy(self.End_HandlePosition[2])
        
        x12 = x_end-x_begin; y12 = y_end-y_begin; z12 = z_end-z_begin
        L12 = np.sqrt( x12**2 + y12**2 + z12**2 )
        x1 = 0; y1 = 0; z1 = 0
        x2 = x12; y2 = y12; z2 = z12

        T = self.doubleSpinBox_MoveTime.value() # total movement time in sec 
        _time = np.linspace(0, T, int(T/0.008)+1, endpoint = True) # To satisfy Beschi's condition of having steps = 8ms
        print("_time")
        print(_time[0:4])
        numSamples = len(_time) # length of the final vectors, the ones given as results
        _numPoints = numSamples * 1000 #lenght of oversampled vectors
        v1 = np.zeros( (numSamples,), dtype = float, order='C' )    

        if self.TypeOfMovement == 1:  # Reaching (rectilinear trajectory)
            x = np.linspace( x1, x2, _numPoints)
            y = np.linspace( y1, y2, _numPoints )
            z = np.linspace( z1, z2, _numPoints )
            L = L12
            _ContinueCreateMovement = 1
        elif self.TypeOfMovement == 2:    # Hand to Mouth
            filename = QtWidgets.QFileDialog.getOpenFileName(None, "Load Movement", MovementsPath, "*.yaml")
            if bool(filename[0]):
                print('This is the filename of the loaded movement:')
                print(filename)
                _TrjYamlData = yaml.load(open(filename [0]), Loader=SafeLoader)
                MovementData = _TrjYamlData.get("cart_trj1").get("cart_positions")
                _originalNumPoints = len(MovementData)
                x1_LF = MovementData[0][0]; x2_LF = MovementData[-1][0]      #_LF = Loaded File   
                y1_LF = MovementData[0][1]; y2_LF = MovementData[-1][1]
                z1_LF = MovementData[0][2]; z2_LF = MovementData[-1][2]                
                x12_LF = x2_LF-x1_LF; y12_LF = y2_LF-y1_LF; z12_LF = z2_LF-z1_LF
                L12_LF = np.sqrt( x12_LF**2 + y12_LF**2 + z12_LF**2 ) # actually the starting point is [0,0,0]

                print('L12_LF:')
                print(L12_LF)
 #               _ConvPara = L12/L12_LF
                _ConvPara = z12/(z2_LF - z1_LF)
                print('_ConvPara:')
                print(_ConvPara)
 
                x_LF = np.zeros( (1, _originalNumPoints), dtype=float, order='C' ) 
                y_LF = np.zeros( (1, _originalNumPoints), dtype=float, order='C' )  
                z_LF = np.zeros( (1, _originalNumPoints), dtype=float, order='C' ) 
        
                for i in range(_originalNumPoints):
                    x_LF[0][i]  = MovementData[i][0] * _ConvPara
                    y_LF[0][i]  = MovementData[i][1] * _ConvPara
                    z_LF[0][i]  = MovementData[i][2] * _ConvPara

        #       First interpolation
                xyz = np.zeros( (3, len(MovementData) ), dtype=float, order='C' )
                # xyz = np.ndarray( 3,len(x) )

                xyz[0,:] = x_LF 
                xyz[1,:] = y_LF 
                xyz[2,:] = z_LF
        
                distances_2 = np.sum( np.diff( xyz, 1, 1 ) * np.diff( xyz, 1, 1 ), 0)          # distance^2 ; matlab  sum( diff( xyz,[],1 ).^2,2 )
                distances = np.sqrt( distances_2 )
                cum_distances = np.cumsum(distances)
                s1 = np.concatenate( ([0], cum_distances), axis = None)    #% Cumulative sum to calculate abscissa S  
                L1 = s1[-1]                                             # L1 = trajectory length
                interp_function_1 = interpolate.interp1d(s1, xyz, 'cubic')
                CSn_1 = np.linspace(0, L1, _numPoints/1000 )
                xyz_1 = interp_function_1( CSn_1 )
                print('L1:')
                print(L1)
                print('_numpoints')
                print(_numPoints)
                _numPoints2 = len( xyz_1)
                print('_numpoints2')
                print(_numPoints2)                
                
                # Second interpolation (needed to have equal dS (The first time the length of S may change if there are very distant points
                distances2_2 = np.sum( np.diff( xyz_1, 1, 1 ) * np.diff( xyz_1, 1, 1 ), 0)          # distance^2 ; matlab  sum( diff( xyz,[],1 ).^2,2 )
                distances2 = np.sqrt( distances2_2 )
                cum_distances2 = np.cumsum(distances2) 
                s2 = np.concatenate ( ([0], cum_distances2 ), axis = None)    #% Cumulative sum to calculate abscissa S  
                L = s2[-1]                                             # L = trajectory length
                print('L')
                print(L)                
#                interp_function_2 = interpolate.interp1d(s2, xyz_1, 'cubic')
                interp_function_2 = interpolate.interp1d(s2, xyz_1, 'cubic')
                CSn_2 = np.linspace( 0, L, _numPoints )
                XYZ = interp_function_2( CSn_2 ) 
                print('L:')
                print(L)
                x = XYZ[0,:]
                y = XYZ[1,:]
                z = XYZ[2,:]
                x_end = x_begin + float(x[-1])
                print( "x_end" )
                print( x_end )
                print( "x_end" )
                print( x_end )
                print(" type(x_begin)" )
                print( type(x_begin) )
                print( "type(x)" )
                print( type(x) )
                y_end = y_begin + float(y[-1])
                z_end = z_begin + float(z[-1])                
                print("self.SideOfMovement")
                print(self.SideOfMovement)
                if self.SideOfMovement == 1:
                    print("inverto per movimento sinistro")
                    x = - x
                    
                print('_numpoints')
                print(_numPoints)
                _numPoints2 = len(x)
                print('_numpoints2')
                print(_numPoints2)                
                
                _ContinueCreateMovement = 1    
            else:
                _ContinueCreateMovement = 0       

        if _ContinueCreateMovement == 1:
            if self.vel_profile == 1: #Constant velocity
                dt = .3 # acceleretion (and deceleration) time to reach maximum velocity (vel =  zero) in sec 
                Vmax = 1 / (T-dt) # S = 0.5*Vmax*dt + Vmax(T-2*dt) + 0.5*Vmax*dt) = 1 
                SlopeSamples = int(np.ceil( numSamples *  dt/T ) )
                v1 = np.zeros( numSamples, dtype=float, order='C' ) 
                v1[0:SlopeSamples] = np.linspace(0,Vmax,SlopeSamples)
                v1[SlopeSamples: -SlopeSamples] = Vmax*np.ones( (1,numSamples-2*SlopeSamples), dtype = float, order='C'  )
                v1[-SlopeSamples-1:-1] = np.linspace(Vmax,0,SlopeSamples)
            
            elif self.vel_profile == 2: # bell shaped velocity profile (sinus)
                Tg=2*T
                v1 = (np.pi / Tg) * np.sin( 2* np.pi/Tg * _time )
            else:
                pass     
    
#       %%% new abscissa calculation, and resampling
            abscissa = np.cumsum( 0.5 * ( v1[0:-1] + v1[1:] ) * np.diff(_time) ) # definition of integral 0.5*[V(i)+V(i+1))]*dt
            s3 = np.concatenate( ([0], abscissa ), axis = None)
            
            samples = np.zeros( (1, numSamples), dtype = int, order='C' )
            samples[0,0] = 0
  

            for i in range(1, numSamples-1):
                samples[0,i] =  int( np.round( _numPoints*s3[i] ) )   

            samples[0, numSamples-1] = _numPoints-1

            time = np.ones( (1,len(_time) ) )
            print("time")
            print(time[0,0:4])
            time[0][:] = _time
            s = s3
            v = v1

            print('s.size')
            print(s.size)
            print('_numPoints')
            print(_numPoints)
            print('numSamples')
            print(numSamples)    
            sv = np.zeros( (2,numSamples), dtype = float, order='C' ) 
            print('sv.shape')
            print(sv.shape)
            sv[0] = s
            sv[1] = v
            
    # Traj1: data of forward movement
            positions = np.zeros( (3,numSamples), dtype=float, order='C' ) 
            positions[0] = x[samples]; positions[1] = y[samples]; positions[2] = z[samples] 
            
    # Traj2: data of backward movement        
            positions2 = np.fliplr(positions)
            time2 = time
            sv2 = sv
            
    # Traj3: data of forward + backward movement          
            s = sv[0]
            v = sv[1]
            s3 = np.append(s/2, s[-1]/2 + s/2)
            v3 = np.append(v,v)
            sv3 = np.zeros(  (2, numSamples*2 ), dtype=float, order='C' ) 
            sv3[0]= s3
            sv3[1] = v3
            DiffTime = np.diff(time)
            minDiffTime = np.amin(DiffTime) #needed to build first elemtn of re
    #        time3 = np.zeros(  )     
#            time3 = np.append( time, time[0][-1] + minDiffTime/2 + time, axis = 1)
            time3 = np.ones( (1, 2*len(_time) ) )
            time3[0][:] = np.append( time, time[0][-1] + minDiffTime + time, axis = 1)

            positions3 =  np.append( positions, np.fliplr(positions), axis = 1 )         
            
    #       Creation of Trajectorydata to be save in yaml file
            TrjYamlData = dict()
    #        
            a_movement_definition = dict()
            a_movement_definition['type']= [ self.TypeOfMovement ]
            a_movement_definition['side'] = [ self.SideOfMovement ]
            a_movement_definition['vel_profile'] = [ self.vel_profile ]
            a_movement_definition['total_time'] = [ T ]
            a_movement_definition['begin_joint_config'] = [self.Start_RobotJointPosition ]
            a_movement_definition['begin_config'] = [ [x_begin, y_begin, z_begin] ]
            a_movement_definition['end_config'] = [ [x_end, y_end, z_end] ]
    
            TrjYamlData['a_movement_definition'] = a_movement_definition
    #       
            joint_names = [ 'joint_1', 'joint_2','joint_3']
    
    #        
    #       cart_trj1
            cart_trj1 = dict()
            positions1 = positions.T.tolist() #conversion to list is needed to make redable yaml files
            sv1 = sv.T.tolist()
            time1 = time.T.tolist()
            cart_trj1['cart_positions'] = positions1
            cart_trj1['MotionLaw'] = sv1 
            cart_trj1['time_from_start'] =  time1
            cart_trj1['joint_names'] =  joint_names
            TrjYamlData['cart_trj1'] = cart_trj1
    #
    #       cart_trj2      
            cart_trj2 =  dict()
            positions2 = positions2.T.tolist() #conversion to list is needed to make redable yaml files
            sv2 = sv2.T.tolist()
            time2 = time2.T.tolist()
            cart_trj2['cart_positions'] = positions2
            cart_trj2['MotionLaw'] = sv2 
            cart_trj2['time_from_start'] =  time2
            cart_trj2['joint_names'] =  joint_names
            TrjYamlData['cart_trj2'] = cart_trj2
    #
    #       cart_trj3
            cart_trj3 =  dict()
            positions3 = positions3.T.tolist() #conversion to list is needed to make redable yaml files
            sv3 = sv3.T.tolist()
            time3 = time3.T.tolist()
            cart_trj3['cart_positions'] = positions3
            cart_trj3['MotionLaw'] = sv3 
            cart_trj3['time_from_start'] =  time3      
            cart_trj3['joint_names'] =  joint_names
            TrjYamlData['cart_trj3'] = cart_trj3
            
            self.TrjYamlData =TrjYamlData
            NewFilename = QtWidgets.QFileDialog.getSaveFileName(None, "Save new movement as:", MovementsPath, "*.yaml")            

            if bool(NewFilename[0]):
                print('This is the filename of the loaded movement: ')
                print(NewFilename[0])
                print('This is T: %s' %T)
                self.SaveNewFile(TrjYamlData, NewFilename[0])
            else:
                # self.pushButton_CREATEMovement.enablePushButton(0)
                print('No proper filename was selected. Create movemement again or use the SaveMovement button')

    
    def SaveNewFile(self, Data, NewFilename):
        print('This is the new file.yaml:')
        print(NewFilename)
        yaml.Dumper.ignore_aliases = lambda *args : True
        with open(NewFilename, 'w') as outfile:
#            outfile.write(self._comment)
            yaml.dump(Data, outfile , default_flow_style=False)
# To change name in edit line
        FMRR_RootPath = self.ui_FMRRMainWindow.FMRR_Paths['Root']
        MovementsPath = self.ui_FMRRMainWindow.FMRR_Paths['Movements']
        MovementName = NewFilename [len(FMRR_RootPath+MovementsPath)+2:-5 ] # +2 for the / symbols
        _translate = QtCore.QCoreApplication.translate
        self.lineEdit.setText(_translate("MovementWindow", MovementName))
        print('MovementName: ')
        print(MovementName)

  
  
    def clbk_BtnLoadMovementData(self):
        FMRR_RootPath = self.ui_FMRRMainWindow.FMRR_Paths['Root']
        MovementsPath = self.ui_FMRRMainWindow.FMRR_Paths['Movements']
        filename = QtWidgets.QFileDialog.getOpenFileName(None, "Load Movement", MovementsPath, "*.yaml")
        if bool(filename[0]):
            print('This is the filename of the loaded movement:')
            print(filename)
            self.TrjYamlData = yaml.load(open(filename [0]), Loader=SafeLoader)
            self.CartesianMovementData = yaml.load(open(filename [0]), Loader=SafeLoader)
            _translate = QtCore.QCoreApplication.translate
            MovementName = filename [0][ len(FMRR_RootPath+MovementsPath)+2:-5 ] # +2 for the / symbols
            self.lineEdit.setText(_translate("MovementWindow", MovementName))
            self.lineEdit.MovementIsLoaded = 1 # da sistemare, va messo sotto l'altra finestra in modo da non resettare MovementWindow quando ci ritorno!!!
            self.pushButton_GOtoTraining.enablePushButton(1)
 

##############################################################################################################
#####                                                                                                    #####  
#####        Here after all parameters to create the movement are set and dispalyed in the GUI           ##### 
#####                                                                                                    #####
#####                                                                                                    #####
##############################################################################################################
#   

            self.TypeOfMovement = self.TrjYamlData.get("a_movement_definition").get("type")[0]           
            self.SideOfMovement = self.TrjYamlData.get("a_movement_definition").get("side")[0]
            self.vel_profile = self.TrjYamlData.get("a_movement_definition").get("vel_profile")[0]            
            MovTime =  self.TrjYamlData.get("a_movement_definition").get("total_time")[0]

#            print('TypeOfMovement[0]')
#            print(TypeOfMovement[0])
#            print('TypeOfMovement[0][0]')
#            print(TypeOfMovement[0][0])            
#           Checkbox type of moevement 
            if self.TypeOfMovement == 1:
                self.radioButton_TypeOfExercise_Reaching.setChecked(True)
                print('Type of movements is ''Reaching'' ')
                pass
            elif self.TypeOfMovement == 2:
                self.radioButton_TypeOfExercise_HandtoMouth.setChecked(True)
                print('Type of movements is ''Hand to Mouth'' ')
                pass
            else:
                print('No type of movement is selected!')
                
#           Checkbox side (left or right) 
            if self.SideOfMovement == 1:
                self.radioButton_SideLeft.setChecked(True)
                print('Selected side is ''left'' ')
                pass
            elif self.SideOfMovement == 2:
                self.radioButton_SideRight.setChecked(True)
                print('Selected side is ''Right'' ')
                pass
            else:
                print('No side is selected!')  

#           Checkbox Velocity profile  (Constant or Bell Shaped) 
            if self.vel_profile == 1:
                self.radioButton_VelocityProfileConstant.setChecked(True)
                print('Velocity is constant')
                pass
            elif self.vel_profile == 2:
                self.radioButton_VelocityProfileBellshaped.setChecked(True)
                print('Velocity is Bell-Shaped')
                pass
            else:
                print('No Velocity profile is selected!')                

#           START and END positions are converted from meters to cm and further displayed
            
            _toolPosCovFact = self.ui_FMRRMainWindow._toolPosCovFact
            
            StartPos = deepcopy(self.TrjYamlData.get("a_movement_definition").get("begin_config")[0])
            EndPos = deepcopy(self.TrjYamlData.get("a_movement_definition").get("end_config")[0])
            
            StartPos[:] = [i_position * _toolPosCovFact for i_position in StartPos]
            EndPos[:]   = [i_position * _toolPosCovFact for i_position in EndPos]
 
            self.lcdNumber_EndPos_X.display(int( EndPos[0] ))
            self.lcdNumber_EndPos_Y.display(int( EndPos[1] ))    
            self.lcdNumber_EndPos_Z.display(int( EndPos[2] ))
            self.doubleSpinBox_MoveTime.setValue(MovTime)


##### Start and End positions needed to create a new movement            
            self.Start_HandlePosition = deepcopy( self.TrjYamlData.get("a_movement_definition").get("begin_config")[0] )
            self.End_HandlePosition = deepcopy( self.TrjYamlData.get("a_movement_definition").get("end_config")[0] )
            self.Start_RobotJointPosition  = deepcopy( self.TrjYamlData.get("a_movement_definition").get("begin_joint_config") [0] )
            self.pushButton_CREATEMovement.enablePushButton(1)
#            

        else:
            print('No file was selected!')

            
##############################################################################################################
#####                                                                                                    #####  
#####                                            MOVEMENT PARAMETERS                                     ##### 
#####                                                 callbacks                                          #####
#####                                                                                                    #####
##############################################################################################################            


    def clbk_BtnGOtoTraining(self):
        if self.lineEdit.MovementIsLoaded:
            self.ui_FMRRMainWindow.pushButton_LoadCreateProtocol.enablePushButton(1)
        else:
            self.ui_FMRRMainWindow.pushButton_LoadCreateProtocol.enablePushButton(0)
        if self.SideOfMovement == 1: #Left
            print('side of movement1:')
            print(self.SideOfMovement)
            self.ui_FMRRMainWindow.radioButton_SideLeft.setChecked(True)
#            self.ui_FMRRMainWindow.radioButton_SideRight.setChecked(False)
        elif self.SideOfMovement == 2: #Right
            print('side of movement2:')
            print(self.SideOfMovement)
#            self.ui_FMRRMainWindow.radioButton_SideLeft.setChecked(False)
            self.ui_FMRRMainWindow.radioButton_SideRight.setChecked(True)
        else:
            print('non dovrei essere qui')
            self.ui_FMRRMainWindow.radioButton_SideLeft.setChecked(False)
            self.ui_FMRRMainWindow.radioButton_SideRight.setChecked(False)
        if self.TypeOfMovement == 1: # Reaching 
            self.ui_FMRRMainWindow.radioButton_TypeOfExercise_Reaching.setChecked(True)
#            self.ui_FMRRMainWindow.radioButton_TypeOfExercise_HandtoMouth.setChecked(False)
        elif self.TypeOfMovement == 2: # HtMM
#            self.ui_FMRRMainWindow.radioButton_TypeOfExercise_Reaching.setChecked(False)
            self.ui_FMRRMainWindow.radioButton_TypeOfExercise_HandtoMouth.setChecked(True)
        else:
            self.ui_FMRRMainWindow.radioButton_TypeOfExercise_Reaching.setChecked(False)
            self.ui_FMRRMainWindow.radioButton_TypeOfExercise_HandtoMouth.setChecked(False)
        if self.vel_profile == 1: # Reaching 
            self.radioButton_VelocityProfileConstant.setChecked(True)
#            self.ui_FMRRMainWindow.radioButton_TypeOfExercise_HandtoMouth.setChecked(False)
        elif self.vel_profile == 2: # HtMM
#            self.ui_FMRRMainWindow.radioButton_TypeOfExercise_Reaching.setChecked(False)
            self.radioButton_VelocityProfileBellshaped.setChecked(True) 
        else:
            self.radioButton_VelocityProfileConstant.setChecked(False)
            self.radioButton_VelocityProfileBellshaped.setChecked(False)
        self.DialogFMRRMainWindow.show()
        self.DialogMovementWindow.hide()


    def clbk_BtnGOtoRobotMovement(self):
        self.ui_FMRRMainWindow.DialogRobotWindow.show()
        self.DialogMovementWindow.hide()


    def setupUi_MovementWindow(self, MovementWindow):
        Ui_MovementWindow.setupUi(self, MovementWindow)
        MovementWindow.IsMovementReady = 0
                
                
    def retranslateUi_MovementWindow(self, MovementWindow):
        _translate = QtCore.QCoreApplication.translate
        MovementWindow.setWindowTitle(_translate("MovementWindow", "Movement Window"))
        self.lineEdit.MovementIsLoaded = 0

############################      Modify LCDs
######
      
        LcdWidgets  = [self.lcdNumber_EndPos_X, self.lcdNumber_EndPos_Y, self.lcdNumber_EndPos_Z ]  #, self.lcdNumber_J4, self.lcdNumber_J5, self.lcdNumber_J6,

        for iWidget in LcdWidgets:
            iWidget.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
            iWidget.setDigitCount(4)
        
       
#        LcdWidgetsSartEndposition = [ self.lcdNumber_StartPos_X, self.lcdNumber_StartPos_Y , self.lcdNumber_StartPos_Z ,
#                                   self.lcdNumber_EndPos_X, self.lcdNumber_EndPos_Y, self.lcdNumber_EndPos_Z ]
#        
#        for iWidgetES in LcdWidgetsSartEndposition:
#           iWidgetES.setDigitCount(6)


#####
##############################################################################################

        
        ##############################################################################################
        #####                                                                                    #####  
        #####                               ENABLE/DISABLE BUTTONS                               ##### 
        #####                               (In MAIN add this line ...                           #####
        #####                  "QPushButton.enablePushButton = MC_Tools.enablePushButton"        #####
        #####                                                                                    #####
        #####                                                                                    #####
        ##############################################################################################        
        
# ENABLE
        self.pushButton_LOADMovement.enablePushButton(1)
        self.pushButton_GOtoTraining.enablePushButton(1)
        self.pushButton_GOtoRobotMovement.enablePushButton(1)
        
    
# DISABLE        
        self.pushButton_CREATEMovement.enablePushButton(1)
        
        ##############################################################################################
        #####                                                                                    #####  
        #####                                   MOVEMENT PARAMETERS                              ##### 
        #####                                      Connections                                   #####
        #####                                                                                    #####
        ##############################################################################################
        
        self.pushButton_LOADMovement.clicked.connect(lambda: self.clbk_BtnLoadMovementData())
        self.pushButton_CREATEMovement.clicked.connect(lambda: self.clbk_BtnCreateMovementData())

        ##############################################################################################
        #####                                                                                    #####  
        #####                                   GENERAL BUTTONS                                  ##### 
        #####                                     Connections                                    #####
        #####                                                                                    #####
        ##############################################################################################

        self.pushButton_GOtoTraining.clicked.connect(lambda: self.clbk_BtnGOtoTraining())
        self.pushButton_GOtoRobotMovement.clicked.connect(lambda: self.clbk_BtnGOtoRobotMovement())
        MovementWindow.adjustSize()
        

def main():
    QPushButton.enablePushButton = MC_Tools.enablePushButton #Adding a new method defined in MC_Tools file    
    app = QtWidgets.QApplication(sys.argv)
    MovementWindow = QtWidgets.QDialog()
    ui = FMRR_Ui_MovementWindow()
    ui.setupUi(ui,MovementWindow)
    ui.retranslateUi_MovementWindow(ui,MovementWindow)
    ui.GetCurrentPositions(ui)
    MovementWindow.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
