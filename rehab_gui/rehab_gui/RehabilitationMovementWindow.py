# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from GuiRosTasks import Call, gui_task
from YamlSupport import (read_yaml, validate_movement, validate_protocol,
                         atomic_save_yaml, gui_transaction, check_spin_value,
                         number, YamlDataError, report_yaml_error)


# -*- coding: utf-8 -*-


from enum import Enum
import sys
import traceback
from pathlib import Path
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QMessageBox, QPushButton, QFileDialog, QWidget, QApplication
from PyQt5.QtCore import QTimer
from ui.uiRehabilitationMovementWindow import Ui_RehabilitationMovementWindow
from RosCommunicationManager import RosCommunicationManager
import yaml
try:
    from yaml import CSafeLoader as SafeLoader
except ImportError:
    from yaml import SafeLoader
import numpy as np
from copy import deepcopy
#MC Classes/methods

class ExerciseType(Enum):
    NONE = 0
    REACHING = 1
    HAND_TO_MOUTH = 2

class SimpleFileDialog(QFileDialog):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setOption(QFileDialog.DontUseNativeDialog, True)
        self.setSidebarUrls([])  # remove content
        self.setFileMode(QFileDialog.ExistingFile)
        # Constrain to available screen area so the dialog never exceeds the display
        screen = QApplication.primaryScreen().availableGeometry()
        self.setMaximumSize(screen.width(), screen.height())
        self.resize(min(int(screen.width() * 0.9), 1100), min(int(screen.height() * 0.85), 750))

        for child in self.findChildren(QWidget):
            if "sidebar" in child.objectName().lower():
                child.hide()

def open_file(title="Select a file", path=""):
    dlg = SimpleFileDialog()
    dlg.setWindowTitle(title)
    dlg.setDirectory(path)
    dlg.setNameFilter("YAML files (*.yaml)")
    dlg.showMaximized()
    return dlg.selectedFiles()[0] if dlg.exec_() else None

class RehabilitationMovementWindow(QtWidgets.QDialog):
    
    def __init__(self,main_app) -> None:
        super().__init__()
        self.ui = Ui_RehabilitationMovementWindow()
        self.ui.setupUi(self)
        self.main_app = main_app
        self.SideOfMovement = 0 
        self.TypeOfMovement : ExerciseType = ExerciseType.NONE
        self.vel_profile = 2
        self._go_to_start_retry_armed = False

        self.main_app.movement_loaded = 0
        
        ######## Modify LCDs
        LcdWidgets  = [self.ui.lcdNumber_EndPos_X, self.ui.lcdNumber_EndPos_Y, self.ui.lcdNumber_EndPos_Z ]
        for iWidget in LcdWidgets:
            iWidget.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
            iWidget.setDigitCount(4)
        
        # ENABLE or DISABLE buttons in MovementWindow
        self.ui.pushButton_LOADMovement.setEnabled(True)
        self.ui.pushButton_SetCurrentPos_2.setEnabled(True)    
        self.ui.pushButton_GoToZERO.setEnabled(True)
        self.ui.pushButton_CREATEMovement.setEnabled(False)
        self.ui.pushButton_SAVEMovement.setEnabled(False)
        
        ##############################################################################################
        #####                                                                                    #####  
        #####                                   MOVEMENT PARAMETERS                              ##### 
        #####                                      Connections                                   #####
        #####                                                                                    #####
        ##############################################################################################
        
        self.ui.pushButton_LOADMovement.clicked.connect(self.clbk_BtnLoadMovementData)
        self.ui.pushButton_CREATEMovement.clicked.connect(self.clbk_BtnCreateMovementData)
        self.ui.pushButton_SetCurrentPos_2.clicked.connect(self.clbk_pushButton_SetCurrentPos)
        self.ui.pushButton_SAVEMovement.clicked.connect(self.clbk_BtnSAVEMovement)
        self.ui.pushButton_GoToZERO.clicked.connect(self.clbk_BtnGoToStartPosition)

        ##############################################################################################
        #####                                                                                    #####  
        #####                                   GENERAL BUTTONS                                  ##### 
        #####                                     Connections                                    #####
        #####                                                                                    #####
        ##############################################################################################

    def connect(self, ROS: RosCommunicationManager, parent_timer: QTimer):
        self.ROS = ROS
        self.parent_timer = parent_timer

    def updateWindow(self):
        if self.ROS.isRosCommunicationActive():
            self.ui.pushButton_GoToZERO.setEnabled(True)
        else:
            self.ui.pushButton_GoToZERO.setEnabled(False)
            self._go_to_start_retry_armed = False

        if self.main_app.ui.tabWidget.currentIndex() == 1 and\
            self.ROS.areMotorsOn() and self.ROS.getTrajectoryCompleted():
            self.ui.pushButton_GoToZERO.setChecked(False)
            self.ROS.setTrajectoryCompleted(False)
            self._finishTrajectory()

##############################################################################################################
#####                                                                                                    #####  
#####                                        CREATE TRAJECTORY                                           ##### 
#####                                                                                                    #####
#####                                                                                                    #####
##############################################################################################################
    def clbk_BtnCreateMovementData(self):
        # This runs in a Qt slot: an exception escaping it makes PyQt5 abort the
        # whole GUI. Numerical failures (interpolation of odd source files,
        # degenerate geometry) are reported instead.
        if not (self.ui.radioButton_TypeOfExercise_Reaching.isChecked()
                or self.ui.radioButton_TypeOfExercise_HandtoMouth.isChecked()):
            QMessageBox.warning(self, "Warning", "Select the type of exercise (Reaching or Hand to Mouth) before creating the movement.")
            return
        try:
            self._createMovementData()
        except Exception as exc:
            traceback.print_exc()
            QMessageBox.critical(self, "Movement not created",
                                 f"The movement could not be created:\n{type(exc).__name__}: {exc}")

    def _createMovementData(self):
        source_data = None
        if self.ui.radioButton_TypeOfExercise_HandtoMouth.isChecked():
            source_path = open_file(path=self.main_app.FMRR_Paths['Movements'])
            if not source_path:
                return
            try:
                source_data = read_yaml(source_path, validate_movement)
                if len(source_data['cart_trj3']['cart_positions']) < 6:
                    raise YamlDataError('Il movimento sorgente richiede almeno 6 punti per la suddivisione andata/ritorno.')
            except Exception as exc:
                report_yaml_error(self, source_path, exc)
                return
        from scipy import interpolate

        MovementsPath = self.main_app.FMRR_Paths['Movements']
        update_rate = 50
        
        if self.ui.radioButton_SideLeft.isChecked() == True:
            self.SideOfMovement = 1
        elif self.ui.radioButton_SideRight.isChecked() == True :
            self.SideOfMovement = 2
        else:
            self.SideOfMovement = 0
              
        if self.ui.radioButton_TypeOfExercise_Reaching.isChecked() == True:
            self.TypeOfMovement = ExerciseType.REACHING
        elif self.ui.radioButton_TypeOfExercise_HandtoMouth.isChecked() == True:
            self.TypeOfMovement = ExerciseType.HAND_TO_MOUTH
        else:
            self.TypeOfMovement = ExerciseType.NONE
            
        x_begin = 0.0        
        y_begin = 0.0
        z_begin = 0.0
        x_end = deepcopy(self.End_HandlePosition[0])
        y_end = deepcopy(self.End_HandlePosition[1])
        z_end = deepcopy(self.End_HandlePosition[2])
        x12 = x_end-x_begin; y12 = y_end-y_begin; z12 = z_end-z_begin
        L12 = np.sqrt( x12**2 + y12**2 + z12**2 )
        x1 = 0; y1 = 0; z1 = 0
        x2 = x12; y2 = y12; z2 = z12

        T = self.ui.doubleSpinBox_MoveTime.value() # total movement time in sec

        if abs(T) < 0.1:
                QMessageBox.warning(self, "Warning", "Set a valid movement time")
                return

        t_sample = 1/update_rate # sampling time in sec
        _time = np.linspace(0, T, int(T/t_sample)+1, endpoint = True) # To satisfy Beschi's condition of having steps = 8ms
        print("_time")
        print(_time[0:4])
        numSamples = len(_time) # length of the final vectors, the ones given as results
        _numPoints = numSamples * 30 #lenght of oversampled vectors
        v1 = np.zeros( (numSamples,), dtype = float, order='C' )    

        _ContinueCreateMovement = 0
        if self.TypeOfMovement == ExerciseType.REACHING:  # Reaching (rectilinear trajectory)
            x = np.linspace( x1, x2, _numPoints)
            y = np.linspace( y1, y2, _numPoints )
            z = np.linspace( z1, z2, _numPoints )
            L = L12
            _ContinueCreateMovement = 1
        elif self.TypeOfMovement == ExerciseType.HAND_TO_MOUTH:    # Hand to Mouth
            if source_data is not None:
                _TrjYamlData = source_data
                PositionsLenght_LF = len( _TrjYamlData.get("cart_trj3").get("cart_positions") )
                cart_positions_LF = _TrjYamlData.get("cart_trj3").get("cart_positions")

                x_coords_LF = np.array([p[0] for p in cart_positions_LF])
                y_coords_LF = np.array([p[1] for p in cart_positions_LF])
                z_coords_LF = np.array([p[2] for p in cart_positions_LF])

                x_coords_andata_LF = np.array([p[0] for p in cart_positions_LF[0:PositionsLenght_LF//2-1]])
                y_coords_andata_LF = np.array([p[1] for p in cart_positions_LF[0:PositionsLenght_LF//2-1]])
                z_coords_andata_LF = np.array([p[2] for p in cart_positions_LF[0:PositionsLenght_LF//2-1]])

                MovementData = cart_positions_LF[0:PositionsLenght_LF//2-1]
                side = _TrjYamlData.get("a_movement_definition").get("side")
                _originalNumPoints = len(MovementData)
                x1_LF = MovementData[0][0]; x2_LF = MovementData[-1][0]      #_LF = Loaded File   
                y1_LF = MovementData[0][1]; y2_LF = MovementData[-1][1]
                z1_LF = MovementData[0][2]; z2_LF = MovementData[-1][2]                
                x12_LF = x2_LF-x1_LF; y12_LF = y2_LF-y1_LF; z12_LF = z2_LF-z1_LF
                L12_LF = np.sqrt( x12_LF**2 + y12_LF**2 + z12_LF**2 ) # actually the starting point is [0,0,0]

                print("position lenght lf: ", PositionsLenght_LF)
                print("movement data lenght lf: ", len(MovementData))
                print("loaded movement data size [x, y , z]: ",len(x_coords_LF), len(y_coords_LF), len(z_coords_LF))

                print('L12_LF:')
                print(L12_LF)
                # _ConvPara = L12/L12_LF
                if min(abs(x2_LF - x1_LF), abs(y2_LF - y1_LF), abs(z2_LF - z1_LF)) < 1e-9:
                    raise YamlDataError('The source movement has no displacement along one axis '
                                        f'(dx={x2_LF - x1_LF}, dy={y2_LF - y1_LF}, dz={z2_LF - z1_LF}): it cannot be scaled.')
                _ConvParaX = np.abs(x12/(x2_LF - x1_LF))
                _ConvParaY = np.abs(y12/(y2_LF - y1_LF))
                _ConvParaZ = np.abs(z12/(z2_LF - z1_LF))
                print('_ConvParaX')
                print(_ConvParaX)
                print('_ConvParaY:')
                print(_ConvParaY)
                print('_ConvParaZ:')
                print(_ConvParaZ)
 
                x_new = np.zeros( (1, _originalNumPoints), dtype=float, order='C' ) 
                y_new = np.zeros( (1, _originalNumPoints), dtype=float, order='C' )  
                z_new = np.zeros( (1, _originalNumPoints), dtype=float, order='C' ) 
        
                x_new  = x_coords_andata_LF * _ConvParaX
                y_new  = y_coords_andata_LF * _ConvParaY
                z_new  = z_coords_andata_LF * _ConvParaZ

                #First interpolation
                xyz = np.zeros( (3, len(MovementData) ), dtype=float, order='C' )
                # xyz = np.ndarray( 3,len(x) )

                xyz[0,:] = x_new 
                xyz[1,:] = y_new 
                xyz[2,:] = z_new
        
                distances_2 = np.sum( np.diff( xyz, 1, 1 ) * np.diff( xyz, 1, 1 ), 0)          # distance^2 ; matlab  sum( diff( xyz,[],1 ).^2,2 )
                distances = np.sqrt( distances_2 )
                cum_distances = np.cumsum(distances)
                s1 = np.concatenate( ([0], cum_distances), axis = None)    #% Cumulative sum to calculate abscissa S  
                L1 = s1[-1]                                             # L1 = trajectory length
                interp_function_1 = interpolate.interp1d(s1, xyz, 'cubic')
                print('_numpoints')
                print(_numPoints)
                print('L1:')
                print(L1)
                CSn_1 = np.linspace(0, L1, _numPoints)
                xyz_1 = interp_function_1( CSn_1 )
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
                # interp_function_2 = interpolate.interp1d(s2, xyz_1, 'cubic')
                interp_function_2 = interpolate.interp1d(s2, xyz_1, 'cubic')
                CSn_2 = np.linspace( 0, L, _numPoints )
                XYZ = interp_function_2( CSn_2 ) 
                print('L:')
                print(L)
                x = XYZ[0,:]
                y = XYZ[1,:]
                z = XYZ[2,:]
                x_end = x_begin + float(x[-1])
                y_end = y_begin + float(y[-1])
                z_end = z_begin + float(z[-1])  
                print( "x_end" )
                print( x_end )
                print( "y_end" )
                print( y_end )
                print( "z_end" )
                print( z_end )
                print('x_endConv:')
                print(x_new[-1])
                print('y_endConv:')
                print(y_new[-1])
                print('z_endCov:')
                print(z_new[-1])

                print("self.SideOfMovement")
                print(self.SideOfMovement)
                print("side da file:")
                print(side)
                if self.SideOfMovement != side[0]:
                    print("Inverto movimento per cambio braccio")
                    print("y before: ", y[10:14])
                    y = - y
                    print("y after: ", y[10:14])


                print('_numpoints')
                print(_numPoints)
                _numPoints2 = len(x)
                print('_numpoints2')
                print(_numPoints2)                
                
                _ContinueCreateMovement = 1    
            else:
                _ContinueCreateMovement = 0

        if _ContinueCreateMovement == 1:
            Vmax = 0.0
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
                Vmax = np.max(v1) 
            
            # new abscissa calculation, and resampling
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
            positions[0] = x[samples] # type:ignore
            positions[1] = y[samples] # type:ignore
            positions[2] = z[samples] # type:ignore

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

            positions3 =  np.append( positions, positions2, axis = 1 )      

            # Main application values are committed only after validation.
            
    #       Creation of Trajectorydata to be save in yaml file
            TrjYamlData = dict()
    #        
            a_movement_definition = dict()
            a_movement_definition['type']= [ self.TypeOfMovement.value ]
            a_movement_definition['side'] = [ self.SideOfMovement ]
            a_movement_definition['vel_profile'] = [ self.vel_profile ]
            a_movement_definition['max_velocity'] = [ float(Vmax*100) ]
            a_movement_definition['total_time'] = [ T ]
            a_movement_definition['begin_joint_config'] = [ [x_begin, y_begin, z_begin] ]
            a_movement_definition['begin_config'] = [ [x_begin, y_begin, z_begin] ]
            a_movement_definition['end_config'] = [ [x_end, y_end, z_end] ]
    
            TrjYamlData['a_movement_definition'] = a_movement_definition
    #       
            joint_names = [ 'joint_x', 'joint_y','joint_z']
    
    #        
    #       cart_trj1
            # cart_trj1 = dict()
            # positions1 = positions.T.tolist() #conversion to list is needed to make redable yaml files
            # sv1 = sv.T.tolist()
            # time1 = time.T.tolist()
            # cart_trj1['cart_positions'] = positions1
            # cart_trj1['MotionLaw'] = sv1 
            # cart_trj1['time_from_start'] =  time1
            # cart_trj1['joint_names'] =  joint_names
            # TrjYamlData['cart_trj1'] = cart_trj1
    #
    #       cart_trj2      
            # cart_trj2 =  dict()
            # positions2 = positions2.T.tolist() #conversion to list is needed to make redable yaml files
            # sv2 = sv2.T.tolist()
            # time2 = time2.T.tolist()
            # cart_trj2['cart_positions'] = positions2
            # cart_trj2['MotionLaw'] = sv2 
            # cart_trj2['time_from_start'] =  time2
            # cart_trj2['joint_names'] =  joint_names
            # TrjYamlData['cart_trj2'] = cart_trj2
    #
    #       cart_trj3
            cart_trj3 =  dict()
            positions3 = positions3.T.tolist() #conversion to list is needed to make redable yaml files
            sv3 = sv3.T.tolist()
            time3 = time3.T.tolist()
            cart_trj3['cart_positions'] = positions3
            # cart_trj3['MotionLaw'] = sv3 
            cart_trj3['time_from_start'] =  time3      
            cart_trj3['joint_names'] =  joint_names
            TrjYamlData['cart_trj3'] = cart_trj3
            
            try:
                validate_movement(TrjYamlData, execution=True)
            except Exception as exc:
                report_yaml_error(self, "movimento generato", exc)
                return
            self.TrjYamlData = TrjYamlData
            self.main_app.PhaseDuration = 2 * T
            self.main_app.Vmax = float(Vmax * 100)
            NewFilename = QtWidgets.QFileDialog.getSaveFileName(None, "Save new movement as:", MovementsPath, "*.yaml")            
            self.ui.pushButton_SAVEMovement.setEnabled(True)

            if bool(NewFilename[0]):
                print('This is the filename of the created movement: ')
                print(NewFilename[0])
                print('This is T: %s' %T)
                if not self.SaveNewFile(TrjYamlData, NewFilename[0]):
                    return
                self.main_app.PhaseDuration = 2 * T
                # self.main_app.Vmax = Vmax*100 # conversion to cm/s
                self.main_app.movement_loaded = 1
            else:
                self.ui.pushButton_CREATEMovement.setEnabled(False)
                print('No proper filename was selected. Create movemement again or use the SaveMovement button')
                    
    def clbk_BtnSAVEMovement(self):
        MovementsPath = self.main_app.FMRR_Paths['Movements']
        NewFilename = QtWidgets.QFileDialog.getSaveFileName(None, "Save new movement as:", MovementsPath, "*.yaml")            
            
        if bool(NewFilename[0]):
            print('This is the filename of the loaded movement:')
            print(NewFilename[0])
            if self.SaveNewFile(self.TrjYamlData, NewFilename[0]):
                self.main_app.movement_loaded = True

    def SaveNewFile(self, Data, NewFilename):
        destination = Path(NewFilename)
        if destination.suffix.lower() not in ('.yaml', '.yml'):
            destination = Path(str(destination) + '.yaml')
        try:
            atomic_save_yaml(destination, Data, validate_movement)
        except Exception as exc:
            report_yaml_error(self, destination, exc)
            return False
        self.ui.lineEdit_MovementName.setText(destination.stem)
        return True


    def clbk_BtnLoadMovementData(self) -> None:
        training = self.main_app.trainingProtocolWindow
        if training.Training_ON or training._stop_pending or self.ROS.isCommandBusy():
            QMessageBox.warning(self, "Movimento", "Terminare l'operazione in corso prima di caricare un movimento.")
            return
        selected_file = open_file(path=self.main_app.FMRR_Paths['Movements'])
        if not selected_file:
            return
        try:
            candidate = read_yaml(selected_file, lambda data: validate_movement(data, execution=True))
            self._applyMovement(candidate, Path(selected_file).stem)
        except Exception as exc:
            report_yaml_error(self, selected_file, exc)

    def _applyMovement(self, candidate, name):
        # Every conversion/range check happens before changing self or widgets.
        validate_movement(candidate, execution=True)
        meta = candidate['a_movement_definition']
        kind = ExerciseType(meta['type'][0])
        side, profile = meta['side'][0], meta['vel_profile'][0]
        duration, vmax = meta['total_time'][0], meta['max_velocity'][0]
        check_spin_value(self.ui.doubleSpinBox_MoveTime, duration, 'total_time')
        end = list(meta['end_config'][0])
        display = [int(value * 100) for value in end[:3]]
        radios = [self.ui.radioButton_TypeOfExercise_Reaching,
                  self.ui.radioButton_TypeOfExercise_HandtoMouth,
                  self.ui.radioButton_SideLeft, self.ui.radioButton_SideRight]
        widgets = [(w, 'isChecked', 'setChecked') for w in radios]
        widgets += [(self.ui.lineEdit_MovementName, 'text', 'setText'),
                    (self.ui.doubleSpinBox_MoveTime, 'value', 'setValue'),
                    (self.ui.pushButton_SAVEMovement, 'isEnabled', 'setEnabled'),
                    (self.ui.pushButton_CREATEMovement, 'isEnabled', 'setEnabled')]
        widgets += [(w, 'value', 'display') for w in (self.ui.lcdNumber_EndPos_X, self.ui.lcdNumber_EndPos_Y, self.ui.lcdNumber_EndPos_Z)]
        names = ['TrjYamlData', 'TypeOfMovement', 'SideOfMovement', 'vel_profile',
                 'JointTargetPosition', 'Start_HandlePosition', 'End_HandlePosition',
                 'Start_RobotJointPosition', '_go_to_start_retry_armed']
        with gui_transaction([(self, names), (self.main_app, ['Vmax', 'PhaseDuration', 'movement_loaded'])], widgets):
            self.TrjYamlData = candidate
            self.TypeOfMovement, self.SideOfMovement, self.vel_profile = kind, side, profile
            self.JointTargetPosition = list(end)
            self.Start_HandlePosition = list(meta['begin_config'][0])
            self.End_HandlePosition = end
            self.Start_RobotJointPosition = list(meta['begin_joint_config'][0])
            self._go_to_start_retry_armed = False
            self.main_app.Vmax, self.main_app.PhaseDuration = vmax, 2 * duration
            self.ui.lineEdit_MovementName.setText(name)
            self.ui.doubleSpinBox_MoveTime.setValue(duration)
            self.ui.radioButton_TypeOfExercise_Reaching.setChecked(kind == ExerciseType.REACHING)
            self.ui.radioButton_TypeOfExercise_HandtoMouth.setChecked(kind == ExerciseType.HAND_TO_MOUTH)
            self.ui.radioButton_SideLeft.setChecked(side == 1)
            self.ui.radioButton_SideRight.setChecked(side == 2)
            for widget, value in zip((self.ui.lcdNumber_EndPos_X, self.ui.lcdNumber_EndPos_Y, self.ui.lcdNumber_EndPos_Z), display):
                widget.display(value)
            self.ui.pushButton_SAVEMovement.setEnabled(True)
            self.ui.pushButton_CREATEMovement.setEnabled(True)
            self.main_app.movement_loaded = True  # Commit last.


            
##############################################################################################################
#####                                                                                                    #####  
#####                                            MOVEMENT PARAMETERS                                     ##### 
#####                                                 callbacks                                          #####
#####                                                                                                    #####
##############################################################################################################    
    def clbk_pushButton_SetCurrentPos(self):
        _toolPosCovFact = self.main_app._toolPosCovFact
        # Deep-copy before any local mutation: ROS.HandlePosition/RobotJointPosition
        # are live references shared with the joint_states callback (and, by
        # construction, with each other -- see getJointAndToolState). Scaling
        # them in place for display would corrupt the actual feedback used as
        # the robot's current position for the next PTP/Go-To-Start command.
        HandlePosition = deepcopy(self.main_app.trainingProtocolWindow.ROS.getHandleFeedbackPosition())
        RobotJointPosition = deepcopy(self.main_app.trainingProtocolWindow.ROS.getRobotJointPosition())#       Put doublespin values in list to allow for iteration

        # save data to create movement
        self.End_HandlePosition = deepcopy(HandlePosition)
        self.End_RobotJointPosition = RobotJointPosition

        # visualize data
        HandlePosition[:] = [iPosition * _toolPosCovFact for iPosition in HandlePosition]
        self.ui.lcdNumber_EndPos_X.display( int( HandlePosition[0] ) ) 
        self.ui.lcdNumber_EndPos_Y.display( int( HandlePosition[1] ) ) 
        self.ui.lcdNumber_EndPos_Z.display( int( HandlePosition[2] ) )
        self.ui.pushButton_CREATEMovement.setEnabled(True)

    @gui_task
    def clbk_BtnGoToStartPosition(self) -> None:
        if not self.main_app.syncExerciseTypeToPLC(force=True):
            QMessageBox.warning(self, 'Warning', 'Failed to synchronize exercise type with PLC.\nPlease check the PLC connection and try again.')
            return
        self.ROS.setManualMode(False)
        switch_ok = (yield Call(self.ROS.enableControllerBehaviour, 'GoToStart'))
        ctrl = self.ROS.getCurrentControllerName()
        target_ctrl = self.ROS.getGoToStartControllerName()
        ctrl_ready = ctrl == target_ctrl
        if not (switch_ok and ctrl_ready):
            if not self._go_to_start_retry_armed:
                self._go_to_start_retry_armed = True
                QMessageBox.information(self, 'Info', 'Setting mode of operation 8 and Go to start controller.\nPress the button again to start the movement.')
                return
            moos = self.ROS.getDriversModeOfOperations()
            if not switch_ok:
                QMessageBox.warning(self, 'Warning', f'Could not switch to Cyclic Synchronous Position mode (mode 8).\nActive controller: {ctrl}\nDrive modes: {moos}\nPlease verify that all drives are operational before starting training.')
                return
            QMessageBox.warning(self, 'Warning', f'Go to start controller is not active (active: {ctrl}).\nCannot start training.')
            return
        self._go_to_start_retry_armed = False
        self.Training_ON = True
        yield Call(__import__("time").sleep, 0.5)
        yield from self._goToStartPosition_afterDelay.__wrapped__(self)

    @gui_task
    def _finishTrajectory(self):
        if (yield Call(self.ROS.turnOffMotors)):
            if self.ROS.getCurrentControllerName() == self.ROS.getGoToStartControllerName():
                yield Call(self.ROS.enableControllerBehaviour, 'PTP')

    @gui_task
    def _restoreTrajectoryController(self):
        yield Call(self.ROS.enableControllerBehaviour, 'PTP')

    @gui_task
    def _goToStartPosition_afterDelay(self):
        if self.ROS.getCurrentControllerName() != self.ROS.getGoToStartControllerName():
            if not (yield Call(self.ROS.enableControllerBehaviour, 'GoToStart')):
                QMessageBox.warning(self, 'Warning', 'Failed in setting the go to startcontroller')
                return
        if (yield Call(self.ROS.turnOnMotors)):
            yield Call(self.ROS.sendGoToStartPTPTrajectory, [0.0, 0.0, 0.0], 3.0)
        else:
            QMessageBox.warning(self, 'Warning', 'Failed in switching on the motors')

        
def main():
    app = QtWidgets.QApplication(sys.argv)
    MovementWindow = QtWidgets.QDialog()
    ui = RehabilitationMovementWindow(None)
    MovementWindow.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
