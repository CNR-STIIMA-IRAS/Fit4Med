# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QTimer

from ui.uiMotorsWindow import Ui_MotorsWindow
from RosCommunicationManager import RosCommunicationManager

class MotorsWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MotorsWindow()
        self.ui.setupUi(self)  # Set up the UI for the secondary widget

    def connect(self, ROS: RosCommunicationManager, parent_timer: QTimer):
        self.ROS : RosCommunicationManager = ROS
        self.ui.pushButton_ResetFaults.clicked.connect(self.resetFaults) #type: ignore
        self.parent_timer = parent_timer
        self.ui.comboBox_ResetFaults.currentIndexChanged.connect(self.ROS.resetModeChanged)

        # Create a QTableWidget
        self.ui.tableWidget_MotorsInfo.setRowCount(len(self.ROS.joint_names))      # Set the number of rows
        self.ui.tableWidget_MotorsInfo.setColumnCount(4)  # Set the number of columns
        self.ui.tableWidget_MotorsInfo.setColumnWidth(0, 100)
        self.ui.tableWidget_MotorsInfo.setColumnWidth(1, 200)
        self.ui.tableWidget_MotorsInfo.setColumnWidth(2, 200)
        self.ui.tableWidget_MotorsInfo.setColumnWidth(3, 100)
        
        self.ui.tableWidget_EthercatNodesInfo.setRowCount(self.ROS.number_of_ec_slaves)      # Set the number of rows
        self.ui.tableWidget_EthercatNodesInfo.setColumnCount(2)  # Set the number of columns
        self.ui.tableWidget_EthercatNodesInfo.setColumnWidth(0, 300)
        self.ui.tableWidget_EthercatNodesInfo.setColumnWidth(1, 300)

        # Pre-create table items so we don't recreate them every tick
        self._motor_items = []
        for i in range(len(self.ROS.joint_names)):
            row_items = []
            for j in range(4):
                item = QtWidgets.QTableWidgetItem('')
                self.ui.tableWidget_MotorsInfo.setItem(i, j, item)
                row_items.append(item)
            self._motor_items.append(row_items)

        self._ec_items = []
        for i in range(self.ROS.number_of_ec_slaves):
            row_items = []
            for j in range(2):
                item = QtWidgets.QTableWidgetItem('')
                self.ui.tableWidget_EthercatNodesInfo.setItem(i, j, item)
                row_items.append(item)
            self._ec_items.append(row_items)

        # Cache for emergency state to avoid redundant setText/setStyleSheet
        self._last_emergency_state = None
        self._last_controller_name = None

    def resetFaults(self) -> None:
        if self.ui.comboBox_ResetFaults.currentIndex() == 2:
            print('Switch Off Logic Power')
            self.ROS.driveLogicSwitchOff()
            self.ui.pushButton_ResetFaults.setEnabled(False)
            QTimer.singleShot(2000, self._resetFaults_driveLogicOn)
        elif self.ui.comboBox_ResetFaults.currentIndex() == 3:
            print('Switch Off FT Sensor Power')
            self.ROS.FTSensorReset(0)
            self.ui.pushButton_ResetFaults.setEnabled(False)
            QTimer.singleShot(2000, self._resetFaults_FTSensorOn)
        else:
            self.ROS.resetFaults()

    def _resetFaults_driveLogicOn(self) -> None:
        print('Switch On Logic Power')
        self.ROS.driveLogicSwitchOn()
        self.ui.pushButton_ResetFaults.setEnabled(True)

    def _resetFaults_FTSensorOn(self) -> None:
        self.ROS.FTSensorReset(1)
        print('Switch On FT Sensor Power')
        self.ui.pushButton_ResetFaults.setEnabled(True)


    def resetModeChanged(self, index: int) -> None:
        self.ROS.resetModeChanged(index=index)

    def disconnect_ros(self):
        self.ui.comboBox_ResetFaults.currentIndexChanged.disconnect(self.ROS.resetModeChanged)
        
    def updateWindow(self):
        # Determine emergency state key
        if not self.ROS.isRosCommunicationActive():
            state_key = 'disconnected'
        elif self.ROS.isEmergencyActive():
            state_key = 'emergency'
        elif self.ROS.isInFaultState():
            state_key = 'fault'
        elif self.ROS.areMotorsOn():
            state_key = 'motors_on'
        else:
            state_key = 'motors_off'

        if state_key != self._last_emergency_state:
            self._last_emergency_state = state_key
            if state_key == 'disconnected':
                self.ui.lineEdit_Emergency.setText('Disconnected')
                self.ui.lineEdit_Emergency.setStyleSheet("background-color: rgb(255,215,00); color: white")
            elif state_key == 'emergency':
                self.ui.lineEdit_Emergency.setText('!!! Emergency !!!')
                self.ui.lineEdit_Emergency.setStyleSheet("background-color: red; color: white")
            elif state_key == 'fault':
                self.ui.lineEdit_Emergency.setText("MOTORS FAULT")
                self.ui.lineEdit_Emergency.setStyleSheet("background-color: red; color: white")
            elif state_key == 'motors_on':
                self.ui.lineEdit_Emergency.setText("Warning \n Motors On")
                self.ui.lineEdit_Emergency.setStyleSheet("background-color: rgb(255,215,00); color: white")
            else:
                self.ui.lineEdit_Emergency.setText("Motors Off \n State OK")
                self.ui.lineEdit_Emergency.setStyleSheet("background-color: green; color: white")

        if state_key not in ('disconnected', 'emergency'):
            self.ui.pushButton_ResetFaults.setEnabled(self.ROS.isManualResetFaults())

        ctrl_name = self.ROS.getCurrentControllerName()
        if ctrl_name != self._last_controller_name:
            self._last_controller_name = ctrl_name
            self.ui.plainTextEdit_controller_name.setPlainText(ctrl_name)

        # Update table items in-place (setText instead of creating new QTableWidgetItems)
        for i, name in enumerate(self.ROS.getJointNames()):
            self._motor_items[i][0].setText(name)
        for i, state in enumerate(self.ROS.getDriversStates()):
            self._motor_items[i][1].setText(state)
        for i, moo in enumerate(self.ROS.getDriversModeOfOperations()):
            self._motor_items[i][2].setText(moo)
        for i, q in enumerate(self.ROS.getDriversFeedbackPosition()):
            self._motor_items[i][3].setText(f'{q:.3f}')

        for i, name in enumerate(self.ROS.getSlaveNames()):
            self._ec_items[i][0].setText(name)
        for i, state in enumerate(self.ROS.getSlaveStates()):
            self._ec_items[i][1].setText(state)


