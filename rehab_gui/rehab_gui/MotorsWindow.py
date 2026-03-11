# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from PyQt5 import QtWidgets, QtCore, QtWidgets
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QTimer

from ui.uiMotorsWindow import Ui_MotorsWindow
from RosCommunicationManager import RosCommunicationManager

import time

class MotorsWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MotorsWindow()
        self.ui.setupUi(self)  # Set up the UI for the secondary widget

    def connect(self, ROS: RosCommunicationManager, parent_timer: QTimer):
        self.ROS : RosCommunicationManager = ROS
        self.ui.pushButton_ResetFaults.clicked.connect(self.resetFaults) #type: ignore
        self.parent_timer = parent_timer
        self.parent_timer.timeout.connect(self.updateWindow)
        self.ui.comboBox_ResetFaults.currentIndexChanged.connect(self.ROS.resetModeChanged)

        # Create a QTableWidget
        self.ui.tableWidget_MotorsInfo.setRowCount(len(self.ROS.joint_names))      # Set the number of rows
        self.ui.tableWidget_MotorsInfo.setColumnCount(4)  # Set the number of columns
        self.ui.tableWidget_MotorsInfo.setColumnWidth(0, 100)
        self.ui.tableWidget_MotorsInfo.setColumnWidth(1, 200)
        self.ui.tableWidget_MotorsInfo.setColumnWidth(2, 200)
        self.ui.tableWidget_MotorsInfo.setColumnWidth(3, 100)

    def resetFaults(self) -> None:
        if self.ui.comboBox_ResetFaults.currentIndex() == 2:
            print('Switch Off Logic Power')
            self.ROS.driveLogicSwitchOff()
            time.sleep(2.0)
            print('Switch On Logic Power')
            self.ROS.driveLogicSwitchOn()
        elif self.ui.comboBox_ResetFaults.currentIndex() == 3:
            print('Switch Off FT Sensor Power')
            self.ROS.FTSensorReset(0)
            time.sleep(2.0)
            self.ROS.FTSensorReset(1)
            print('Switch On FT Sensor Power')
        else:
            self.ROS.resetFaults()


    def resetModeChanged(self, index: int) -> None:
        self.ROS.resetModeChanged(index=index)

    def disconnect_ros(self):
        self.parent_timer.timeout.disconnect(self.updateWindow)
        self.ui.comboBox_ResetFaults.currentIndexChanged.connect(self.ROS.resetModeChanged)
        
    def updateWindow(self):

        if not self.ROS.isRosCommunicationActive():
            self.ui.lineEdit_Emergency.setText('Disconnected')
            self.ui.lineEdit_Emergency.setStyleSheet("background-color: rgb(255,215,00); color: white")
        else:
            if self.ROS.isEmergencyActive():
                self.ui.lineEdit_Emergency.setText('!!! Emergency !!!')
                self.ui.lineEdit_Emergency.setStyleSheet("background-color: red; color: white")
            else:      
                self.ui.pushButton_ResetFaults.setEnabled(self.ROS.isManualResetFaults())
                if self.ROS.isInFaultState():
                    self.ui.lineEdit_Emergency.setText("MOTORS FAULT")
                    self.ui.lineEdit_Emergency.setStyleSheet("background-color: red; color: white")
                else:
                    if self.ROS.areMotorsOn():
                        self.ui.lineEdit_Emergency.setText("Warning \n Motors On")
                        self.ui.lineEdit_Emergency.setStyleSheet("background-color: rgb(255,215,00); color: white")
                    else:
                        self.ui.lineEdit_Emergency.setText("Motors Off \n State OK")
                        self.ui.lineEdit_Emergency.setStyleSheet("background-color: green; color: white")

        self.ui.plainTextEdit_controller_name.setPlainText(self.ROS.getCurrentControllerName())
        self.ui.plainTextEdit_controller_name.show()
        # Fill the table with data

        for i,name in enumerate(self.ROS.getJointNames()):
            # 0 Joint Names
            item = QtWidgets.QTableWidgetItem(name)  #
            self.ui.tableWidget_MotorsInfo.setItem(i, 0, item)
        for i,state in enumerate(self.ROS.getDriversStates()):
            item = QtWidgets.QTableWidgetItem(state)  #
            self.ui.tableWidget_MotorsInfo.setItem(i, 1, item)
        for i,moo in enumerate(self.ROS.getDriversModeOfOperations()):
            item = QtWidgets.QTableWidgetItem(moo)  #
            self.ui.tableWidget_MotorsInfo.setItem(i, 2, item)
        for i,q in enumerate(self.ROS.getDriversFeedbackPosition()):
            item = QtWidgets.QTableWidgetItem(f'{q:.3f}')  #
            self.ui.tableWidget_MotorsInfo.setItem(i, 3, item)

