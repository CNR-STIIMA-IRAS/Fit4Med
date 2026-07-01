# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from datetime import datetime

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QTimer

from ui.uiMotorsWindow import Ui_MotorsWindow
from RosCommunicationManager import RosCommunicationManager
from UdpCommunicationManager import UdpCommunicationManager

class MotorsWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MotorsWindow()
        self.ui.setupUi(self)  #type:ignore # Set up the UI for the secondary widget
        self._plc_outputs = []
        self._last_plc_status_snapshot = (None, None)
        self._plc_udp_watchdog_active = False

    def connect(self, ROS: RosCommunicationManager, UDP: UdpCommunicationManager, parent_timer: QTimer):
        self.ROS : RosCommunicationManager = ROS
        self.UDP : UdpCommunicationManager = UDP
        self.ui.pushButton_ResetFaults.clicked.connect(self.resetFaults) #type: ignore
        self.parent_timer = parent_timer
        self.ui.comboBox_ResetFaults.currentIndexChanged.connect(self.ROS.resetModeChanged)

        
        self.ui.label_SystemState.setAlignment(QtCore.Qt.AlignCenter)


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

        self.ui.tableWidget_PLC_Outputs.setRowCount(5)      # Set the number of rows
        self.ui.tableWidget_PLC_Outputs.setColumnCount(4)  # Set the number of columns
        self.ui.tableWidget_PLC_Outputs.setColumnWidth(0, 250)
        self.ui.tableWidget_PLC_Outputs.setColumnWidth(1, 50)
        self.ui.tableWidget_PLC_Outputs.setColumnWidth(2, 250)
        self.ui.tableWidget_PLC_Outputs.setColumnWidth(3, 50)

        self.ui.tableWidget_PLC_Inputs.setRowCount(5)      # Set the number of rows
        self.ui.tableWidget_PLC_Inputs.setColumnCount(4)  # Set the number of columns
        self.ui.tableWidget_PLC_Inputs.setColumnWidth(0, 250)
        self.ui.tableWidget_PLC_Inputs.setColumnWidth(1, 50)
        self.ui.tableWidget_PLC_Inputs.setColumnWidth(2, 250)
        self.ui.tableWidget_PLC_Inputs.setColumnWidth(3, 50)

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

        self._plc_outputs = []
        for i in range(5):
            row_items = []
            for j in range(4):
                item = QtWidgets.QTableWidgetItem('')
                if j in (1, 3):
                    item.setTextAlignment(QtCore.Qt.AlignCenter)
                self.ui.tableWidget_PLC_Outputs.setItem(i, j, item)
                row_items.append(item)
            self._plc_outputs.append(row_items)

        self._plc_inputs = []
        for i in range(5):
            row_items = []
            for j in range(4):
                item = QtWidgets.QTableWidgetItem('')
                if j in (1, 3):
                    item.setTextAlignment(QtCore.Qt.AlignCenter)
                self.ui.tableWidget_PLC_Inputs.setItem(i, j, item)
                row_items.append(item)
            self._plc_inputs.append(row_items)

        # Cache for emergency state to avoid redundant setText/setStyleSheet
        self._last_emergency_state = None
        self._last_controller_name = None

    @QtCore.pyqtSlot(bytes, tuple)
    def onUdpMessageReceived(self, data, addr):
        current_plc_status = (self.UDP.getPlcState(), self.UDP.getPlcPending())
        if self._plc_udp_watchdog_active or current_plc_status != self._last_plc_status_snapshot:
            self._last_emergency_state = None
        self._plc_udp_watchdog_active = False
        self._last_plc_status_snapshot = current_plc_status
        self._update_plc_table()
        self._refresh_udp_status_text()

    @staticmethod
    def _format_wall_time() -> str:
        return datetime.now().strftime("%H:%M:%S")

    def _refresh_udp_status_text(self) -> None:
        last_udp_time = self.UDP.getLastUdpReceivedWallTime() or "never"
        self.ui.plainTextEdit_udp_channel.setPlainText(
            "Current time: "
            + self._format_wall_time()
            + " Last UDP received: "
            + last_udp_time
            + "\nROBOT State: "
            + self.UDP.getUdpDisplayMessage()
        )

    def updateUdpWatchdog(self, timeout_s: float) -> None:
        watchdog_active = self.UDP.isUdpWatchdogExpired(timeout_s)
        if watchdog_active != self._plc_udp_watchdog_active:
            self._plc_udp_watchdog_active = watchdog_active
            self._last_emergency_state = None

        self._refresh_udp_status_text()

    @staticmethod
    def _format_plc_interface_name(name: str) -> str:
        if name.startswith("PLC_node/"):
            return name[len("PLC_node/"):]
        if name.startswith("PLC_node"):
            return name[len("PLC_node"):].lstrip("/")
        return name

    @staticmethod
    def _format_plc_value(value) -> str:
        if isinstance(value, str):
            return "1" if value.strip().lower() in ("1", "true", "yes", "on") else "0"
        return "1" if bool(value) else "0"

    def _update_table_items(self, table_items, source_items) -> None:
        rows = len(table_items)
        if rows == 0:
            return

        max_items = rows * 2
        for _idx in range(max_items):
            row = _idx % rows
            column = 0 if _idx < rows else 2
            name_text = ""
            value_text = ""

            if _idx < len(source_items):
                name, value = source_items[_idx]
                name_text = self._format_plc_interface_name(name)
                value_text = self._format_plc_value(value)

            table_items[row][column].setText(name_text)
            table_items[row][column + 1].setText(value_text)

    def _update_plc_table(self) -> None:
        if not self._plc_outputs:
            return

        self._update_table_items(self._plc_outputs, self.UDP.getPLCOutputItems())
        self._update_table_items(self._plc_inputs, self.UDP.getPLCInputItems())

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
        plc_state = self.UDP.getPlcState()
        plc_pending = self.UDP.getPlcPending()
        if self.UDP.getLastUdpReceivedMonotonic() is None:
            state_key = 'waiting_fmrr_bringup'
        elif self._plc_udp_watchdog_active:
            state_key = 'plc_udp_timeout'
        elif not self.ROS.isRosCommunicationActive():
            state_key = 'disconnected'
        elif self.UDP.isEmergencyActive():
            state_key = 'emergency'
        elif self.ROS.isInFaultState():
            state_key = 'fault'
        elif not self.ROS.isModeSet():
            state_key = 'no_mode'
        elif self.ROS.areMotorsOn():
            state_key = 'motors_on'
        elif self.ROS.isExerciseInSuspension():
            state_key = 'suspended'
        else:
            state_key = 'motors_off'

        if state_key != self._last_emergency_state:
            self._last_emergency_state = state_key
            if state_key == 'waiting_fmrr_bringup':
                self.ui.label_SystemState.setText('Waiting for the FMRR bring-up')
                self.ui.label_SystemState.setStyleSheet("background-color: rgb(255,140,0); color: white")
            elif state_key == 'plc_udp_timeout':
                self.ui.label_SystemState.setText('PLC communication lost\nReboot all the system')
                self.ui.label_SystemState.setStyleSheet("background-color: red; color: white")
            elif state_key == 'disconnected':
                if plc_pending is None:
                    if plc_state in ("IDLE",):
                        self.ui.label_SystemState.setText('Clear the Emergencies and \n Turn the Key to Reset and Start')
                        self.ui.label_SystemState.setStyleSheet("background-color: rgb(0,128,0); color: white")
                    elif plc_state in ("IDLE_RECOVERY",):
                        self.ui.label_SystemState.setText('Turn the Key to Start Recovery  \n !!!SMALL MOVEMENT EXPECTED!!!')
                        self.ui.label_SystemState.setStyleSheet("background-color: rgb(255,150,00); color: white")
                else:
                    if plc_pending.get("source") == "IDLE":
                        self.ui.label_SystemState.setText('CoE Drivers and Controllers Bringing-up')
                        self.ui.label_SystemState.setStyleSheet("background-color: rgb(255,215,00); color: white")
                    elif plc_pending.get("source") == "IDLE_RECOVERY":
                        self.ui.label_SystemState.setText('Recovery Bringing-up \n !!!SMALL MOVEMENT EXPECTED!!!')
                        self.ui.label_SystemState.setStyleSheet("background-color: rgb(255,140,00); color: white")
            elif state_key == 'emergency':
                if plc_pending is not None\
                    and plc_pending.get("source") in ("IDLE", "IDLE_RECOVERY"):
                    pass
                self.ui.label_SystemState.setText('!!! Emergency !!!')
                self.ui.label_SystemState.setStyleSheet("background-color: red; color: white")
            elif state_key == 'fault':
                self.ui.label_SystemState.setText("MOTORS FAULT")
                self.ui.label_SystemState.setStyleSheet("background-color: red; color: white")
            elif state_key == 'no_mode':
                self.ui.label_SystemState.setText("CoE Drivers Up with No Mode Set")
                self.ui.label_SystemState.setStyleSheet("background-color: rgb(255,140,0); color: white")
            elif state_key == 'motors_on':
                self.ui.label_SystemState.setText("Warning \n Motors On")
                self.ui.label_SystemState.setStyleSheet("background-color: rgb(255,215,00); color: white")
            elif state_key == 'suspended':
                self.ui.label_SystemState.setText("SUSPENSION STATE")
                self.ui.label_SystemState.setStyleSheet("background-color: rgb(255,140,0); color: white")
            else:
                self.ui.label_SystemState.setText("Motors Off \n State OK")
                self.ui.label_SystemState.setStyleSheet("background-color: green; color: white")

        if state_key not in ('waiting_fmrr_bringup', 'plc_udp_timeout', 'disconnected', 'emergency'):
            self.ui.pushButton_ResetFaults.setEnabled(self.ROS.isManualResetFaults())

        ctrl_name = self.ROS.getCurrentControllerName()
        if ctrl_name != self._last_controller_name:
            self._last_controller_name = ctrl_name
            if ctrl_name is None:
                self.ui.plainTextEdit_controller_name.setPlainText("CTRL: None")
            else:
                self.ui.plainTextEdit_controller_name.setPlainText("CTRL: "+ ctrl_name)

        # Update table items in-place (setText instead of creating new QTableWidgetItems)
        for i, name in enumerate(self.ROS.getJointNames()):
            self._motor_items[i][0].setText(name)
        for i, state in enumerate(self.ROS.getDriversStates()):
            self._motor_items[i][1].setText(state)
        for i, moo in enumerate(self.ROS.getDriversModeOfOperations()):
            self._motor_items[i][2].setText(moo)
        for i, q in enumerate(self.ROS.getDriversFeedbackPosition()):
            self._motor_items[i][3].setText(f'{q:.3f}')

        for i, name in enumerate(self.UDP.getSlaveNames()):
            self._ec_items[i][0].setText(name)
        for i, state in enumerate(self.UDP.getSlaveStates()):
            self._ec_items[i][1].setText(state)
