# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

import json
from typing import Dict

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QTimer, Qt

from ui.uiMotorsWindow import Ui_MotorsWindow
from RosCommunicationManager import RosCommunicationManager

class MotorsWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MotorsWindow()
        self.ui.setupUi(self)  #type:ignore # Set up the UI for the secondary widget
        self._last_plc_state = None
        self._last_plc_pending = None
        self._plc_items = []

    def connect(self, ROS: RosCommunicationManager, parent_timer: QTimer):
        self.ROS : RosCommunicationManager = ROS
        self.ui.pushButton_ResetFaults.clicked.connect(self.resetFaults) #type: ignore
        self.parent_timer = parent_timer
        self.ui.comboBox_ResetFaults.currentIndexChanged.connect(self.ROS.resetModeChanged)

        
        self.ui.textEdit_SystemState.setReadOnly(True)
        self.ui.textEdit_SystemState.setAlignment(
            Qt.Alignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
        )


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

        self.ui.tableWidget_PLC.setRowCount(5)      # Set the number of rows
        self.ui.tableWidget_PLC.setColumnCount(4)  # Set the number of columns
        self.ui.tableWidget_PLC.setColumnWidth(0, 250)
        self.ui.tableWidget_PLC.setColumnWidth(1, 50)
        self.ui.tableWidget_PLC.setColumnWidth(2, 250)
        self.ui.tableWidget_PLC.setColumnWidth(3, 50)

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

        self._plc_items = []
        for i in range(5):
            row_items = []
            for j in range(4):
                item = QtWidgets.QTableWidgetItem('')
                if j in (1, 3):
                    item.setTextAlignment(QtCore.Qt.AlignCenter)
                self.ui.tableWidget_PLC.setItem(i, j, item)
                row_items.append(item)
            self._plc_items.append(row_items)

        # Cache for emergency state to avoid redundant setText/setStyleSheet
        self._last_emergency_state = None
        self._last_controller_name = None

    @QtCore.pyqtSlot(bytes, tuple)
    def onUdpMessageReceived(self, data, addr):
        previous_plc_status = (self._last_plc_state, self._last_plc_pending)
        message = self._format_udp_message(data)
        self.ui.plainTextEdit_udp_channel.setPlainText("ROBOT State: " + message)
        current_plc_status = (self._last_plc_state, self._last_plc_pending)
        if current_plc_status != previous_plc_status:
            self._last_emergency_state = None

    def _format_udp_message(self, data: bytes) -> str:
        message = data.decode("utf-8", errors="replace")
        try:
            payload = json.loads(message)
        except json.JSONDecodeError:
            self._last_plc_state = message.strip()
            self._last_plc_pending = None
            return message

        if not isinstance(payload, dict):
            self._last_plc_state = message.strip()
            self._last_plc_pending = None
            return message

        if payload.get("schema") != "fit4med.plc_fsm_status.v1":
            self._last_plc_state = message.strip()
            self._last_plc_pending = None
            return message

        self._update_plc_table(payload)

        state = payload.get("state")
        state = state if isinstance(state, str) else ""
        self._last_plc_state = state

        pending = payload.get("pending")
        if not isinstance(pending, dict):
            self._last_plc_pending = None
            return state
        self._last_plc_pending = pending

        event = pending.get("event")
        source = pending.get("source")
        target = pending.get("target")
        steps = pending.get("steps")

        event = event if isinstance(event, str) else "UNKNOWN"
        source = source if isinstance(source, str) else "UNKNOWN"
        target = target if isinstance(target, str) else "UNKNOWN"
        steps_text = f" [{steps}]" if steps is not None else ""

        return f"{state} | pending {event}: {source} -> {target}{steps_text}"

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

    def _update_plc_table(self, payload: Dict[str, object]) -> None:
        if not self._plc_items:
            return

        command_msg = payload.get("command_msg")
        if not isinstance(command_msg, dict):
            return

        interface_names = command_msg.get("interface_names")
        values = command_msg.get("values")
        if not isinstance(interface_names, list) or not isinstance(values, list):
            return

        rows = self.ui.tableWidget_PLC.rowCount()
        max_commands = rows * 2
        for command_idx in range(max_commands):
            row = command_idx % rows
            column = 0 if command_idx < rows else 2
            name_text = ""
            value_text = ""

            if command_idx < len(interface_names) and command_idx < len(values):
                name = interface_names[command_idx]
                if isinstance(name, str):
                    name_text = self._format_plc_interface_name(name)
                value_text = self._format_plc_value(values[command_idx])

            self._plc_items[row][column].setText(name_text)
            self._plc_items[row][column + 1].setText(value_text)

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
            if state_key == 'disconnected':
                if self._last_plc_pending is None:
                    if self._last_plc_state in ("IDLE",):
                        self.ui.textEdit_SystemState.setPlainText('Turn the Key to Start')
                        self.ui.textEdit_SystemState.setStyleSheet("background-color: rgb(0,128,0); color: white")
                    elif self._last_plc_state in ("IDLE_RECOVERY",):
                        self.ui.textEdit_SystemState.setPlainText('Turn the Key to Start Recovery  \n !!!SMALL MOVEMENT EXPECTED!!!')
                        self.ui.textEdit_SystemState.setStyleSheet("background-color: rgb(255,150,00); color: white")
                else:
                    if self._last_plc_pending.get("source") == "IDLE":
                        self.ui.textEdit_SystemState.setPlainText('CoE Drivers and Controllers Bring-up')
                        self.ui.textEdit_SystemState.setStyleSheet("background-color: rgb(255,215,00); color: white")
                    elif self._last_plc_pending.get("source") == "IDLE_RECOVERY":
                        self.ui.textEdit_SystemState.setPlainText('Recovery Bring-up \n !!!SMALL MOVEMENT EXPECTED!!!')
                        self.ui.textEdit_SystemState.setStyleSheet("background-color: rgb(255,140,00); color: white")
            elif state_key == 'emergency':
                if self._last_plc_pending is not None\
                    and self._last_plc_pending.get("source") in ("IDLE", "IDLE_RECOVERY"):
                    pass
                self.ui.textEdit_SystemState.setPlainText('!!! Emergency !!!')
                self.ui.textEdit_SystemState.setStyleSheet("background-color: red; color: white")
            elif state_key == 'fault':
                self.ui.textEdit_SystemState.setPlainText("MOTORS FAULT")
                self.ui.textEdit_SystemState.setStyleSheet("background-color: red; color: white")
            elif state_key == 'no_mode':
                self.ui.textEdit_SystemState.setPlainText("CoE Drivers with No Mode Set")
                self.ui.textEdit_SystemState.setStyleSheet("background-color: rgb(255,140,0); color: white")
            elif state_key == 'motors_on':
                self.ui.textEdit_SystemState.setPlainText("Warning \n Motors On")
                self.ui.textEdit_SystemState.setStyleSheet("background-color: rgb(255,215,00); color: white")
            elif state_key == 'suspended':
                self.ui.textEdit_SystemState.setPlainText("SUSPENSION STATE")
                self.ui.textEdit_SystemState.setStyleSheet("background-color: rgb(255,140,0); color: white")
            else:
                self.ui.textEdit_SystemState.setPlainText("Motors Off \n State OK")
                self.ui.textEdit_SystemState.setStyleSheet("background-color: green; color: white")

        if state_key not in ('disconnected', 'emergency'):
            self.ui.pushButton_ResetFaults.setEnabled(self.ROS.isManualResetFaults())

        ctrl_name = self.ROS.getCurrentControllerName()
        if ctrl_name != self._last_controller_name:
            self._last_controller_name = ctrl_name
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

        for i, name in enumerate(self.ROS.getSlaveNames()):
            self._ec_items[i][0].setText(name)
        for i, state in enumerate(self.ROS.getSlaveStates()):
            self._ec_items[i][1].setText(state)
