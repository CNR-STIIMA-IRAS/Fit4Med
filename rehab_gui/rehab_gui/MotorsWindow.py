# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

import json
import time
from datetime import datetime
from typing import Dict

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QTimer

from ui.uiMotorsWindow import Ui_MotorsWindow
from RosCommunicationManager import RosCommunicationManager

class MotorsWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MotorsWindow()
        self.ui.setupUi(self)  #type:ignore # Set up the UI for the secondary widget
        self._last_plc_state = None
        self._last_plc_pending = None
        self._plc_outputs = []
        self._last_udp_received_monotonic: float | None = None
        self._last_udp_received_wall_time: str | None = None
        self._last_udp_display_message = "No UDP message received yet"
        self._plc_udp_watchdog_active = False

    def connect(self, ROS: RosCommunicationManager, parent_timer: QTimer):
        self.ROS : RosCommunicationManager = ROS
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
        previous_plc_status = (self._last_plc_state, self._last_plc_pending)
        first_udp_message = self._last_udp_received_monotonic is None
        self._last_udp_received_monotonic = time.monotonic()
        self._last_udp_received_wall_time = self._format_wall_time()
        if first_udp_message or self._plc_udp_watchdog_active:
            self._plc_udp_watchdog_active = False
            self._last_emergency_state = None
        message = self._format_udp_message(data)
        self._last_udp_display_message = message
        self._refresh_udp_status_text()
        current_plc_status = (self._last_plc_state, self._last_plc_pending)
        if current_plc_status != previous_plc_status:
            self._last_emergency_state = None

    @staticmethod
    def _format_wall_time() -> str:
        return datetime.now().strftime("%H:%M:%S")

    def _refresh_udp_status_text(self) -> None:
        last_udp_time = self._last_udp_received_wall_time or "never"
        self.ui.plainTextEdit_udp_channel.setPlainText(
            "Current time: "
            + self._format_wall_time()
            + " Last UDP received: "
            + last_udp_time
            + "\nROBOT State: "
            + self._last_udp_display_message
        )

    def updateUdpWatchdog(self, timeout_s: float) -> None:
        if self._last_udp_received_monotonic is None:
            watchdog_active = False
        else:
            watchdog_active = (
                time.monotonic() - self._last_udp_received_monotonic
            ) > timeout_s
        if watchdog_active != self._plc_udp_watchdog_active:
            self._plc_udp_watchdog_active = watchdog_active
            self._last_emergency_state = None

        self._refresh_udp_status_text()

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
        if not self._plc_outputs:
            return

        plc_outputs = payload.get("plc_outputs")
        if not isinstance(plc_outputs, dict):
            return

        plc_outputs_interface_names = plc_outputs.get("interface_names")
        values = plc_outputs.get("values")
        if not isinstance(plc_outputs_interface_names, list) or not isinstance(values, list):
            return

        plc_outputs_rows = self.ui.tableWidget_PLC_Outputs.rowCount()
        plc_outputs_max_commands = plc_outputs_rows * 2
        for _idx in range(plc_outputs_max_commands):
            row = _idx % plc_outputs_rows
            column = 0 if _idx < plc_outputs_rows else 2
            name_text = ""
            value_text = ""

            if _idx < len(plc_outputs_interface_names) and _idx < len(values):
                name = plc_outputs_interface_names[_idx]
                if isinstance(name, str):
                    name_text = self._format_plc_interface_name(name)
                value_text = self._format_plc_value(values[_idx])

            self._plc_outputs[row][column].setText(name_text)
            self._plc_outputs[row][column + 1].setText(value_text)

        plc_inputs= payload.get("plc_inputs")
        if not isinstance(plc_inputs, dict):
            return

        plc_inputs_interface_names = plc_inputs.get("interface_names")
        values = plc_inputs.get("values")
        if not isinstance(plc_inputs_interface_names, list) or not isinstance(values, list):
            return

        plc_inputs_rows = self.ui.tableWidget_PLC_Inputs.rowCount()
        plc_inputs_max_commands = plc_inputs_rows * 2
        for _idx in range(plc_inputs_max_commands):
            row = _idx % plc_inputs_rows
            column = 0 if _idx < plc_inputs_rows else 2
            name_text = ""
            value_text = ""

            if _idx < len(plc_inputs_interface_names) and _idx < len(values):
                name = plc_inputs_interface_names[_idx]
                if isinstance(name, str):
                    name_text = self._format_plc_interface_name(name)
                value_text = self._format_plc_value(values[_idx])

            self._plc_inputs[row][column].setText(name_text)
            self._plc_inputs[row][column + 1].setText(value_text)

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
        if self._last_udp_received_monotonic is None:
            state_key = 'waiting_fmrr_bringup'
        elif self._plc_udp_watchdog_active:
            state_key = 'plc_udp_timeout'
        elif not self.ROS.isRosCommunicationActive():
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
            if state_key == 'waiting_fmrr_bringup':
                self.ui.label_SystemState.setText('Waiting for the FMRR bring-up')
                self.ui.label_SystemState.setStyleSheet("background-color: rgb(255,140,0); color: white")
            elif state_key == 'plc_udp_timeout':
                self.ui.label_SystemState.setText('PLC communication lost\nReboot all the system')
                self.ui.label_SystemState.setStyleSheet("background-color: red; color: white")
            elif state_key == 'disconnected':
                if self._last_plc_pending is None:
                    if self._last_plc_state in ("IDLE",):
                        self.ui.label_SystemState.setText('Turn the Key to Start')
                        self.ui.label_SystemState.setStyleSheet("background-color: rgb(0,128,0); color: white")
                    elif self._last_plc_state in ("IDLE_RECOVERY",):
                        self.ui.label_SystemState.setText('Turn the Key to Start Recovery  \n !!!SMALL MOVEMENT EXPECTED!!!')
                        self.ui.label_SystemState.setStyleSheet("background-color: rgb(255,150,00); color: white")
                else:
                    if self._last_plc_pending.get("source") == "IDLE":
                        self.ui.label_SystemState.setText('CoE Drivers and Controllers Bringing-up')
                        self.ui.label_SystemState.setStyleSheet("background-color: rgb(255,215,00); color: white")
                    elif self._last_plc_pending.get("source") == "IDLE_RECOVERY":
                        self.ui.label_SystemState.setText('Recovery Bringing-up \n !!!SMALL MOVEMENT EXPECTED!!!')
                        self.ui.label_SystemState.setStyleSheet("background-color: rgb(255,140,00); color: white")
            elif state_key == 'emergency':
                if self._last_plc_pending is not None\
                    and self._last_plc_pending.get("source") in ("IDLE", "IDLE_RECOVERY"):
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

        for i, name in enumerate(self.ROS.getSlaveNames()):
            self._ec_items[i][0].setText(name)
        for i, state in enumerate(self.ROS.getSlaveStates()):
            self._ec_items[i][1].setText(state)
