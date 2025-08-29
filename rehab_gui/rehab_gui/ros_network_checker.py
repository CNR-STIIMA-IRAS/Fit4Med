import os
import sys
import signal
import time
from PyQt5 import QtWidgets, QtCore 
from PyQt5.QtWidgets import QMessageBox, QLabel, QPushButton, QDialog, QVBoxLayout
from PyQt5.QtCore import QTimer, QObject, QThread, pyqtSignal, Qt
from functools import partial
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rclpy.executors import ExternalShutdownException

class WaitingDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Alert")
        self.setModal(True)
        self.setWindowFlags(Qt.Dialog | Qt.WindowStaysOnTopHint)

        label = QLabel("Waiting for ROS connection...")
        label.setAlignment(Qt.AlignCenter)

        layout = QVBoxLayout()
        layout.addWidget(label)
        self.setLayout(layout)
        self.resize(300, 100)

class ROSNetworkChecker(QObject):
    ros_ready = pyqtSignal(bool)  # True = ROS attivo, False = ROS non attivo

    def __init__(self, ros_node: Node, interval_sec=0.5):
        super().__init__()
        self.ros_node = ros_node
        self.context_alive = True  # Flag per sapere se ROS è vivo

        # Timer Qt, così il check viene sempre fatto anche se ROS è spento
        self.check_timer = QTimer(self)
        self.check_timer.timeout.connect(self.check)
        self.check_timer.start(int(interval_sec * 1000))

        print('[ROSNetworkChecker] initialized.')

    def check(self):
        """Controlla se ROS è vivo e se il nodo 'state_controller' è presente."""
        # print('[ROSNetworkChecker] checking ROS status...' )
        ready = False
        if self.context_alive:
            try:
                nodes = list(self.ros_node.get_node_names())
                ready = 'state_controller' in nodes
            except ExternalShutdownException:
                # ROS è stato spento → segnala False ma continua il timer
                print('[ROSNetworkChecker] ROS shutdown detected.')
                self.context_alive = False
                ready = False
            except Exception as e:
                # Altri errori (es. nodo non raggiungibile)
                print(f'[ROSNetworkChecker] Exception: {e}')
                ready = False

            self.ros_ready.emit(ready)

    def stop(self):
        """Da chiamare quando vogliamo chiudere il checker (es. in closeEvent)."""
        print('[ROSNetworkChecker] stopping...')
        self.check_timer.stop()
