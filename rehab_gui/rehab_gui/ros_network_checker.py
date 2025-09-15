import os
import sys
import signal
import time
from PyQt5 import QtWidgets, QtCore 
from PyQt5.QtWidgets import QMessageBox, QLabel, QPushButton, QDialog, QVBoxLayout
from PyQt5.QtCore import QTimer, QObject, QThread, pyqtSignal, Qt
from functools import partial

class WaitingDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Alert")
        self.setModal(True)
        self.setWindowFlags(Qt.Dialog | Qt.WindowStaysOnTopHint)

        label = QLabel("Waiting for robot connection...")
        label.setAlignment(Qt.AlignCenter)

        layout = QVBoxLayout()
        layout.addWidget(label)
        self.setLayout(layout)
        self.resize(300, 100)
