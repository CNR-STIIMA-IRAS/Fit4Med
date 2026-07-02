# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from enum import Enum, auto


class EStopState(Enum):
    OK = 1
    EMERGENCY = 0


class EthercatCheckState(Enum):
    IDLE = 0
    PENDING = 1
    READY = 2
    FAILED = 3


class State(Enum):
    IDLE = auto()
    IDLE_RECOVERY = auto()
    ESTOP = auto()
    RUNNING = auto()
    RUNNING_RECOVERY = auto()
    RECOVERED = auto()
    ERROR_RECOVERY = auto()
    ERROR = auto()


class Event(Enum):
    SWITCH_MODE = auto()
    START = auto()
    STOP = auto()
    FAIL = auto()
    NONE = auto()
