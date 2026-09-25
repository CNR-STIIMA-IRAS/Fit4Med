"""UDP status link with a simulated plc_manager on localhost sockets.
Run: QT_QPA_PLATFORM=offscreen python -m unittest discover -s tests -v
"""
import json
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import socket
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import time
import unittest

from PyQt5.QtCore import QCoreApplication

from UdpCommunicationManager import UdpCommunicationManager

APP = QCoreApplication.instance() or QCoreApplication([])


def status(state, pending=None):
    return json.dumps({"schema": "fit4med.plc_fsm_status.v1", "state": state,
                       "pending": pending}).encode()


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class UdpStatusLinkTest(unittest.TestCase):
    def setUp(self):
        self.port = free_port()
        self.udp = UdpCommunicationManager("127.0.0.1", self.port)
        self.received = []
        self.udp.udp_message_received.connect(lambda data, addr: self.received.append(data))
        self.starts = []
        self.udp.start_ros_communication.connect(lambda: self.starts.append(1))
        self.plc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.plc.bind(("127.0.0.1", 0))
        self.plc.settimeout(0.5)
        self.pump(0.3)  # let the UDP thread bind

    def tearDown(self):
        self.udp.shutdown()
        self.plc.close()

    def pump(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            APP.processEvents()
            time.sleep(0.005)

    def send(self, payload, settle=0.05):
        self.plc.sendto(payload, ("127.0.0.1", self.port))
        self.pump(settle)

    def replies(self):
        out = []
        self.plc.settimeout(0.01)
        try:
            while True:
                out.append(self.plc.recvfrom(4096)[0])
        except socket.timeout:
            pass
        return out

    def test_every_status_gets_the_current_ros_state(self):
        self.send(status("IDLE"))
        self.send(status("IDLE"), settle=0.2)
        self.assertEqual(self.replies(), [b"ROS_DISCONNECTED"] * 2)
        self.udp.onRosCommunicationEstablished()
        self.send(status("RUNNING"), settle=0.2)
        self.assertEqual(self.replies()[-1], b"ROS_CONNECTED")

    def test_repeats_are_rate_limited_changes_are_not(self):
        for _ in range(50):  # 50 identical packets in ~0.25 s
            self.plc.sendto(status("IDLE"), ("127.0.0.1", self.port))
            time.sleep(0.005)
        self.pump(0.3)
        repeats = len(self.received)
        self.assertGreaterEqual(repeats, 2)
        self.assertLessEqual(repeats, 6)
        self.assertEqual(self.received[-1], status("IDLE"))  # latest one is delivered
        self.send(status("ESTOP"), settle=0.03)  # a change goes through at once
        self.assertEqual(self.received[-1], status("ESTOP"))

    def test_failure_is_repeated_until_the_plc_leaves_running(self):
        self.send(status("RUNNING"))
        self.assertEqual(len(self.starts), 1)
        self.replies()  # drop the reply sent before the failure
        self.udp.onRosCommunicationFailed()
        self.udp.onResetRosCommunication()  # the GUI's ROS teardown must not hide it
        self.send(status("RUNNING"), settle=0.2)
        fail = {"event": "FAIL", "source": "RUNNING", "target": "IDLE", "steps": 1}
        self.send(status("RUNNING", fail), settle=0.2)
        self.assertEqual(len(self.starts), 1)  # no reconnection while FAILED
        self.assertTrue(all(r == b"ROS_CONNECTION_FAILED" for r in self.replies()))
        self.send(status("IDLE"), settle=0.2)
        self.assertEqual(self.replies()[-1], b"ROS_DISCONNECTED")


if __name__ == "__main__":
    unittest.main()
