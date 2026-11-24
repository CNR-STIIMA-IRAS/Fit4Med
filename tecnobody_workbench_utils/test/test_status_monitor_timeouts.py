# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
"""Lost service responses must not block the START readiness monitors.

Run 0097: the controller manager logged "failed to send response to
/controller_manager/list_controllers (timeout)"; the pending request of
plc_manager never completed and no new one was sent, so START waited with no
log until the emergency stop.
"""

from types import SimpleNamespace

import pytest

from tecnobody_workbench_utils import utils


class FakeFuture:

    def __init__(self):
        self._done = False
        self._result = None
        self._callbacks = []
        self.cancelled = False

    def done(self):
        return self._done

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def cancel(self):
        self.cancelled = True

    def set_result(self, result):
        self._result = result
        self._done = True
        for callback in self._callbacks:
            callback(self)


class FakeClient:

    def __init__(self):
        self.futures = []
        self.removed = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = FakeFuture()
        future.request = request
        self.futures.append(future)
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


class FakeNode:

    def __init__(self):
        self.clients = []

    def create_client(self, *args, **kwargs):
        client = FakeClient()
        self.clients.append(client)
        return client

    def create_subscription(self, *args, **kwargs):
        return object()


class FakeLogger:

    def __init__(self):
        self.warnings = []

    def info(self, message):
        pass

    def warn(self, message):
        self.warnings.append(message)


@pytest.fixture
def clock(monkeypatch):
    now = {'t': 1000.0}
    monkeypatch.setattr(utils.time, 'monotonic', lambda: now['t'])
    return now


def _controller(name, state):
    return SimpleNamespace(name=name, state=state)


def test_lost_list_controllers_response_is_requested_again(clock):
    pytest.importorskip('controller_manager_msgs.srv')
    node, logger = FakeNode(), FakeLogger()
    monitor = utils.ControllerManagerStatusMonitor(
        node, 'controller_manager', active_controllers=['a'], logger=logger)
    client = node.clients[0]

    assert not monitor.ok()
    assert len(client.futures) == 1
    lost = client.futures[0]

    # Before the timeout: still waiting, no second request.
    clock['t'] += 0.5
    monitor.ok()
    assert len(client.futures) == 1

    # Response lost: after the timeout the request is sent again.
    clock['t'] += 0.6
    monitor.ok()
    assert len(client.futures) == 2
    assert client.removed == [lost] and lost.cancelled
    assert any('no response' in w for w in logger.warnings)

    # The new request is answered: the monitor reports OK.
    client.futures[1].set_result(SimpleNamespace(controller=[_controller('a', 'active')]))
    assert monitor.ok()


def test_late_response_of_abandoned_request_is_ignored(clock):
    pytest.importorskip('controller_manager_msgs.srv')
    node = FakeNode()
    monitor = utils.ControllerManagerStatusMonitor(
        node, 'controller_manager', active_controllers=['a'])
    client = node.clients[0]
    monitor.ok()
    clock['t'] += 1.1
    monitor.ok()
    assert len(client.futures) == 2
    # The abandoned request answers late (wrong state): not used.
    client.futures[0].set_result(SimpleNamespace(controller=[_controller('a', 'inactive')]))
    client.futures[1].set_result(SimpleNamespace(controller=[_controller('a', 'active')]))
    assert monitor.ok()


def test_lost_mode_switch_response_is_requested_again(clock):
    pytest.importorskip('ethercat_controller_msgs.srv')
    node, logger = FakeNode(), FakeLogger()
    monitor = utils.ModeOfOperationMonitor(
        node, dof_names=['joint_x'], target_mode=8, logger=logger)
    client = node.clients[0]

    monitor.ok()
    assert len(client.futures) == 1
    clock['t'] += 0.5
    monitor.ok()
    assert len(client.futures) == 1

    clock['t'] += 0.6
    monitor.ok()
    assert len(client.futures) == 2
    assert client.removed == [client.futures[0]]
    assert any('no response' in w for w in logger.warnings)
