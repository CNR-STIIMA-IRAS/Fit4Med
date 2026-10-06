# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from plc_manager.plc_commands import (
    PLC_COMMAND_INTERFACE_NAMES,
    PlcCommandPublisher,
)


class FakePublisher:
    """Capture published PLC command messages."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        """Store a compact copy of the published message."""
        self.messages.append((list(msg.interface_names), list(msg.values)))


class FakeLogger:
    """Capture log output emitted by PlcCommandPublisher."""

    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message):
        """Store info log messages."""
        self.infos.append(message)

    def warn(self, message, throttle_duration_sec=None):
        """Store warning log messages."""
        self.warnings.append((message, throttle_duration_sec))


def test_command_interface_names_match_plc_configuration():
    """Command names should match the configured PLC command interfaces."""
    assert 'PLC_node/estop_bypass' in PLC_COMMAND_INTERFACE_NAMES
    assert 'PLC_node/s_output.4' not in PLC_COMMAND_INTERFACE_NAMES


class FakeClock:
    """Monotonic clock moved by hand."""

    def __init__(self):
        self.now = 100.0

    def __call__(self):
        """Return the current fake time."""
        return self.now


def test_repeated_commands_are_not_republished():
    """Repeated identical commands should not produce extra ROS publishes."""
    publisher = FakePublisher()
    commands = PlcCommandPublisher(publisher, FakeLogger(), clock=FakeClock())

    commands.set_automatic_mode()
    commands.set_automatic_mode()
    assert len(publisher.messages) == 1

    commands.clear_sw_estop()
    commands.clear_sw_estop()
    assert len(publisher.messages) == 2


def test_unchanged_commands_are_republished_after_the_period():
    """A lost command must be recovered by the periodic refresh."""
    publisher = FakePublisher()
    logger = FakeLogger()
    clock = FakeClock()
    commands = PlcCommandPublisher(
        publisher, logger, republish_period_sec=0.5, clock=clock)

    commands.clear_sw_estop()
    assert len(publisher.messages) == 1
    assert len(logger.infos) == 1

    clock.now += 0.4
    commands.clear_sw_estop()
    assert len(publisher.messages) == 1

    clock.now += 0.2
    commands.clear_sw_estop()
    assert len(publisher.messages) == 2
    assert publisher.messages[1] == publisher.messages[0]
    assert len(logger.infos) == 1  # the refresh is not logged

    clock.now += 0.1
    commands.clear_sw_estop()
    assert len(publisher.messages) == 2  # period restarts from the refresh


def test_changed_and_forced_commands_are_published_at_once():
    """Changes and force_print commands ignore the refresh period."""
    publisher = FakePublisher()
    logger = FakeLogger()
    commands = PlcCommandPublisher(publisher, logger, clock=FakeClock())

    commands.clear_sw_estop()
    commands.raise_sw_estop()
    commands.wire_endstroke_to_emergency_chain()
    commands.wire_endstroke_to_emergency_chain()
    assert len(publisher.messages) == 4
    assert len(logger.infos) == 4


def test_unknown_interface_is_not_published():
    """A command for an unknown interface only warns."""
    publisher = FakePublisher()
    logger = FakeLogger()
    commands = PlcCommandPublisher(publisher, logger, clock=FakeClock())

    commands._publish_command('PLC_node/does_not_exist', 1)
    assert publisher.messages == []
    assert len(logger.warnings) == 1
