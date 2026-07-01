# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from typing import Any

from tecnobody_workbench_utils.fsm import StateMachine

from plc_manager.plc_types import Event, State


def build_plc_fsm(controller: Any) -> StateMachine[State, Event]:
    environment = controller.environment
    plc_commands = controller.plc_commands

    fsm: StateMachine[State, Event] = StateMachine[State, Event](
        State.IDLE,
        controller.get_logger(),
    )

    fsm.add_transition(
        Event.SWITCH_MODE,
        State.IDLE,
        State.IDLE_RECOVERY,
        action=lambda: plc_commands.publish_command('PLC_node/z_recovery', 0),
        msg="🔑 Change mode (IDLE=>IDLE_RECOVERY)",
    )
    fsm.add_transition(
        Event.SWITCH_MODE,
        State.RECOVERED,
        State.IDLE,
        msg="🔑 Change mode (RECOVERED=>IDLE)",
    )
    fsm.add_transition(
        Event.START,
        State.IDLE,
        State.RUNNING,
        msg="🔑 KEY TURN DETECTED: E-stop 0=>1 (IDLE=>RUNNING)",
        guard=controller._ready_to_start,
        action=environment.bringup_env,
        success_check=environment.check_env_running,
        max_steps=50000,
        failure_destination=State.ERROR,
    )
    fsm.add_transition(
        Event.START,
        State.IDLE_RECOVERY,
        State.RUNNING_RECOVERY,
        msg="🔑 KEY TURN DETECTED: E-stop 0=>1 (IDLE_RECOVERY=>RUNNING_RECOVERY)",
        guard=controller._ready_to_start,
        action=environment.bringup_recovery_env,
        success_check=environment.check_env_running_recovery,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.STOP,
        State.RUNNING_RECOVERY,
        State.RECOVERED,
        msg="🛑 EMERGENCY STOP REQUESTED (RUNNING_RECOVERY=>RECOVERED)",
        action=environment.kill_recovery_env,
        success_check=environment.check_env_running_recovery_stopped,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.STOP,
        State.RUNNING,
        State.IDLE,
        msg="🛑 EMERGENCY STOP REQUESTED (RUNNING=>IDLE)",
        action=environment.kill_env,
        success_check=environment.check_env_running_stopped,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.FAIL,
        State.RUNNING,
        State.ERROR,
        msg="� SYSTEM FAILURE (RUNNING=>ERROR)",
        action=environment.kill_env,
        success_check=environment.check_env_running_stopped,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.FAIL,
        State.RUNNING_RECOVERY,
        State.ERROR,
        msg="� SYSTEM FAILURE (RUNNING_RECOVERY=>ERROR)",
        action=environment.kill_recovery_env,
        success_check=environment.check_env_running_recovery_stopped,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.STOP,
        State.IDLE,
        State.IDLE,
        msg="🛑 EMERGENCY STOP REQUESTED (IDLE=>IDLE)",
        action=environment.handle_idle_stop,
        success_check=environment.check_env_running_stopped,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.STOP,
        State.IDLE_RECOVERY,
        State.IDLE_RECOVERY,
        msg="🛑 EMERGENCY STOP REQUESTED (IDLE_RECOVERY=>IDLE_RECOVERY)",
        action=environment.handle_idle_stop,
        success_check=environment.check_env_running_recovery_stopped,
        max_steps=50000,
        failure_destination=State.ERROR,
    )
    
    fsm.add_transition(
        Event.STOP,
        State.ERROR,
        State.ERROR,
        msg="🛑 EMERGENCY STOP REQUESTED (ERROR=>ERROR)",
        action=environment.handle_idle_stop,
        max_steps=50000,
        failure_destination=State.ERROR,
    )


    return fsm
