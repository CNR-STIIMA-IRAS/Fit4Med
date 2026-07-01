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
        action=plc_commands.wire_endstroke_to_emergency_chain,
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
        action=controller._bringup_env,
        success_check=environment.check_env_running,
        timeout_action=controller._kill_env,
        max_steps=50000,
        failure_destination=State.ERROR,
    )
    fsm.add_transition(
        Event.START,
        State.IDLE_RECOVERY,
        State.RUNNING_RECOVERY,
        msg="🔑 KEY TURN DETECTED: E-stop 0=>1 (IDLE_RECOVERY=>RUNNING_RECOVERY)",
        guard=controller._ready_to_start,
        action=controller._bringup_recovery_env,
        success_check=environment.check_env_running_recovery,
        timeout_action=controller._kill_recovery_env,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.STOP,
        State.RUNNING_RECOVERY,
        State.RECOVERED,
        msg="🛑 EMERGENCY STOP REQUESTED (RUNNING_RECOVERY=>RECOVERED)",
        action=controller._kill_recovery_env,
        success_check=environment.check_env_running_recovery_stopped,
        timeout_action=controller._kill_recovery_env,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.STOP,
        State.RUNNING,
        State.IDLE,
        msg="🛑 EMERGENCY STOP REQUESTED (RUNNING=>IDLE)",
        action=controller._kill_env,
        success_check=environment.check_env_running_stopped,
        timeout_action=controller._kill_env,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.FAIL,
        State.RUNNING,
        State.ERROR,
        msg="� SYSTEM FAILURE (RUNNING=>ERROR)",
        action=controller._kill_env,
        success_check=environment.check_env_running_stopped,
        timeout_action=controller._kill_env,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.FAIL,
        State.RUNNING_RECOVERY,
        State.ERROR,
        msg="� SYSTEM FAILURE (RUNNING_RECOVERY=>ERROR)",
        action=controller._kill_recovery_env,
        success_check=environment.check_env_running_recovery_stopped,
        timeout_action=controller._kill_recovery_env,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.STOP,
        State.IDLE,
        State.IDLE,
        msg="🛑 EMERGENCY STOP REQUESTED (IDLE=>IDLE)",
        action=controller._handle_idle_stop,
        success_check=environment.check_env_running_stopped,
        timeout_action=controller._kill_env,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.STOP,
        State.IDLE_RECOVERY,
        State.IDLE_RECOVERY,
        msg="🛑 EMERGENCY STOP REQUESTED (IDLE_RECOVERY=>IDLE_RECOVERY)",
        action=controller._handle_idle_stop,
        success_check=environment.check_env_running_recovery_stopped,
        timeout_action=controller._kill_recovery_env,
        max_steps=50000,
        failure_destination=State.ERROR,
    )
    
    fsm.add_transition(
        Event.STOP,
        State.ERROR,
        State.ERROR,
        msg="🛑 EMERGENCY STOP REQUESTED (ERROR=>ERROR)",
        action=controller._handle_idle_stop,
        max_steps=50000,
        failure_destination=State.ERROR,
    )


    return fsm
