# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from typing import Any

from tecnobody_workbench_utils.fsm import StateMachine

from plc_manager.plc_types import Event, State


def build_plc_fsm(controller: Any) -> StateMachine[State, Event]:
    environment = controller.environment
    plc_commands = controller.plc_commands
    startup_guards = (
        controller._ros_gui_disconnected,
        controller._ethercat_slaves_status_ok,
    )
    platform_bringup_actions = (
        environment.bringup_env,
        plc_commands.brake_enable,
        plc_commands.set_automatic_mode,
        plc_commands.wire_endstroke_to_emergency_chain,
        plc_commands.clear_sw_estop
    )
    recovery_bringup_actions = (
        environment.bringup_recovery_env,
        plc_commands.brake_enable,
        plc_commands.set_automatic_mode,
        plc_commands.detach_endstroke_from_emergency_chain,
        plc_commands.clear_sw_estop
    )
    platform_cleanup_actions = (
        environment.kill_env,
        plc_commands.set_automatic_mode,
        plc_commands.brake_enable,
        plc_commands.wire_endstroke_to_emergency_chain,
        plc_commands.clear_sw_estop
    )
    recovery_cleanup_actions = (
        environment.kill_recovery_env,
        plc_commands.set_automatic_mode,
        plc_commands.brake_enable,
        plc_commands.wire_endstroke_to_emergency_chain,
        plc_commands.clear_sw_estop
    )
    idle_stop_actions = (
        environment.handle_idle_stop,
        plc_commands.raise_sw_estop,
        plc_commands.set_automatic_mode,
        plc_commands.clear_sw_estop
    )

    fsm: StateMachine[State, Event] = StateMachine[State, Event](
        State.IDLE,
        controller.get_logger(),
    )

    fsm.add_transition(
        Event.SWITCH_MODE,
        State.IDLE,
        State.IDLE_RECOVERY,
        actions=(
            plc_commands.detach_endstroke_from_emergency_chain,
            plc_commands.clear_sw_estop
        ),
    )
    fsm.add_transition(
        Event.SWITCH_MODE,
        State.RECOVERED,
        State.IDLE,
        actions=(
            plc_commands.wire_endstroke_to_emergency_chain,
            plc_commands.clear_sw_estop
        ),
    )
    fsm.add_transition(
        Event.START,
        State.IDLE,
        State.RUNNING,
        guards=startup_guards,
        actions=platform_bringup_actions,
        success_checks=(environment.check_env_running,),
        timeout_actions=platform_cleanup_actions,
        max_steps=50000,
        failure_destination=State.ERROR,
    )
    fsm.add_transition(
        Event.START,
        State.IDLE_RECOVERY,
        State.RUNNING_RECOVERY,
        guards=startup_guards,
        actions=recovery_bringup_actions,
        success_checks=(environment.check_env_running_recovery,),
        timeout_actions=recovery_cleanup_actions,
        max_steps=50000,
        failure_destination=State.ERROR_RECOVERY,
    )

    fsm.add_transition(
        Event.STOP,
        State.RUNNING_RECOVERY,
        State.RECOVERED,
        actions=recovery_cleanup_actions,
        success_checks=(environment.check_env_running_recovery_stopped,),
        timeout_actions=recovery_cleanup_actions,
        max_steps=50000,
        failure_destination=State.ERROR_RECOVERY,
    )

    fsm.add_transition(
        Event.STOP,
        State.RUNNING,
        State.IDLE,
        actions=platform_cleanup_actions,
        success_checks=(environment.check_env_running_stopped,),
        timeout_actions=platform_cleanup_actions,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.FAIL,
        State.RUNNING,
        State.ERROR,
        actions=platform_cleanup_actions,
        success_checks=(environment.check_env_running_stopped,),
        timeout_actions=platform_cleanup_actions,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.FAIL,
        State.RUNNING_RECOVERY,
        State.ERROR_RECOVERY,
        actions=recovery_cleanup_actions,
        success_checks=(environment.check_env_running_recovery_stopped,),
        timeout_actions=recovery_cleanup_actions,
        max_steps=50000,
        failure_destination=State.ERROR_RECOVERY,
    )

    fsm.add_transition(
        Event.STOP,
        State.IDLE,
        State.IDLE,
        actions=idle_stop_actions,
        success_checks=(environment.check_env_running_stopped,),
        timeout_actions=platform_cleanup_actions,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.STOP,
        State.IDLE_RECOVERY,
        State.IDLE_RECOVERY,
        actions=idle_stop_actions,
        success_checks=(environment.check_env_running_recovery_stopped,),
        timeout_actions=recovery_cleanup_actions,
        max_steps=50000,
        failure_destination=State.ERROR_RECOVERY,
    )

    fsm.add_transition(
        Event.STOP,
        State.ERROR,
        State.ERROR,
        actions=idle_stop_actions,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    
    fsm.add_transition(
        Event.STOP,
        State.ERROR_RECOVERY,
        State.ERROR_RECOVERY,
        actions=idle_stop_actions,
        max_steps=50000,
        failure_destination=State.ERROR_RECOVERY,
    )

    fsm.add_transition(
        Event.START,
        State.ERROR,
        State.IDLE,
        actions=platform_cleanup_actions,
        max_steps=50000,
        failure_destination=State.ERROR,
    )

    fsm.add_transition(
        Event.START,
        State.ERROR_RECOVERY,
        State.IDLE_RECOVERY,
        actions=recovery_cleanup_actions,
        max_steps=50000,
        failure_destination=State.ERROR_RECOVERY,
    )



    return fsm
