from typing import Callable, Generic, Iterable, Optional, TypeVar
from enum import Enum, auto

from dataclasses import dataclass
from rclpy.impl.rcutils_logger import RcutilsLogger as Logger

from tecnobody_workbench_utils.utils import bcolors


class InvalidTransition(Exception):
    pass


class GuardFailed(Exception):
    pass


class TransitionTimeout(Exception):
    pass


class TransitionFailed(TransitionTimeout):
    pass


StateT = TypeVar("StateT", bound=Enum)
EventT = TypeVar("EventT", bound=Enum)
CallbackT = TypeVar("CallbackT")


class TransitionStatus(Enum):
    COMPLETED = auto()
    PENDING = auto()
    TIMEOUT = auto()


@dataclass
class Transition(Generic[StateT]):
    destination: StateT
    guards: tuple[Callable[[], bool], ...] = ()
    actions: tuple[Callable[[], None], ...] = ()
    success_checks: tuple[Callable[[], bool], ...] = ()
    timeout_actions: tuple[Callable[[], None], ...] = ()
    max_steps: int = 1
    failure_destination: Optional[StateT] = None
    msg: str = ""


@dataclass
class PendingTransition(Generic[StateT, EventT]):
    source: StateT
    event: EventT
    transition: Transition[StateT]
    steps: int = 0


class StateMachine(Generic[StateT, EventT]):
    def __init__(self, initial_state: StateT, logger: Logger | None = None) -> None:
        self.state = initial_state
        self.transitions: dict[tuple[StateT, EventT], Transition[StateT]] = {}
        self.pending: Optional[PendingTransition[StateT, EventT]] = None
        self.msg = ""
        self.logger = logger

    def _debug_transition(
        self,
        prefix: str,
        source: StateT,
        event: EventT,
        destination: StateT,
        info: str = "",
    ) -> None:
        msg = f"[FSM DEBUG] {prefix}: {source.name} + {event.name} --> {destination.name} [{info}]"
        if self.logger:
            self.logger.info(msg) #type: ignore
        else:
            print(msg)

    def add_transition(
        self,
        event: EventT,
        source: StateT,
        destination: StateT,
        *,
        guard: Optional[Callable[[], bool]] = None,
        guards: Optional[Iterable[Callable[[], bool]]] = None,
        action: Optional[Callable[[], None]] = None,
        actions: Optional[Iterable[Callable[[], None]]] = None,
        success_check: Optional[Callable[[], bool]] = None,
        success_checks: Optional[Iterable[Callable[[], bool]]] = None,
        timeout_action: Optional[Callable[[], None]] = None,
        timeout_actions: Optional[Iterable[Callable[[], None]]] = None,
        max_steps: int = 1,
        failure_destination: Optional[StateT] = None,
    ) -> None:
        if max_steps < 1:
            raise ValueError("max_steps must be at least 1")

        self.transitions[(source, event)] = Transition(
            destination=destination,
            guards=self._ordered_callbacks(guard, guards, "guard", "guards"),
            actions=self._ordered_callbacks(action, actions, "action", "actions"),
            success_checks=self._ordered_callbacks(
                success_check,
                success_checks,
                "success_check",
                "success_checks",
            ),
            timeout_actions=self._ordered_callbacks(
                timeout_action,
                timeout_actions,
                "timeout_action",
                "timeout_actions",
            ),
            max_steps=max_steps,
            failure_destination=failure_destination,
            msg="",
        )
        self._debug_transition("transition created", source, event, destination)

    @staticmethod
    def _ordered_callbacks(
        callback: Optional[CallbackT],
        callbacks: Optional[Iterable[CallbackT]],
        callback_name: str,
        callbacks_name: str,
    ) -> tuple[CallbackT, ...]:
        if callback is not None and callbacks is not None:
            raise ValueError(
                f"Use either {callback_name} or {callbacks_name}, not both"
            )
        if callback is not None:
            return (callback,)
        if callbacks is None:
            return ()
        return tuple(callbacks)

    def _log_error(self, msg: str) -> None:
        if self.logger:
            self.logger.error(msg)  # type: ignore
        else:
            print(msg)

    def _run_timeout_actions(self, transition: Transition[StateT]) -> None:
        for timeout_action in transition.timeout_actions:
            try:
                timeout_action()
            except Exception as e:
                self._log_error(
                    f"{transition.msg} Transition timeout action failed "
                    f"🚨with error: {e}"
                )

    def _fail_transition(
        self,
        source: StateT,
        event: EventT,
        transition: Transition[StateT],
        reason: str,
        failure_type: type[TransitionTimeout] = TransitionFailed,
        exception: Exception | None = None,
    ) -> None:
        self.pending = None
        if transition.failure_destination is not None:
            self.state = transition.failure_destination

        self._run_timeout_actions(transition)

        failure = failure_type(reason)
        if exception is not None:
            raise failure from exception
        raise failure

    def trigger(self, event: EventT, msg: str = "") -> TransitionStatus:
        if self.pending is not None:
            raise InvalidTransition(
                "Cannot trigger a new event while another transition is pending"
            )

        source = self.state
        key = (self.state, event)
        if key not in self.transitions:
            raise InvalidTransition(
                f"No transition for {self.state.name} + {event.name} [{msg}]"
            )

        transition = self.transitions[key]
        transition.msg = (
            f"[{self.state.name} + {event.name} => "
            f"{transition.destination.name} ] {msg}"
        )
        self._debug_transition(
            "transition called",
            self.state,
            event,
            transition.destination,
            info=msg
        )

        _msg = (
            bcolors.OKBLUE
            + transition.msg
            + "Transition Started ⚠️"
            + bcolors.ENDC
        )
        if self.logger:
            self.logger.info(_msg) #type: ignore
        else:
            print(_msg)

        if not all(guard() for guard in transition.guards):
            raise GuardFailed(f"{transition.msg} Transition Guard failed 🚨!")

        try:
            for action in transition.actions:
                action()
        except Exception as e:
            self._fail_transition(
                source,
                event,
                transition,
                (
                    f"{source.name} --{event.name}--> "
                    f"{transition.destination.name} action failed: {e}"
                ),
                exception=e,
            )

        if not transition.success_checks:
            self._complete(event, transition)
            return TransitionStatus.COMPLETED

        self.pending = PendingTransition(
            source=self.state,
            event=event,
            transition=transition,
        )
        return TransitionStatus.PENDING

    def step(self) -> Optional[TransitionStatus]:
        if self.pending is None:
            return None

        pending = self.pending
        transition = pending.transition

        success = False
        try:
            success = bool(
                transition.success_checks
                and all(success_check() for success_check in transition.success_checks)
            )
        except Exception as e:
            self._log_error(
                f"{transition.msg} Transition success check failed with error: {e}"
            )

        if success:
            self._complete(pending.event, transition)
            self.pending = None

            _msg = (
                bcolors.OKGREEN
                + transition.msg
                + "Transition Completed ✅"
                + bcolors.ENDC
            )
            if self.logger: #type: ignore
                self.logger.info(_msg ) #type: ignore
            else:
                print(_msg)

            return TransitionStatus.COMPLETED

        pending.steps += 1
        if pending.steps < transition.max_steps:
            return TransitionStatus.PENDING
        
        _msg = bcolors.FAIL + transition.msg + "Transition ❌" + bcolors.ENDC
        if self.logger: #type: ignore
            self.logger.error(  _msg ) #type: ignore
        else:
            print(_msg)


        self.pending = None
        if transition.failure_destination is not None:
            self.state = transition.failure_destination

        timeout_error = TransitionTimeout(
            f"{pending.source.name} --{pending.event.name}--> "
            f"{transition.destination.name} failed after "
            f"{transition.max_steps} steps"
        )
        self._fail_transition(
            pending.source,
            pending.event,
            transition,
            str(timeout_error),
            failure_type=TransitionTimeout,
        )

    def cancel_pending(self) -> None:
        self.pending = None

    def _complete(self, event: EventT, transition: Transition[StateT]) -> None:
        self.state = transition.destination
