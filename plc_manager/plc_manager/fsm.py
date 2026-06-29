from enum import Enum, auto
from dataclasses import dataclass
from typing import Callable, Optional
from rclpy.impl.rcutils_logger import RcutilsLogger as Logger

from plc_manager.utils import bcolors


class InvalidTransition(Exception):
    pass


class GuardFailed(Exception):
    pass


class TransitionTimeout(Exception):
    pass


class State(Enum):
    IDLE = auto()
    IDLE_RECOVERY = auto()
    ESTOP = auto()
    RUNNING = auto()
    RUNNING_RECOVERY = auto()
    RECOVERED = auto()
    ERROR = auto()

class Event(Enum):
    SWITCH_MODE = auto()
    START = auto()
    STOP = auto()
    FAIL = auto()
    NONE = auto()

class TransitionStatus(Enum):
    COMPLETED = auto()
    PENDING = auto()
    TIMEOUT = auto()


@dataclass
class Transition:
    destination: State
    guard: Optional[Callable[[], bool]] = None
    action: Optional[Callable[[], None]] = None
    success_check: Optional[Callable[[], bool]] = None
    max_steps: int = 1
    failure_destination: Optional[State] = State.ERROR
    msg: str = ""


@dataclass
class PendingTransition:
    source: State
    event: Event
    transition: Transition
    steps: int = 0


class StateMachine:
    def __init__(self, initial_state: State, logger: Logger | None = None) -> None:
        self.state = initial_state
        self.transitions: dict[tuple[State, Event], Transition] = {}
        self.pending: Optional[PendingTransition] = None
        self.msg = ""
        self.logger = logger

    def _debug_transition(
        self,
        prefix: str,
        source: State,
        event: Event,
        destination: State,
    ) -> None:
        msg = f"[FSM DEBUG] {prefix}: {source.name} --{event.name}--> {destination.name}"
        if self.logger:
            self.logger.info(msg) #type: ignore
        else:
            print(msg)

    def add_transition(
        self,
        event: Event,
        source: State,
        destination: State,
        *,
        msg: str,
        guard: Optional[Callable[[], bool]] = None,
        action: Optional[Callable[[], None]] = None,
        success_check: Optional[Callable[[], bool]] = None,
        max_steps: int = 1,
        failure_destination: Optional[State] = State.ERROR,
    ) -> None:
        if max_steps < 1:
            raise ValueError("max_steps must be at least 1")
        
        self.transitions[(source, event)] = Transition(
            destination=destination,
            guard=guard,
            action=action,
            success_check=success_check,
            max_steps=max_steps,
            failure_destination=failure_destination,
            msg=msg
        )
        self._debug_transition("transition created", source, event, destination)

    def trigger(self, event: Event) -> TransitionStatus:
        if self.pending is not None:
            raise InvalidTransition(
                "Cannot trigger a new event while another transition is pending"
            )

        key = (self.state, event)
        if key not in self.transitions:
            raise InvalidTransition(f"No transition for {self.state.name} + {event.name}")

        transition = self.transitions[key]
        self._debug_transition(
            "transition called",
            self.state,
            event,
            transition.destination,
        )

        if self.logger:
            self.logger.info( #type: ignore
                bcolors.OKBLUE +
                transition.msg +
                bcolors.ENDC
            )
        else:
            print(
                bcolors.OKBLUE +
                transition.msg +
                bcolors.ENDC
            )

        if transition.guard and not transition.guard():
            raise GuardFailed(f"Guard failed for {self.state.name} + {event.name}")

        if transition.action:
            transition.action()

        if transition.success_check is None:
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

        if transition.success_check is not None and transition.success_check():
            self._complete(pending.event, transition)
            self.pending = None

            _msg = bcolors.OKGREEN + "✅ Transition Completed: " + transition.msg + bcolors.ENDC
            if self.logger: #type: ignore
                self.logger.info(_msg ) #type: ignore
            else:
                print(_msg)

            return TransitionStatus.COMPLETED

        pending.steps += 1
        if pending.steps < transition.max_steps:
            return TransitionStatus.PENDING
        
        _msg = bcolors.FAIL + "❌ timeout" + transition.msg + bcolors.ENDC
        if self.logger: #type: ignore
            self.logger.error(  _msg ) #type: ignore
        else:
            print(_msg)


        self.pending = None
        if transition.failure_destination is not None:
            self.state = transition.failure_destination

        raise TransitionTimeout(
            f"{pending.source.name} --{pending.event.name}--> "
            f"{transition.destination.name} failed after "
            f"{transition.max_steps} steps"
        )

    def cancel_pending(self) -> None:
        self.pending = None

    def _complete(self, event: Event, transition: Transition) -> None:
        old_state = self.state
        self.state = transition.destination
        print(f"{old_state.name} --{event.name}--> {self.state.name}")
