"""Small Qt task runner: GUI continuations stay on the GUI thread.

Only explicitly yielded Call objects run in a QThread. No extra dependencies.
A stop cancels further ordinary commands; the polling thread stays independent.
"""
from functools import wraps
import inspect
import threading
import traceback
from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot

class Call:
    def __init__(self, function, *args, **kwargs):
        self.function, self.args, self.kwargs = function, args, kwargs

class CallThread(QThread):
    def __init__(self, call, parent, cancel=None):
        super().__init__(parent)
        self.call = call
        self.cancel = cancel or threading.Event()
        self.value = None
        self.error = None

    def run(self):
        from sync_ros_events import command_context
        command_context.cancel = self.cancel
        try:
            self.value = self.call.function(*self.call.args, **self.call.kwargs)
        except Exception as exc:
            self.error = exc
            traceback.print_exc()
        finally:
            command_context.cancel = None

class GuiTask(QObject):
    finished = pyqtSignal()
    failed = pyqtSignal(str)

    def __init__(self, generator, parent):
        super().__init__(parent)
        self.generator = generator
        self.cancel = threading.Event()
        self.worker = None
        self.result = None

    def start(self):
        self._advance()

    def _advance(self, value=None, error=None):
        try:
            if self.cancel.is_set():
                self.generator.close()  # finally blocks run on the GUI thread.
                self.finished.emit()
                return
            call = self.generator.throw(error) if error is not None else self.generator.send(value)
            if not isinstance(call, Call):
                raise TypeError('GUI task must yield a Call')
            self.worker = CallThread(call, self, self.cancel)
            self.worker.finished.connect(self._worker_finished)
            self.worker.start()
        except StopIteration as done:
            self.result = done.value
            self.finished.emit()
        except Exception as exc:
            traceback.print_exc()
            self.failed.emit(str(exc))
            self.finished.emit()

    @pyqtSlot()
    def _worker_finished(self):
        worker = self.worker
        self.worker = None
        value, error = worker.value, worker.error
        worker.deleteLater()
        self._advance(value, error)


def gui_task(function):
    """Public slot starts a task; internal calls use yield from __wrapped__.

    Qt signals (e.g. QPushButton.clicked) always pass their own arguments
    (a `checked` bool) to the connected slot. PyQt5 normally trims those
    extra arguments to match the slot's arity, but it inspects the actual
    callable it is given -- here that is always this `start` wrapper, whose
    signature is `(self, *args, **kwargs)`. Seeing a varargs signature, PyQt5
    assumes the slot accepts anything and forwards every signal argument,
    which then gets passed straight through to `function` even if `function`
    declares no such parameter (TypeError: takes 1 positional argument but 2
    were given). So the trimming has to happen here instead, based on how
    many positional parameters `function` itself actually declares.
    """
    sig = inspect.signature(function)
    params = list(sig.parameters.values())[1:]  # drop `self`
    has_varargs = any(p.kind is p.VAR_POSITIONAL for p in params)
    max_positional = sum(1 for p in params if p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD))

    @wraps(function)
    def start(self, *args, **kwargs):
        manager = self.ROS
        call_args = args if has_varargs else args[:max_positional]
        generator = function(self, *call_args, **kwargs)
        if not inspect.isgenerator(generator):
            return generator
        return manager.startGuiTask(generator)
    return start
