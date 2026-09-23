"""Validated YAML I/O and reversible GUI updates (Python 3.8+)."""
import hashlib
import logging
import math
import numbers
import os
from pathlib import Path
import tempfile
from contextlib import contextmanager
from collections.abc import Mapping
import yaml
try:
    from yaml import CSafeLoader as BaseLoader
except ImportError:
    from yaml import SafeLoader as BaseLoader

log = logging.getLogger(__name__)

class YamlDataError(ValueError):
    pass

class UniqueSafeLoader(BaseLoader):
    """Reject duplicate keys instead of silently keeping the last value."""
    def construct_mapping(self, node, deep=False):
        self.flatten_mapping(node)
        result = {}
        for key_node, value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            try:
                if key in result:
                    raise YamlDataError("Chiave YAML duplicata: {!r}, riga {}".format(key, key_node.start_mark.line + 1))
                result[key] = self.construct_object(value_node, deep=deep)
            except TypeError as exc:
                raise YamlDataError("Chiave YAML non valida alla riga {}".format(key_node.start_mark.line + 1)) from exc
        return result


def read_yaml(path, validator):
    """Read one byte snapshot; log its identity even when parsing fails."""
    path = Path(path)
    digest = 'non disponibile'
    try:
        with path.open('rb') as stream:
            before = os.fstat(stream.fileno())
            raw = stream.read()
            after = os.fstat(stream.fileno())
        digest = hashlib.sha256(raw).hexdigest()
        log.info('YAML read path=%s bytes=%d sha256=%s', path.resolve(), len(raw), digest)
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise YamlDataError('Il file è cambiato durante la lettura. Riprovare quando il salvataggio è terminato.')
        data = yaml.load(raw, Loader=UniqueSafeLoader)  # safe constructors only
        return validator(data)
    except Exception as exc:
        log.exception('YAML load failed path=%s sha256=%s', path, digest)
        raise YamlDataError('{}\n\nFile: {}\nSHA-256: {}'.format(exc, path, digest)) from exc


def mapping(value, path):
    if not isinstance(value, Mapping):
        raise YamlDataError(path + ': atteso un dizionario non vuoto, non un file vuoto o una lista.')
    return value


def array(value, path, count=None, minimum=None):
    if not isinstance(value, list):
        raise YamlDataError(path + ': attesa una lista.')
    if count is not None and len(value) != count:
        raise YamlDataError('{}: attesi {} elementi, ricevuti {}.'.format(path, count, len(value)))
    if minimum is not None and len(value) < minimum:
        raise YamlDataError('{}: servono almeno {} elementi.'.format(path, minimum))
    return value


def number(value, path, minimum=None, positive=False, integer=False):
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise YamlDataError(path + ': atteso un numero, non testo o booleano.')
    try:
        finite = math.isfinite(value)
    except (OverflowError, ValueError):
        finite = False
    if not finite:
        raise YamlDataError(path + ': NaN, infinito o numero troppo grande non ammessi.')
    if positive and value <= 0 or minimum is not None and value < minimum:
        raise YamlDataError(path + ': valore fuori intervallo.')
    if integer and int(value) != value:
        raise YamlDataError(path + ': atteso un numero intero.')
    return value


def singleton(section, key, prefix):
    return array(section.get(key), prefix + '.' + key, count=1)[0]


def validate_movement(data, execution=False):
    mapping(data, 'movimento')
    meta = mapping(data.get('a_movement_definition'), 'a_movement_definition')
    for key, values in [('type', (0, 1, 2)), ('side', (0, 1, 2)), ('vel_profile', (1, 2))]:
        value = singleton(meta, key, 'a_movement_definition')
        number(value, key, integer=True)
        if value not in values:
            raise YamlDataError('{}: valori ammessi {}.'.format(key, values))
    number(singleton(meta, 'total_time', 'a_movement_definition'), 'total_time', positive=True)
    number(singleton(meta, 'max_velocity', 'a_movement_definition'), 'max_velocity', minimum=0)
    for key in ('begin_config', 'end_config', 'begin_joint_config'):
        vector = array(singleton(meta, key, 'a_movement_definition'), key, minimum=3)
        for i, value in enumerate(vector):
            number(value, '{}[{}]'.format(key, i))
    trajectory = mapping(data.get('cart_trj3'), 'cart_trj3')
    positions = array(trajectory.get('cart_positions'), 'cart_trj3.cart_positions', minimum=2)
    times = array(trajectory.get('time_from_start'), 'cart_trj3.time_from_start', count=len(positions))
    previous = -1
    for i, (position, stamp) in enumerate(zip(positions, times)):
        for j, value in enumerate(array(position, 'cart_positions[{}]'.format(i), count=3)):
            number(value, 'cart_positions[{}][{}]'.format(i, j))
        value = number(array(stamp, 'time_from_start[{}]'.format(i), count=1)[0],
                       'time_from_start[{}][0]'.format(i), minimum=0)
        if value <= previous:
            raise YamlDataError('time_from_start: i tempi devono essere strettamente crescenti (indice {}).'.format(i))
        previous = value
    if 'joint_names' in trajectory:
        names = array(trajectory['joint_names'], 'cart_trj3.joint_names', count=3)
        if any(not isinstance(n, str) or not n for n in names) or len(set(names)) != 3:
            raise YamlDataError('joint_names: servono tre nomi distinti non vuoti.')
    if execution:
        # Preserve the original GUI checks, but perform them before any mutation.
        if not all(abs(x * 100) < 1e-4 for x in meta['begin_config'][0][:3]):
            raise YamlDataError('begin_config: la posizione iniziale deve essere zero.')
        if all(abs(x * 100) < 1 for x in meta['end_config'][0][:3]):
            raise YamlDataError('end_config: posizione finale troppo vicina allo zero (controllo preesistente della GUI).')
    return data


def validate_protocol(data):
    mapping(data, 'protocollo')
    phases = mapping(data.get('Phases'), 'Phases')
    for key in ('PhaseIsEnabled', 'Modalities', 'Percentage'):
        values = array(singleton(phases, key, 'Phases'), 'Phases.' + key + '[0]', count=20)
        for i, value in enumerate(values):
            path = 'Phases.{}[0][{}]'.format(key, i)
            if key == 'PhaseIsEnabled':
                if not isinstance(value, (bool, int)) or value not in (0, 1):
                    raise YamlDataError(path + ': atteso 0/1 o false/true.')
            else:
                number(value, path, minimum=0 if key == 'Percentage' else None, integer=True)
    if 'Duration' in phases:
        for i, value in enumerate(array(singleton(phases, 'Duration', 'Phases'), 'Duration[0]', count=20)):
            number(value, 'Duration[0][{}]'.format(i), minimum=0, integer=True)
    for key in ('V_max', 'PhaseDuration'):
        if key in data:
            number(singleton(data, key, 'protocollo'), key, minimum=0)
    return data


def plain_data(value):
    """Serialize NumPy arrays/scalars as ordinary YAML, never Python tags."""
    if hasattr(value, 'tolist'):
        return plain_data(value.tolist())
    if isinstance(value, Mapping):
        return {key: plain_data(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain_data(item) for item in value]
    return value


def atomic_save_yaml(path, data, validator):
    path = Path(path)
    candidate = validator(plain_data(data))
    # Serialize BEFORE touching the destination or allocating its replacement.
    payload = yaml.safe_dump(candidate, allow_unicode=True, sort_keys=False).encode('utf-8')
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='wb', dir=str(path.parent), prefix='.' + path.name + '.', suffix='.tmp', delete=False) as stream:
            temporary = stream.name
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
    except Exception:
        log.exception('YAML save failed: %s', path)
        raise
    finally:
        if temporary is not None:
            try:
                os.unlink(temporary)
            except OSError:
                log.warning('Could not remove temporary file %s', temporary)
    return path


def check_spin_value(widget, value, name):
    number(value, name)
    if not widget.minimum() <= value <= widget.maximum():
        raise YamlDataError('{}: {} fuori dai limiti del controllo [{}, {}].'.format(name, value, widget.minimum(), widget.maximum()))
    return value


@contextmanager
def gui_transaction(attributes, widgets):
    """Snapshot attributes and widget properties; rollback while signals blocked.

    attributes: [(object, [names])]; widgets: [(widget, getter_name, setter_name)].
    Must be entered on the GUI thread, with no nested event loop inside.
    """
    from PyQt5.QtCore import QSignalBlocker
    missing = object()
    old_attrs = [(obj, name, getattr(obj, name, missing)) for obj, names in attributes for name in names]
    old_widgets = [(widget, setter, getattr(widget, getter)()) for widget, getter, setter in widgets]
    unique = list({id(w): w for w, _, _ in widgets}.values())
    blockers = [QSignalBlocker(w) for w in unique]
    exclusive = [(w, w.autoExclusive()) for w in unique if hasattr(w, 'autoExclusive')]
    groups = {w.group() for w, _ in exclusive if w.group() is not None}
    group_states = [(g, g.exclusive()) for g in groups]
    for group, _ in group_states:
        group.setExclusive(False)
    for widget, _ in exclusive:
        widget.setAutoExclusive(False)
    try:
        yield
    except Exception:
        for obj, name, value in reversed(old_attrs):
            if value is missing:
                if hasattr(obj, name):
                    delattr(obj, name)
            else:
                setattr(obj, name, value)
        for widget, setter, value in old_widgets:
            getattr(widget, setter)(value)
        raise
    finally:
        for widget, value in exclusive:
            widget.setAutoExclusive(value)
        for group, value in group_states:
            group.setExclusive(value)
        for blocker in blockers:
            blocker.unblock()


def report_yaml_error(parent, path, exc):
    from PyQt5.QtWidgets import QMessageBox
    log.exception('YAML operation failed: %s', path)
    dialog = QMessageBox(parent)
    dialog.setIcon(QMessageBox.Critical)
    dialog.setWindowTitle('Caricamento / salvataggio YAML non riuscito')
    dialog.setText('Operazione YAML non riuscita. Consultare i dettagli per il campo o la causa dell’errore.')
    dialog.setInformativeText(str(exc).split('\n')[0])
    dialog.setDetailedText('File: {}\n\n{}: {}'.format(path, type(exc).__name__, exc))
    dialog.exec_()
