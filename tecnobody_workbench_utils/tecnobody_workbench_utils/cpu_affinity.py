# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
"""CPU affinity of the non real-time processes.

The kernel command line isolates some CPUs (isolcpus) for the EtherCAT loops
of ros2_control. Everything else (Python nodes, launch processes, rosbridge)
belongs on the other CPUs, the "housekeeping" ones. The set is read from sysfs,
so it follows the kernel command line and no CPU number is written here.

Call pin_to_housekeeping_cpus() before creating threads (rclpy nodes,
executors): sched_setaffinity applies to the calling thread, and the threads
and processes created afterwards inherit it.
"""

import os
import sys

SYSFS_CPU_DIR = '/sys/devices/system/cpu'


def parse_cpu_list(text: str) -> set[int]:
    """Parse a kernel CPU list such as '0-1,4-5' or '2,3' ('' gives no CPU)."""
    cpus: set[int] = set()
    for chunk in text.strip().split(','):
        chunk = chunk.strip()
        if not chunk:
            continue
        if '-' in chunk:
            first_text, last_text = chunk.split('-', 1)
            first, last = int(first_text), int(last_text)
            if last < first:
                raise ValueError(f"CPU range '{chunk}' is not ascending")
            cpus.update(range(first, last + 1))
        else:
            cpus.add(int(chunk))
    return cpus


def _read_sysfs_cpu_list(name: str, sysfs_dir: str) -> set[int]:
    with open(os.path.join(sysfs_dir, name), encoding='ascii') as cpu_file:
        return parse_cpu_list(cpu_file.read())


def housekeeping_cpus(sysfs_dir: str = SYSFS_CPU_DIR) -> set[int]:
    """Online CPUs not isolated by the kernel. Raises if there are none."""
    cpus = (
        _read_sysfs_cpu_list('online', sysfs_dir)
        - _read_sysfs_cpu_list('isolated', sysfs_dir)
    )
    if not cpus:
        raise RuntimeError('every online CPU is isolated')
    return cpus


def pin_to_housekeeping_cpus(process_name: str) -> set[int] | None:
    """Move the calling thread to the housekeeping CPUs.

    Returns the CPUs, or None if they cannot be set: the process then keeps
    the affinity it inherited, and a warning goes to stderr (this runs before
    the ROS logger exists).
    """
    try:
        cpus = housekeeping_cpus()
        os.sched_setaffinity(0, cpus)
    except (OSError, ValueError, RuntimeError) as exc:
        print(
            f'[{process_name}] WARNING: cannot move to the housekeeping CPUs '
            f'({exc}); running on CPUs {sorted(os.sched_getaffinity(0))}.',
            file=sys.stderr,
        )
        return None
    return cpus
