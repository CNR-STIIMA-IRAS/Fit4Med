# EtherCAT runtime safety stop

## Fast path: driver bus supervision

`ethercat_driver_ros2` supervises the bus inside `read()`, every control cycle.
While a bus fault is active it reports status word `0x0008` on every exported
`status_word` state interface. CiA402 consumers decode it as FAULT; no communicating
drive produces it (bit 9, remote, is always set), so it identifies a bus fault. The
`state_controller` publishes it on `/state_controller/drive_states`, and
`plc_manager` raises `ETHERCAT_FAULT` on it within one 50 ms safety tick.

Fault causes, all armed only after the domain working counter was complete once
(startup is ignored):

| Cause | Default | Hardware `<param>` |
|---|---|---|
| Working counter incomplete for N cycles | 3 cycles (12 ms) | `bus_fault_wc_cycles` |
| Slave offline or not OP (after it was OP) | refreshed every 10 cycles | — |
| Counter not advancing (`counter_watchdog` on a component, e.g. ATI `Sample_Counter`) | 50 ms | `counter_watchdog_timeout` |
| Single control-cycle gap | 50 ms, keep below the drive SM watchdog | `cycle_fault_max_gap` |
| Slow cycles (> `cycle_slow_factor` × period) in a window | > 20 % in 1 s | `cycle_fault_ratio`, `cycle_fault_window` |

The fault is reported for at least `bus_fault_hold_time` (1 s) after the cause
disappears, so a sampled supervisor cannot miss a short glitch. Slow cycles
are logged (throttled) even below the fault ratio. The driver does not change
any command: the PLC emergency chain is responsible for the safe state.

Powering off the drive logic or the FT sensor from the GUI during RUNNING takes
slaves offline and therefore triggers this stop, as the polling path below does.

## Backstop: GetSlaveStates polling

`plc_manager` also monitors both `/tecnobody/get_slave_states_master0` (PLC)
and `/tecnobody/get_slave_states_master1` (platform). Override the
`ethercat_slave_state_services` parameter for other installations; every
configured service must reply successfully. A completed non-OP result,
a missing previously observed slave, unavailable service, service timeout,
or stale telemetry triggers `ETHERCAT_FAULT` in RUNNING and RUNNING_RECOVERY.
A poll in progress retains the previous completed health result.

A separate 50 ms timer detects faults and advances shutdown without depending
on PLC input messages. Detection still depends on ROS scheduling and the
GetSlaveStates implementation. Defaults remain 1 s polling, 3 s response timeout,
5 s stale/initial telemetry deadline. These are supervisory timings, not
hard real-time safety guarantees. Startup is handled by the existing startup
and controller readiness checks; runtime monitoring starts in RUNNING.

The fault transition commands the Z brake closed before opening software
ESTOP, stops the corresponding environment, and holds software ESTOP open.
The diagnostic is retained in `ethercat.latched_fault` in UDP status even
when controllers and rosbridge are gone. The GUI stops ROS communication,
shows a nonblocking critical fault dialog, and retains the reason in its
PLC status display. Closing the dialog does not acknowledge the fault.

After shutdown completes and PLC feedback confirms emergency, the software
command is released using the existing PLC emergency-latch procedure. The
operator must re-arm the machine using the existing physical controls; there
is no additional ROS reset service. The existing START guard requires a fresh
startup check, and the fault diagnostic is cleared only by a guarded START.
That startup check verifies the **PLC**; it does not certify the entire motor
bus while the motor driver is stopped. Runtime checks resume in RUNNING.
The diagnostic latch is process-local and does not survive a manager restart.

## Validation before machine use

Run the regression suite with
`python plc_manager/test/test_ethercat_safety.py` and ROS/Qt integration tests
in the target workspace. Bench-test motor-bus and PLC-bus disconnections,
service timeouts, lost PLC feedback, recovery mode, reconnect without operator rearm,
and physical rearm/restart. Verify actual brake and STO behavior and timings.
ROS command publication cannot prove actuation if the PLC bus itself is lost;
the PLC/hardware watchdog must enforce the safe state independently.

Review the installed `ethercat_driver_ros2` revision to verify that
GetSlaveStates reflects current online/operational status, including working
counter faults, and reports every configured module before the first healthy
snapshot. The learned runtime slave set detects later disappearance but cannot
identify a module omitted from the very first response.

Before enabling the counter watchdog on a new sensor, verify that its counter
advances at least once per `counter_watchdog_timeout` in normal operation.
Read the drives' SM watchdog (`ethercat reg_read -p<pos> 0x0420 2` and divider
`0x0400`) and keep `cycle_fault_max_gap` well below it.
