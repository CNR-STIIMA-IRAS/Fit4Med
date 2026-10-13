# EtherCAT runtime safety stop

`plc_manager` now monitors both `/tecnobody/get_slave_states_master0` (PLC)
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
identify a module omitted from the very first response. Driver source and
revision are needed to close that verification. No upstream packages changed.
