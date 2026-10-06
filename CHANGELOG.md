# Changelog

Notable changes to the Fit4Med platform software. Newest first.

## 2026-10-07 — Brake closed whenever the drives lose torque

The Z axis falls by gravity when the drives have no torque and the brake is released. The brake is a safety-PLC output (`PLC_node/brake_disable`); the drives do not control it. It was released by `ethercat_checker` `start_motors` and closed only by `stop_motors` and by plc_manager state transitions, so several paths left the drives without torque and the brake released. This is the software side; the PLC/drive side (drive-controlled brake, or brake release interlocked with the drive state in the Flexi Soft) is still to be checked.

### Before deploying

- Rebuild `tecnobody_workbench_utils`, `tecnobody_workbench` and `plc_manager` on the robot.
- On the robot, with the motors on and the brake released, check that the brake closes (log `Brake closed: ...` of `ethercat_checker`):
  - on a drive fault (e.g. following error on X or Y);
  - when `ros2_control_node` of the platform is killed (`pkill -f ros2_control_node` is too broad: kill the platform one by PID); the platform launch must also shut down and plc_manager go to ESTOP;
  - at a normal stop of the environment.
- Check there is no false close during normal use, in particular when the GUI switches controller (mode of operation 8 ↔ 9): `drives_on` must stay true.
- Not tested with the real `ros2_control_node`: the launch shutdown on its exit is checked only by loading the launch description. The `OnShutdown` unspawners may wait for the dead controller manager and delay the end of the launch (the brake does not depend on it: `ethercat_checker` closes it as soon as it receives SIGINT).

### Fixed

- **`ethercat_checker` supervises the brake it released** (`eth_checker.py`). It closes it:
  - when the drives are no longer all enabled or report a fault: on a fault of any axis, the CiA402 state controller disables every enabled drive at once, Z included;
  - when the drive states (250 Hz) stop arriving for 0.2 s: `ros2_control_node` crashed or its loop stalled;
  - when the node exits (SIGINT/SIGTERM handled by the node, not by rclpy, so it can still publish).
- **Brake commands are sent 3 times, 20 ms apart**, instead of in a busy loop for 1 s. `PLC_controller` keeps only the last message per cycle: for that whole second the loop undid any brake close sent by plc_manager (an emergency right after `start_motors` reopened the brake at every cycle), and it loaded a CPU.
- **The platform and Z-recovery launches shut down when `ros2_control_node` exits** (`run_platform_control`, `run_z_recovery_control`). plc_manager watches only the launch process: before, a dead `ros2_control_node` left the launch, and the FSM, running.
- **`force_shutdown_stop()` closes the brake before opening the safety chain** (`plc_manager.py`).

### Tests

- `tecnobody_workbench_utils/test/test_eth_checker_brake.py`: 3 brake messages instead of the flood; close on drives off and on fault, only once; nothing when the node did not release the brake; close when the drive states stop; `stop_motors` always closes; the real `main()` closes the released brake on SIGINT and SIGTERM while the drive states keep arriving.

## 2026-10-07 — Z recovery always re-enables the end-stroke sensor

`PLC_node/estop_bypass` (GETC100 output 0x2001:4) bypasses the Z end-stroke safety sensor in the safety PLC, so the machine can be moved back into the workspace. `auto_z_recovery_node` set it to 1 and back to 0 only when the jog reached its target distance: after a jog timeout, a safety reclosure, or plc_manager stopping the recovery environment, the end-stroke sensor stayed bypassed until the PLC `ros2_control_node` restarted.

### Before deploying

- Rebuild `tecnobody_workbench_utils` on the robot.
- Test a Z recovery on the robot and check in the log `PLC safety restored: estop_bypass=0` at the end, and `estop_bypass: 0` in the PLC outputs shown by the GUI.
- Independently of this fix, check in the Flexi Soft program whether the bypass is limited (maximum time, only with `z_recovery=0` or the key, reduced speed): a process killed with SIGKILL or a frozen PC cannot restore it from ROS.

### Fixed

- **`auto_z_recovery_node` restores `estop_bypass=0` and `z_recovery=1` on every exit** (`_restore_plc_safety()`, was `_restore_z_recovery()`): at the end of the stop of the jog (target reached, jog timeout, safety reclosure), at shutdown, and in `main()`. The restore is sent 3 times, 20 ms apart.
- **SIGINT/SIGTERM are handled by the node**, not by rclpy: rclpy's handler shut the context down first, so on `kill_recovery_env()` nothing was restored, not even `z_recovery=1`. A second signal no longer interrupts the restore.
- **Commands for several interfaces go in one message**: `PLC_controller` applies only the last message of each 2 ms cycle, so `z_recovery=0` and `estop_bypass=1`, sent back to back, could lose the first one.
- Docstring: the PLC stays on the `run_sickPLC` `ros2_control_node` during the recovery; the recovery environment does not start a new one.

### Tests

- `tecnobody_workbench_utils/test/test_auto_z_recovery_safety.py`: one message for `z_recovery`/`estop_bypass`; restore repeated and sent once; restore after target reached, jog timeout and safety reclosure; restore received by a subscriber when the real `main()` gets SIGINT or SIGTERM.

## 2026-10-07 — Real-time diagnostics and unbuffered EtherCAT driver output

### Added

- **`bash_scripts/rt_latency_test.sh`**: `cyclictest` on every CPU while the system runs, with a per-CPU summary of late wake-ups (>250/500/1000/2000/4000 us).
- **`bash_scripts/rt_cpu_hogs.sh`**: in one window, the real-time setup (PREEMPT_RT, governor, irqbalance, rtprio limits, scheduling of the `ros2_control_node` and EtherCAT NIC IRQ threads), per-thread CPU and involuntary context switches, per-CPU load, EtherCAT lost frames, kernel warnings, `controller_manager` overrun messages and, with `rtla timerlat`, the kernel analysis of what blocked the CPU. See `bash_scripts/README.md`.

### Changed

- **Kernel warnings in the bring-up log archive**: new `journal/kernel_warnings.log` (`journalctl -k -p warning` for the session), also merged as `[KERNEL]` lines in `journal/fit4med_ethercat_timeline.log`. Before, only kernel lines containing "ethercat" were kept, so NIC link changes, thermal throttling, hung tasks or OOM were missing.
- **`ros2_control_node` runs under `stdbuf -oL -eL`** in `run_sickPLC`, `run_platform_control` and `run_z_recovery_control`. The EtherCAT driver reports working-counter and slave-state changes with `printf`; with stdout on a pipe (launch, `tee`, journal) they were written in 4 KB blocks, late, and lost if the node died.

### Before deploying

- Install on the robot `rt-tests`, `sysstat` and `rtla` (offline: copy the `.deb`).
- Tested on a development PC with mocked `cyclictest`/`pidstat`/`mpstat`/`rtla`/`ethercat`; not yet on the robot.

## 2026-10-06 — Lower `plc_manager` load on the pinned CPU

`plc_manager` was still doing too much work from the 500 Hz PLC-state path. The most visible effect was a very busy CPU 7, where the process is pinned by default.

### Before deploying

- Rebuild and reinstall `plc_manager` on the robot.
- If CPU 7 is still too loaded, run with `PLC_MANAGER_CPU_AFFINITY=6,7` to spread the executor over two cores, or `PLC_MANAGER_CPU_AFFINITY=off` to leave scheduling to Linux.

### Changed

- **PLC commands are published when values change, and refreshed every 0.5 s** (`plc_manager/plc_commands.py`). The IDLE/ESTOP safety refresh still calls `set_automatic_mode()`, `clear_sw_estop()` and `close_brake()` from the state callback, but an identical command vector is published again only after `plc_command_republish_period_sec` (default `0.5`) instead of at PLC-state rate. The refresh is needed: the `PLC_controller` subscription is best effort and `on_activate()` drops the last command, so the bring-up commands (`estop=1`, `z_recovery=1`, `force_sensors_pwr=1`) published between configure and activate of the controller would otherwise be lost for good, leaving the safety chain open. Refreshes are not logged.
- **`PlcCommandPublisher` is thread-safe**: the main loop (bring-up, shutdown) and the executor threads update the command vector under a lock.
- **GUI status payload generation is rate-limited** (`plc_manager/plc_manager.py`). The GUI still receives immediate updates when an FSM event is triggered; unchanged periodic status is now built and queued at 10 Hz by default (`gui_status_publish_period_sec`, default `0.1`) instead of on every PLC-state callback.
- **CPU affinity is configurable** with `PLC_MANAGER_CPU_AFFINITY`. The default remains CPU 7; comma lists and ranges are accepted (for example `6,7` or `5-7`), and `off`/`all`/`none` disable explicit affinity. It is now set before `rclpy.init()`, so the DDS and UDP threads stay on those CPUs too (before, only the main and executor threads did). A CPU that does not exist no longer stops the node: a warning is logged and the process runs on all CPUs.

### Fixed

- The PLC command interface list now uses `PLC_node/estop_bypass`, matching the EtherCAT/ROS 2 control configuration, instead of the stale `PLC_node/s_output.4`.
- The fallback GUI IP is now `127.0.0.1` instead of `127.0.0.0`.

### Tests

- Added `plc_manager/test/test_plc_commands.py` for command-interface consistency and duplicate-command suppression.
- Verified the new tests and the existing UDP client tests locally.

## 2026-10-06 — One log archive per bring-up, written by the launch itself

The logs were collected only by `fmrr_retrieve_logs.ps1`, run by hand on the GUI PC, often late (journal of another boot) or not at all. Now `run_sickPLC.launch.py` archives each run when it exits, however it was started (systemd unit, `fmrr_bringup.ps1`, by hand) and stopped (cleanup script, `systemctl stop`, Ctrl-C, `plc_manager` exit).

### Before deploying

- **Reinstall the systemd unit**: `TimeoutStopSec` goes from 15 to 60 s, so systemd does not kill the archiving. Copy `systemctl_services/fit4med-bringup@.service` to `~/.config/systemd/user/` (if it is a copy, not a link), then `systemctl --user daemon-reload`.
- **`fit4med` must be in the `systemd-journal` group** (or `adm`), or the EtherCAT lines are missing from the archive. Check with `id fit4med`; the procedure, including restarting the `systemd --user` of `fit4med` so the service sees the new group, is at the end of `bash_scripts/README.md`.
- Tested on a development PC with stand-in nodes (e-stop restart, Ctrl-C, launch killed with SIGKILL) and, for the retrieve, with `ssh`/`scp` replaced by local commands; not yet on the robot with the systemd unit and EtherCAT, nor from the GUI PC.

### Added

- **Run archive** (`bash_scripts/fit4med_session_log.sh`): `~/.ros/fit4med_log/run_NNNN_YYYYMMDD-HHMMSS.zip`. `NNNN` grows by one at each bring-up, so the order is right even when the robot clock is not. The archive contains:
  - `session.txt`: start/end time, systemd unit, GUI IP, why the launch stopped;
  - `sickPLC/`: ROS logs of `run_sickPLC.launch.py` and its nodes, with its `launch.log`;
  - `starts/NNN_<label>_HHMMSS/`: see below;
  - `journal/`: `journalctl --user -u fit4med-bringup@<ip>.service`, `ethercat.service` status and journal, EtherCAT kernel messages, and the merged timeline of both, limited to this run (not the whole boot);
  - `ethercat_start.txt`, `ethercat_end.txt`: `ethercat master` and `ethercat slaves -v` at start and end;
  - `ros_home_log/`: what was left in `~/.ros/log`, which is then emptied (what `log.sh` did, now also when the launch is started by hand).
- **One folder per start of the motion stack**: each start of `launch_ros2_env.sh` (`platform`), `launch_ros2_env_z_recovery.sh` (`z_recovery`) and `launch_ros2_bridge.sh` (`rosbridge`), e.g. the restart after each e-stop, gets `starts/NNN_<label>_HHMMSS/` in the run, numbered in order, with `console.log` (whole script output, EtherCAT checks and CoE fault reset included), `run.txt` (start, end, exit code) and the ROS logs of that start. When `plc_manager` stops a start, or the whole group gets SIGINT, the script waits for its `ros2 launch` to finish before closing the folder, and the archiving waits (up to 10 s) for the starts still shutting down.
- **Runs left open are recovered**: a run whose launch was killed without closing (SIGKILL, crash, power loss) is archived at the next bring-up, with the journal up to its last log write. A lock prevents archiving the same run twice.
- `FIT4MED_SESSION_LOG=0` disables the archive; `FIT4MED_LOG_ROOT` and `FIT4MED_SESSION_LOG_SCRIPT` change where it goes and which script is used.

### Changed

- `run_sickPLC.launch.py`: opens the run before starting any process (`FIT4MED_SESSION_DIR` and `ROS_LOG_DIR` are inherited by all nodes and, through `plc_manager`, by the `launch_ros2_*.sh` scripts) and archives it once every process has exited. During the archiving SIGINT/SIGTERM are ignored, so a second Ctrl-C or a cleanup script does not cut it short. The shutdown reason is recorded in `session.txt`.
- `launch_ros2_env.sh`, `launch_ros2_env_z_recovery.sh`, `launch_ros2_bridge.sh`: two lines each to open their start folder; outside a run (launched by hand) they behave as before.
- `systemctl_services/fit4med-bringup@.service`: `TimeoutStopSec=60` (was 15).
- **`fmrr_retrieve_logs.ps1` copies the run archives, not the whole boot journal.**
  - Copies the last 3 archives (`-Last N`, `-All`), chosen by run number, not by date; those already on the PC are not copied again. It lists what it copied, with start time and how each run ended.
  - A run still going on is zipped on the robot (`fit4med_session_log.sh snapshot`: logs and journals up to now, the run is left untouched) and copied as `run_NNNN_..._partial.zip`; the temporary copy on the robot is removed, and the partial zip on the PC is deleted once the full archive of that run is there. Runs left open by a killed launch are archived first, so they are copied too.
  - No longer writes `fit4med_combined.log`, `ethercat_service.log` and `fit4med_ethercat_timeline.log` for the current boot: they duplicated the `journal/` folder of the archives, for the wrong span (whole boot, possibly not the boot of the problem). `clocks.txt` and the GUI logs are still copied. `-GuiIp` is accepted and ignored.
- `fit4med_session_log.sh` also has `snapshot` and `list [N]`, used by the retrieve.
- `fit4med_merged_journal` (`fit4med_backup_common.sh`) takes an optional journalctl time range and an empty unit; without a range, the current boot as before.

### Documentation

- `README.md`, step 9 "Collect the logs after a problem": the run archive, its content and the new retrieve; one-time setup note on the `systemd-journal` group updated.
- `bash_scripts/README.md`, new last section: how to check and grant the `systemd-journal` group and how to tell from an archive whether it was active.
- `bash_scripts/README.md`, new section "Svuotare le cartelle dei log del robot": where the logs are on the robot and how to empty them all or keep the last N archives, keeping the run counter; optional journal vacuum. Linked from `README.md`, step 9.

### Not changed

- `fit4med_bringup_service.sh` still calls `log.sh` before the launch; no longer needed, harmless.
- No automatic deletion: the archives accumulate in `~/.ros/fit4med_log/` (the retrieve copies only the last ones). The `log_*.zip` written by `log.sh` before this change are not copied by the retrieve.
- In legacy mode, `kill_fmrr_apps.sh` kills whatever matches `fit4med_ws` after 10 s: an archiving still running then is interrupted, and the run is recovered at the next bring-up.

## 2026-09-25 — Movement results from the robot to the GUI, suspension causes, single-threaded trajectory manager

How the robot tells the GUI how a movement ended was rewritten on both sides. The GUI now learns why an exercise was suspended (tracking error, controller error, no progress) and tells the operator. No movement request can be lost any more without the GUI noticing. Resolves the "stall detection" open point of the audit below.

### Before deploying

- **Update the robot and the GUI together.** The GUI services `/rehab_gui/trajectory_finished`, `/rehab_gui/exercise_finished` and `/rehab_gui/exercise_suspended` change type from `std_srvs/Trigger` to `tecnobody_msgs/TrajectoryResult`. With mismatched versions these notifications do not get through: the robot logs `notification LOST`, the GUI only notices through the "NO PROGRESS" suspension.
- **Rebuild on the robot** and restart the whole stack, rosbridge included (it must know the new type):
  ```bash
  cd ~/fit4med_ws && colcon build --packages-select tecnobody_msgs tecnobody_workbench_utils
  ```
- **Checks on the machine:**
  - `ros2 interface show tecnobody_msgs/srv/TrajectoryResult` shows the type;
  - with the GUI connected, `ros2 service type /rehab_gui/exercise_suspended` gives `tecnobody_msgs/srv/TrajectoryResult`;
  - to see the pop-up without a real error, during a training:
    ```bash
    ros2 service call /rehab_gui/exercise_suspended tecnobody_msgs/srv/TrajectoryResult \
      "{success: false, action_status: 6, error_code: -4, message: 'test', movement_kind: 'exercise'}"
    ```
    It must answer `accepted: true`, and the GUI must show "Training suspended - tracking error".
  - troubleshooting: `notification LOST` / `did NOT accept` / `Movement refused` in the robot journal (`fit4med_ethercat_timeline.log`), `on_exercise_suspended` in `gui_console.log`.
- **The trajectory manager (`fct_manager_node`) now runs single-threaded with `spin()`** instead of `spin_once()` + `sleep(0.05)`. Measured on one core with the node's timer and subscription rates: CPU from ~1 % to ~2.5 %, all callbacks served. Try a full training, a PAUSE/RESUME and a STOP on the real platform.

### Robot side (`tecnobody_workbench_utils/gui_trajectory_manager.py`)

#### Changed

- **Results sent as `tecnobody_msgs/TrajectoryResult`** (`success`, `action_status`, `error_code`, `message`, `movement_kind`), answered by the GUI with `accepted`:
  - repetition aborted → `exercise_suspended` with the controller's code and message (e.g. −4 `PATH_TOLERANCE_VIOLATED`);
  - repetition without a usable result → `exercise_suspended`, code 999;
  - repetition completed → `exercise_finished`;
  - PTP / go-to-start ended → `trajectory_finished` with its outcome.

  This reinstates the reporting added in July (`4b243d2`, `c0637c6`, `21ab013`) and removed in `564cfd7` because the message was not built on the machine.
- **`GuiNotifier` delivers every notification to the GUI**, including `movement_stopped` (still `Trigger`):
  - it never blocks the calling callback (before: up to 5 s inside the executor);
  - if the GUI service is missing (rosbridge reconnecting), it waits up to 10 s instead of dropping the notification (before: dropped after 0.5 s for "repetition finished");
  - each notification is sent once and in order, since resending one whose reply got lost could count a repetition twice;
  - the GUI's answer is checked: refusals, missing replies and notifications given up are logged;
  - its polling timer runs only while something is pending, costing nothing at rest;
  - one client per service instead of a new one for every notification.
- **Movement requests are refused, not lost, when the controller is missing.** `set_trajectory`, `set_go_to_start_trajectory`, `set_rehab_exercise` and `set_eeg_exercise` used to answer `success: true` in every case. They now wait up to 1 s for the controller's action server (discovery right after a controller switch) and answer `success: false` if it does not appear; the GUI then switches the motors off and reports it.
- **Goals that never start are reported:**
  - rejected by the controller → `INVALID_GOAL` (−1);
  - not answered within 5 s → code 999, and a late acceptance is cancelled at once instead of running;
  - an acceptance of a superseded goal (e.g. after a timeout and a retry) is cancelled as well, instead of replacing the current goal.

  Exercises report to `exercise_suspended`, single trajectories to `trajectory_finished`.
- An aborted repetition stops the progress timer (it kept sending time-based progress for a dead goal).
- A trajectory cancelled by a GUI stop no longer sends `trajectory_finished` (the GUI gets `movement_stopped`). It left the GUI's "completed" flag set, ending the next movement as soon as the motors were on.
- **Single-threaded executor and `spin()`** in `main()`. The process is pinned to one core and Python runs one thread at a time, so the four threads gave no parallelism, only interleaved callbacks (e.g. the progress timer reading the lists `clear()` rebuilds). `spin_once()` + `sleep(0.05)` ran at most ~20 callbacks/s: the 25 Hz speed scaling subscription got ~10 Hz. No callback blocks, so running them one at a time cannot deadlock. Ctrl-C/`ExternalShutdownException` end the node cleanly.

### GUI side (`rehab_gui`)

#### Changed

- **Two kinds of suspension** (`TrainingProtocolWindow.py`):
  - **by the robot** (aborted repetition, or a stop not requested by the GUI): the robot is already stopped, the motors are switched off. State label "TRACKING ERROR" (codes −4/−5), "ROBOT ERROR" or "ROBOT STOP";
  - **by the GUI** ("NO PROGRESS"): no progress from the robot for 15 s (was 5 s), now measured in real time instead of timer ticks, also at 0 %. The movement is stopped (`requestStopAnyMovement`, then motors off) because the robot may still be moving.

  Both keep the phase for the resume.
- **Non-blocking pop-up** on suspension. It gives the phase, the explanation of the code, the robot controller's message and how to resume, with the full result in the details. It is not `exec_()`, so the GUI keeps running.
- **The state label** shows the cause under "SUSPENSION STATE" (`MotorsWindow.py`).
- The three services are `tecnobody_msgs/TrajectoryResult` and answer `accepted` (`sync_ros_events.py`); a malformed request still counts, with code 999.
- A failed PTP or go-to-start is reported in the status bar with its code; the motors were already switched off.

#### Fixed

- The "completed" flag and the last result are cleared before each PTP / go-to-start, and the progress is reset to 0 % at each exercise start (it showed the previous exercise's 100 %).
- The suspension cause is cleared together with its flag.

### Tests

- Robot: `test/test_gui_notifier.py` and `test/test_goal_start.py`, 18 tests with real rclpy nodes:
  - the notifier: late GUI, order, refusal, GUI never coming, idle timer, `Trigger` services;
  - the goal results: aborted, without result, succeeded, cancelled;
  - the real manager against a fake `FollowJointTrajectory` controller: absent, late, rejecting, accepting, not answering in time, a superseded late acceptance, a GUI stop;
  - the real `main()` in its own process: answers a request, stops on Ctrl-C with exit code 0.
- GUI: `tests/test_suspension_kinds.py`: result services, the two suspension kinds with a controlled clock, pop-up texts, flag and progress resets. `tests/test_training_resume.py` gives the suspension cause. 71 GUI tests pass.

### Notes

- `safemod_controllers.yaml` (used by the platform launch files) sets `trajectory: 0.15` per joint for both trajectory controllers: a deviation above 0.15 m aborts with `PATH_TOLERANCE_VIOLATED` (−4), shown as "TRACKING ERROR". With `goal_time: 0.0` the controller waits indefinitely for the goal tolerance (1 mm, 5–10 mm for go-to-start): code −5 cannot occur, and a movement that never gets within tolerance stays at 100 %, which the "NO PROGRESS" suspension now catches after 15 s.
- Not changed: `set_trajectory` still answers before knowing whether the controller accepts the goal (answering later would mean waiting inside the service callback); rejections and timeouts are reported through `trajectory_finished` / `exercise_suspended` instead.

## 2026-09-25 — `rehab_gui` audit: crashes, motion safety, training resume

Second audit of the GUI. Every fix comes with regression tests that fail on the previous code; the GUI test suite grows from 24 to 56 tests.

### Before deploying

- **GoTo (Robot tab) durations change.** Some movements become slower, others faster, all within the same limit (see below). Try a long and a short GoTo on the real platform.
- **CREATE without saving no longer changes the active movement.** To try a newly created movement, save it first.

### Fixed

- **`gui_errors.log` stayed empty for GUI-thread errors** (`TrainingProtocolWindow.py`). The module called `rich.traceback.install()` at import time, after `session_log` had set up its handler, and replaced it. The call is removed: `session_log` installs rich once, for the terminal and the file.
- **CREATE (Rehabilitation Movement tab) could close the whole GUI** (`RehabilitationMovementWindow.py`). An exception escaping the Qt slot makes PyQt5 abort the process:
  - with no exercise type selected (possible after loading a movement file with `type: 0`), an unassigned variable raised `UnboundLocalError`; CREATE now asks to select the type;
  - a Hand-to-Mouth source movement with no displacement along one axis caused a division by zero; it is now refused with a message;
  - any other numerical failure (e.g. cubic interpolation of repeated points) is reported as "Movement not created" and logged.
- **CREATE changed the active movement before it was saved.** Type and side were overwritten at the start, even when the creation then failed; the trajectory, `Vmax` and `PhaseDuration` were replaced before saving. With the save cancelled, training ran the new, unsaved trajectory under the previous file's name, and the PLC could receive the wrong sensor mode. The created movement is now kept aside and becomes active only once saved, right away or later with SAVE. LOAD discards an unsaved created movement. CREATE is refused during a training or a ROS command, like LOAD.
- **GoTo (Robot tab) could be much faster than intended** (`RobotWindow.py`). The duration was computed from the target's distance from zero, not from the current position, and per axis instead of along the path, with a discontinuity at 10 cm. Example: from −0.40 m to (0.05, 0, 0.10), 0.45 m in 1.0 s, i.e. 0.45 m/s average (~0.68 m/s peak) instead of 0.1 m/s. The duration is now the straight-path distance from the current position divided by `PTP_MAX_SPEED` (0.1 m/s average), at least `PTP_MIN_TIME_S` (2 s). The current position is read after switching the motors on, as the ROS side starts the trajectory from there.
  - An invalid current position (wrong length, non-numeric, NaN or infinite distance) switches the motors off and is reported. A NaN distance is turned into an error on purpose: `max(2.0, nan)` would silently give 2 s.
  - After a failure (target out of range, motors not switching on, invalid position), the GoTo button is released: the next press used to send a stop instead of a movement.
- **After a suspension, later trainings restarted from the wrong phase** (`TrainingProtocolWindow.py`). The phase to resume from after a suspension (tracking error) was reset only by reloading the protocol, so a completed or stopped protocol ran again from the suspension phase. Stopping, completing the protocol or losing the ROS connection now reset it to phase 1, and the total time shown is updated. A suspension still resumes from the interrupted phase, so the protocol is concluded.
- **Motors left on when a send failed after switching them on.** The operator saw nothing but "Warning Motors On":
  - training (`sendExercise`): exercise not accepted → motors switched off, message; motors not switching on → message (previously silent);
  - GO to START (`_goToStartPosition_afterDelay`): the send result was never checked; on failure the motors are switched off, the trajectory controller is restored (as after a completed go-to-start), message;
  - GoTo (Robot tab): motors switched off, button released, message.

  A STOP pressed during the send cancels the task: the added lines do not run, and the stop sequence switches the motors off as before.

### Tests

- `tests/test_create_movement.py`: CREATE without type, degenerate Hand-to-Mouth sources, created movement kept until saved, LOAD discarding it, refusal during training.
- `tests/test_goto_ptp.py`: GoTo durations, invalid positions, button release, failed send.
- `tests/test_training_resume.py`: phases sent by START after suspensions, stops, completions and ROS loss.
- `tests/test_send_failures.py`: failed sends in training and go-to-start, including STOP during the send through the real `GuiTask`.
- `tests/test_async_flow.py`: the stop harness also stubs `_update_total_training_time_display`.

### Open points (not changed)

- **Hand-to-Mouth template scaling** (CREATE): per-axis scaling cannot reach a target when the template does not move along an axis (now refused), amplifies noise on axes that move very little, and uses absolute scale factors, so a target of opposite sign ends at the mirrored point; the end point is then overwritten with the reached one, without warning. On hold for review by the author of the algorithm.
- **Stall detection** (`TrainingProtocolWindow.updateWindow`): 5 s without progress are treated as a suspension and the motors are switched off, but the ROS-side progress is time based and also stops at 100 % while the last segment settles, and whenever the speed factor drops below 0.01 for reasons other than the GUI PAUSE.
- **Drive logic power reset** (`MotorsWindow.resetFaults`): power is cut and restored by a 2 s GUI timer; if the ROS link drops meanwhile, it stays cut, and the button is re-enabled within ~100 ms, allowing overlapping cycles.
- **`PhaseIsEnabled`** is read from the protocol but never used: all 20 phases are always executed.
- **`'override': 50`** is sent with every PTP request; how the ROS side applies it (and so the real PTP speeds) is not verified.
- Minor: the relative homing writes marker files to `/tmp` (on Windows `C:\tmp`) that nothing reads; `clbk_BtnGoToStartPosition` sets `Training_ON` on the movement window, where it is never used.

## 2026-09-25 — GUI/controller link robustness, logs, robot backup and sync

The main goal of this round was the GUI (`rehab_gui/FMRRMainProgram.py`) that sometimes crashed or froze and left UDP port 5005 taken, so that the next GUI could not receive the controller status.

### Before deploying

- **Update both PCs.** The Linux PC needs the new `plc_manager` (rebuild with `colcon build`), the Windows PC the new GUI and `ps_scripts`. Either side keeps working with the old version of the other, so the order does not matter.
- **Reading the `ethercat.service` journal** needs, once, on the Linux PC: `sudo usermod -aG systemd-journal fit4med` (effective at the next ssh login).
- **Windows checkouts made before this change** still have shell scripts with Windows line endings. Fix them once, see [bash_scripts/README.md](bash_scripts/README.md#da-windows).
- **To test on the real platform:** a normal START/STOP; a bring-up restart with the GUI left open; a GUI restart while the PLC is `IDLE`; a ROS connection failure (e.g. stop rosbridge while `RUNNING`); one run of `fmrr_to_robot_sync.ps1 -DryRun` and `fmrr_backup.ps1` from Windows.

### GUI ↔ controller UDP link

#### Fixed

- **UDP receive loop leaving port 5005 bound but unread** (`rehab_gui/UdpCommunicationManager.py`). On Windows, an ICMP "port unreachable" caused by one of the GUI's replies is reported as `ConnectionResetError` on the next `recvfrom()`. The loop treated it as fatal and exited without closing the socket: the GUI kept running, the watchdog showed "PLC communication lost", and a new GUI could not use the port. Now:
  - the Windows behaviour is switched off (`SIO_UDP_CONNRESET` through `WSAIoctl`);
  - the loop ignores connection resets and other errors, and ends only when `stop()` is called;
  - the socket is always closed when the loop ends.
- **GUI aborted by a failed UDP send.** `send_response()` runs inside Qt slots, where an uncaught exception makes PyQt5 abort the process. `sendto()` errors are now caught and logged, and the socket is read once to avoid a race with `stop()`.
- **Replies lost after a bring-up restart** (`plc_manager/plc_manager.py`). `plc_manager` sent from a random port; after a restart the GUI replied to the old, dead port (which also triggered the Windows error above). It now sends from the fixed port **UDP 5006** (`PLC_MANAGER_UDP_PORT`). A second `plc_manager` still alive now fails visibly at startup.
- **START blocked after a GUI crash.** `plc_manager` allows START only if the last GUI message is `ROS_DISCONNECTED` (or none), but the GUI sent its state once, on change. A GUI that crashed while connected left `ROS_CONNECTED` as the last message, and any lost reply was never corrected.
- **Controller receive loop could stop silently** (`plc_manager/udp_client.py`): same `except OSError: break` pattern as the GUI. It now keeps receiving until `close()`.

#### Changed

- **The GUI repeats its ROS state** (`ROS_CONNECTED` / `ROS_DISCONNECTED` / `ROS_CONNECTION_FAILED`) in reply to every status packet, so a lost or misdirected reply is corrected within ~0.5 s. A freshly started GUI reports `ROS_DISCONNECTED` on the first packet.
- **`ROS_CONNECTION_FAILED` is kept until the PLC has left `RUNNING`**, so the GUI's own ROS teardown can no longer overwrite it with `ROS_DISCONNECTED` before `plc_manager` has acted on it (previously a small race). It then switches back to `ROS_DISCONNECTED`, which the next START requires.
- **No ROS reconnection attempts after a failure** while the PLC is still `RUNNING`: the GUI waits for `plc_manager` to handle the failure (FAIL). If the FAIL transition itself failed, `plc_manager` keeps receiving `ROS_CONNECTION_FAILED` and retries it.
- **Signal-rate limit in the GUI.** The UDP thread still reads every packet, but hands one to the GUI thread immediately only when the FSM state or pending transition changes. Repeats of an unchanged state are delivered at most every 100 ms (`UdpServer.MIN_EMIT_PERIOD_S`), always the newest. A busy GUI thread can no longer accumulate a queue.
- `plc_manager` logs received GUI messages only when they change (they now arrive about twice a second).
- `ethercat.last_error` is truncated to 300 characters in the status packet, so it always fits the GUI's 4096-byte receive buffer.

#### Removed

- `free_udp_port()` and its use of `psutil` in the GUI: it ran inside the UDP thread and force-killed any process on port 5005 (hard-coded, not the configured port). Leftover processes are now cleaned up by the launcher (see below). `psutil` is still listed in `pyproject.toml` so the lock file needs no update; it can be dropped at the next re-lock.

### GUI logs and diagnostics

#### Added

- **Per-session log files in `C:\temp`** (`rehab_gui/session_log.py`), each rotated at start-up to `<name>_YYYYMMDD_HHMMSS.log` (timestamp = last write of the previous session, i.e. about when it crashed or was closed); only the 20 newest backups are kept:
  - `pyqt_hang.log`: faulthandler, stacks of all threads every 10 s and on a fatal error (as before, now rotated);
  - `gui_errors.log`: every uncaught exception, GUI thread and Python threads, rendered by `rich` with local variables, no colours. PyQt5 calls the exception hook before aborting, so the traceback is written before the process dies;
  - `gui_console.log`: everything the GUI prints, timestamped, colour codes removed.

  The terminal keeps showing `rich`'s coloured tracebacks.

#### Fixed

- A missing `C:\temp` folder crashed the GUI at start-up; it is now created.

### ROS status poller

#### Changed

- The drive-state and controller-list queries (`rehab_gui/sync_ros_events.py`) are sent together with one shared timeout of 1.0 s (`_poll_timeout_s`), instead of one after the other with 3 s each. A poll now takes at most 1 s instead of up to 6 s. Not lower: a missed drive-state answer shows the drives as "n/a / MOTORS FAULT" until the next poll.

### Windows scripts (`ps_scripts`)

#### Changed

- **`fmrr_cleanup.ps1`** also cleans up the local GUI:
  - kills leftover GUI processes, a Windows Error Reporting instance keeping a crashed GUI alive, stale `fmrr_gui.ps1` windows, and anything else bound to UDP 5005;
  - reports whether the port is free;
  - never kills itself or the windows that started it;
  - new `-Target all|gui|remote` (default `all`) and `-UdpPort`.
- **`fmrr_gui.ps1`** runs `fmrr_cleanup.ps1 -Target gui` before starting the GUI, and refuses to start with a clear message if port 5005 is still taken. The remote side is not touched.
- **`fmrr_retrieve_logs.ps1`** also collects:
  - the GUI logs (current session and backups) into `Desktop\fit4med_logs\gui\`;
  - `ethercat_service.log`: `systemctl status`, the `ethercat.service` journal, the EtherCAT lines of the kernel log, `ethercat master` and `ethercat slaves -v`.
  - `fit4med_ethercat_timeline.log`: one timeline of the bring-up service, `ethercat.service` and the EtherCAT kernel messages, sorted by time and tagged `[FIT4MED]`, `[ETHERCAT]`, `[KERNEL]`, to see whether the EtherCAT network or the controller reports a problem first (multi-line messages stay together);
  - `clocks.txt`: robot and PC clocks at retrieval time, to line up the robot logs (robot clock) with the GUI logs (PC clock).

### Robot software sync, backup and restore

#### Fixed

- **`fit4med_to_robot_sync.sh` did not overwrite the robot's changes.**
  - The default host was `10.2.15.217` instead of `192.168.1.1`.
  - `rsync --update` skipped every file whose robot copy had a newer timestamp, so edits made on the robot won (and the offline robot's clock made this unpredictable).
  - Files existing only on the robot were never removed.

#### Changed

- **`fit4med_to_robot_sync.sh` makes each local folder identical on the robot**: no `--update`, `--delete`, and contents compared with `--checksum`.
  - **Only the top-level folders present in the local `src` are synced.** Robot folders the user does not have (e.g. `ethercat_controller` when only `Fit4Med` is checked out) are never touched; `--delete-extra-folders` removes them too, for deleting a whole package.
  - Before copying, it offers a backup of the robot sources (default yes); if the backup fails, it aborts.
  - It lists the files that exist only on the robot and asks before deleting them (default no).
  - `.git` and Python caches on the robot are never sent nor deleted.
  - New options: `--backup`, `--no-backup`, `--yes`, `--local-path`, `--delete-extra-folders`.
  - `fit4med_from_robot_sync.sh` keeps `--update`, so pulling does not overwrite newer local files.
- **The local sources are the workspace `src` containing the repository**, wherever it is, instead of a fixed `~/fit4med_ws/src/`. Override with `--local-path`. The folder must be named `src`: syncing a plain clone's parent folder would copy unrelated folders to the robot.
- All ssh/rsync calls of a run share one connection: without an ssh key the password is asked once.

#### Added

- **`bash_scripts/fit4med_backup.sh`**: copies the robot sources to `/home/fit4med/bkp/YYYYMMDD/HHMM/` (`HHMM_2`, … for several in the same minute; an incomplete copy is removed).
- **`bash_scripts/fit4med_restore.sh`**: lists the backups (newest first, with size), restores the chosen one after confirmation, and offers a safety backup of the current state first.
- Both work on the robot or from the user PC through ssh (the robot needs neither the scripts nor internet); from the PC the backup is named after the PC's clock. `--local` / `--host <ip>` force the mode.
- **Windows versions:** `ps_scripts/fmrr_to_robot_sync.ps1`, `fmrr_backup.ps1`, `fmrr_restore.ps1`. They use the same bash functions on the robot, so they behave the same, and need only the `ssh`, `scp` and `tar` included in Windows 10/11. Windows has no rsync: the sources are packed, uploaded and mirrored on the robot. `-DeleteExtraFolders` matches `--delete-extra-folders`.
  - The executable bit is restored for scripts starting with `#!`.
  - The sync refuses shell scripts with Windows line endings.
- **`.gitattributes`**: keeps `*.sh` with Linux line endings in Windows checkouts. With Git's default `core.autocrlf`, shell scripts copied from Windows would not run on the robot.
- **`bash_scripts/README.md`** (Italian): how to update, back up and restore the robot software, from Linux and from Windows.

### Tests

- `rehab_gui/rehab_gui/tests/test_udp_status.py`: UDP link against a simulated `plc_manager` on local sockets (replies to every status, rate limit, failure kept until the PLC leaves `RUNNING`). All 24 GUI tests pass; the 3 existing `plc_manager` UDP client tests pass.
- The sync, backup and restore scripts (Linux and Windows) were tested against a simulated robot; the Windows scripts ran under PowerShell 7 on Linux, not yet on Windows.
