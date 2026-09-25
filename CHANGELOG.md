# Changelog

Notable changes to the Fit4Med platform software. Newest first.

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
