# RT / EtherCAT investigation — state and next steps (2026-11-12, end of day)

Handoff for the next session. Read together with `RT_DEBUG_NOTES.md` (Windows clone
`C:\Users\NicolaPedrocchi\src\fit4med\git\Fit4Med`, untracked; WSL path
`/mnt/c/Users/NicolaPedrocchi/src/fit4med/git/Fit4Med/RT_DEBUG_NOTES.md`). Where the two disagree,
this file is newer.

Goal unchanged: find out whether the intermittent EtherCAT drive dropouts come from the PC/real-time
side or from the drives. Today's work moved the non-RT load off the isolated CPUs and cut it at the
source. **No dropout has been captured yet.**

## 0. The EtherCAT drops in runs 0040–0048: joint_z drive (slave 1-2) disappears (analysis 2026-11-16)

Logs of runs 0040–0049 in `~/tmp/Logs` (the dates `20260605` of 0040/0041 are not reliable: dead RTC).
Script used: per master-1 session, kernel events between "12/12" and "Releasing master".

- **83 in-operation drops** in runs 0040–0048, always the same signature, with the platform in OP (12/12):
  `Working counter changed to 6/12` + `2 slave(s) responding` → only slaves 0 (joint_x) and 1 (joint_y) answer:
  **slave 2 = joint_z drive** (and the ATI sensor, slave 3, wired after it) vanishes.
- No warning before: no emergency, no AL error, no lost frames, no datagram timeout (one exception: 0043
  10:45:27, WC 0/12 + 4 datagrams timed out, 211 lost frames). The master keeps cycling: **not a PC/RT problem**.
- The slave comes back after **18.2 s, almost always exactly** (`4 slave(s) responding`), then emergency
  0x8120 (comm. error, data `01 85`) from 1-2; meanwhile joint_x/joint_y go to AL 0x0034 "DC Sync Timeout" and
  emergency 0x6200 (data `03 E1`) — consequences of the re-scan. Back to 12/12 after ~36 s in total.
  A constant 18.2 s fits **a reboot of the joint_z drive** (logic power interruption or internal reset), not an
  intermittent cable (that would come back at random times, usually much faster).
- Last CiA402 state printed before a drop: Operation Enabled 32, Fault 14, Switch on Disabled 9, Not Ready 6 →
  the first drop of an episode is usually with the drives enabled (often during an exercise), but drops also
  happen with the power stage off → not simply motor current. Drops cluster (tens of s to minutes apart).
- **Run 0049 (11-11 11:18–14:13) and all runs of 11-12: zero drops**, with more platform-OP time (2.9 h) and more
  motion (95 min of active trajectories) than run 0048 (2.1 h, 86 min, 15 drops, 13 during motion). Something
  changed on 11-11 between 11:05 and 11:18 — not the DC/memlock fixes (applied 11-12 afternoon; memlock still
  failing in 0049) and not the platform software (startup logs identical). **Ask the user** (hardware: Z drive
  cable/connector, power supply, cabinet, drive parameters, power cycle?).
- **Codes decoded with the Delta ASDA-A2-E manual** (drives are Delta ASDA-A2-E, vendor 0x1DD; manuals in
  `tecnobody_workbench_utils/resource/`, chapter 9 alarm list):
  - joint_z `0x8120` data `01 85` = **AL185 "EtherCAT connection error (Servo Off)"**: the drive itself detected
    that its EtherCAT link was lost. P3-18 (default 0x00002000, digit Z = 0) checks AL185 only after OP, so a
    freshly power-cycled drive would not raise it: **the drive control stayed on; its EtherCAT link/interface
    went down for 18.2 s**. This weakens the "logic power reboot" reading above; remaining candidates: the
    joint_y → joint_z cable/connectors (manual: CAT5e STP shielded, max 50 m, proper grounding), or the
    EtherCAT interface of the joint_z drive resetting (fixed 18.2 s) because of disturbances.
  - joint_x/joint_y `0x6200` data `03 E1` = **AL3E1 "CANopen SYNC failed"** (manual: reference clock jitter
    with DC sync): consequence of the re-scan of the bus after joint_z vanished.
  - `0x9000` data `05 00` = AL500 STO (estop, normal); `0x8130` = AL180 (PDOs not received); `0x8611` =
    following error.
  - Drive alarm history: P4-00…P4-04 (last 5 alarms, low word = ALnnn) = CoE objects `0x2400`…`0x2404`
    (`ethercat upload -m1 -p2 0x2400 0 --type uint32`); P0-02 = 121 shows PDO receive errors on the panel.
- **Monitor**: `bash_scripts/ec_link_monitor.sh` (uncommitted) logs, only on change, the slaves answering,
  lost frames, ESC counters 0x0300/0x0308/0x0310 of slaves 1-3 every 2 s, and the Delta alarm history
  (P4-00…04) of slaves 0-2 at start and when the slaves are back after a drop; `-H` prints the history only.
  Log: `~/.ros/fit4med_diagnostics/ec_link_monitor_*.log`. Cost ~1-2% of one housekeeping CPU.
  Also DL status 0x0110 / AL code 0x0134, alarm history at stop, and `ec_drives_*.log` every 10 s: current
  (0x6078), torque (0x6077) and the Delta status monitors P0-09..11 named after P0-17..19 (skipped while 0).
  `ec_delta_monitor_setup.sh` shows / `--apply` (14 = DC bus V, 16 = IGBT °C, 12 = avg load %, with a restore
  file) / `--restore` those selectors — **not applied yet** (user, in the next days).
  `rt_load_monitor.sh`: mpstat, pidstat processes/threads (%wait), RT threads placement, PC temperatures
  (`sensors`), wall-clock steps. Both monitors start from `launch_ros2_env.sh` with `FIT4MED_EC_MONITOR=1` /
  `FIT4MED_CPU_MONITOR=1` and log into the start folder (session zip). All uncommitted, not on the robot yet.
- If it comes back, right after a drop read: `ethercat -m1 -p1 reg_read 0x0310 4` (lost-link counters of joint_y,
  which stays powered: port 1 increments if the link to joint_z dropped) and `-p2 reg_read 0x0300 8` /
  `0x0310 4` / `0x0910 8` (joint_z: counters and DC local time restart from ~0 after a reboot), plus the joint_z
  drive's own fault/event history (vendor tool) and a logger on its 24 V logic supply.

## 1. Where the code is

| Place | State |
|---|---|
| WSL `~/fit4med_ws/src/Fit4Med` | branch **`rt-cpu-layout`** (local only, **not pushed**): `8dc7ba7` → `cfe7b62` → `e064176` → `8697842` → `fa33268` → `a08bcff`. Working tree clean (this file untracked). |
| Robot `fit4med-cnr` (no git) | sources = **`a08bcff`** (plc_states watchdog off, STOP/FAIL announced to the GUI, drive-states watchdog raises SW ESTOP) (synced with `bash_scripts/fit4med_to_robot_sync.sh --folder Fit4Med/<pkg>`, built with `colcon build --packages-select …`). Check with `md5sum` against `git show e064176:<path> \| md5sum`. |
| Windows clone | branch `plc-manager-wip` at `e901b4a` ("wip - fms fix ethercat down": FSM handling of EtherCAT down, `ethercat_monitor`, tests). **Not on the robot and not in `rt-cpu-layout`**: decide whether to rebase `rt-cpu-layout` on it or merge later. |

Commits of `rt-cpu-layout` (details in `CHANGELOG.md`, two 2026-11-12 entries):

- `cfe7b62` **Move non real-time nodes off the isolated CPUs** — new `tecnobody_workbench_utils/cpu_affinity.py`
  (`housekeeping_cpus()` = online CPUs minus `/sys/devices/system/cpu/isolated`); `eth_checker`, `filter_commands`,
  `gui_trajectory_manager` and `plc_manager` (default) use it instead of fixed CPUs 6/6/5/7.
  Known gap: the edit to `run_rosbridge_websocket.launch.py` is useless — the bring-up uses `run_rosbridge.launch.py`
  → `launch/rosbridge_websocket_launch.xml`, which has `launch-prefix="taskset -c $(var cpu_affinity)"`, default `4`
  (rosbridge_websocket and rosapi stay on CPU 4). The CHANGELOG entry claims otherwise: fix both in step 3.
- `e064176` **Publish PLC states on change and lighten the drive-state checks** —
  `plc_controller` publishes `plc_states` at once on any input change, else every `state_publish_period_ms`
  (default 20 ms), names written once (no string allocation in the RT loop); FSM transition timeouts in seconds
  (`timeout_sec`, `fsm_config.TRANSITION_TIMEOUT_SEC = 100`, was `max_steps=50000` = 100 s at 500 Hz);
  `plc_manager` watchdog: no `plc_states` for 0.2 s after bring-up → SW ESTOP, + FAIL when RUNNING
  (**misfired on an estop test, see §1b; disabled by default in the pending fix**);
  `ethercat_checker` `check_states` timer 250 → 50 Hz (brake closes 200–220 ms without drive states).
  Verified on the robot: ~50 Hz idle, input changes arrive 1.8 / 14.2 ms after the previous message (not on the
  heartbeat), estop → `RUNNING + STOP => ESTOP` completed in 1.6 s, no `No PLC states`.

## 1b. Incident 17:45:12 — estop test, watchdog took the wrong path (fixed, deployed 17:54)

The user pressed the estop (test) during the platform start `005_platform_174437` (fct_manager started 17:45:11).
- 17:45:12.280 `plc_manager`: "No PLC states for 0.33 s" → SW ESTOP + `RUNNING + FAIL => ERROR` (watchdog of `e064176`).
- 17:45:12.33 platform launch SIGINT (FAIL cleanup); drives `Fault 0x618` (power cut); master 1 released →
  slaves `SAFEOP + ERROR`, `Sync manager watchdog`, CoE emergencies (consequence of the release, as in RT_DEBUG_NOTES §7).
- The PLC `ros2_control_node` (pid 201254) **never stopped**: the 0.33 s gap was `plc_manager` not running
  `state_callback` (plc_group blocked), right when the estop change arrived. The estop STOP was never processed.
- Safety outcome OK (SW ESTOP raised, `brake_disable: 0`, platform stopped) but wrong FSM path (ERROR, key re-arm).
- The previous start (`004_platform_173736`) ended 17:44:23 with EXIT_CODE=1: check whether that was also an estop
  press (`journalctl --user -u fit4med-bringup@192.168.1.2.service --since 17:44:00 --until 17:44:30`).

Fix: `plc_states_timeout_sec` default 0 = watchdog disabled (`plc_manager.py`, test, CHANGELOG). **Deployed on the
robot at 17:54** (installed file has `0.0`, plc_manager restarted 17:58) and **verified**: estop test at 18:00:35 →
`RUNNING + STOP => ESTOP` completed in 1.2 s, no FAIL, no `No PLC states`.
Open question: **why `plc_manager` stops running `state_callback` for >0.3 s** (it also delays the estop STOP without
the watchdog). Suspects: a long callback in the MultiThreadedExecutor (2 threads) — START success checks
(`platform_controller_readiness_monitor.ok()` → mode switches / service calls), `EthercatMonitor.poll_slave_states`,
FSM actions run inside callbacks (`kill_env` etc. while holding `self.lock`). Instrument callback durations before
re-enabling a watchdog; a watchdog must measure message arrival, not callback execution.

## 1c. False drive-states watchdog trips (brake dragging) — mitigated, cause open

- 18:41:25 `ethercat_checker`: `Brake closed: no drive states for more than 0.2 s` while EtherCAT was fine (no kernel
  event, regular read period), right when `ros2 topic echo` started on the robot. Before the fix the drives stayed
  enabled and the FSM in RUNNING: motors pushed against the closed brake for 20 s (mechanical noise) until the mushroom.
- Fix (commit `a08bcff`, `eth_checker.py`): the watchdog closes the brake **and raises the SW ESTOP**
  (`brake_disable: 0, estop: 0` in one PLC message; the PLC latches it until the key) → usual `RUNNING + STOP => ESTOP`;
  plus "late timer run does not trip". Deployed and tested: 4 × `ros2 topic echo` with the platform in RUNNING,
  no motion → 3 nothing, the 4th tripped → clean stop (brake + ESTOP), no dragging. Journal 19:00:22.054
  `Brake closed and SW ESTOP raised: no drive states for 0.21 s` → 19 ms later `RUNNING + STOP => ESTOP` → GUI
  `ROS_DISCONNECTED` → completed in 1.5 s. The gap was just above the 0.2 s timeout.
- The late-timer guard did not catch it ⇒ the timer ran on time and the drive states really did not reach the node
  for > 0.2 s. Main suspect: the drive-states publisher (non-RT thread of the platform `ros2_control_node`, pinned
  with all its DDS threads to CPU 3, ~66% busy) delayed by the DDS discovery of the new participant. Step 3 (only
  the RT thread on CPU 3) should cure it. To discriminate: an independent probe subscriber logging
  `/state_controller/drive_states` inter-arrival gaps while running `ros2 topic echo` (gaps there too ⇒ publisher side).
- Rule until then: **no `ros2` CLI on the robot while the platform is RUNNING**.

## 1d. CPU 3 (platform RT CPU) overloaded by non-RT work — main open problem

| Run | CPU 3 busy | CPU 2 busy | note |
|---|---|---|---|
| Baseline 16:13 | 63% | 28% | |
| Step 1 | 58% | 23% | |
| Step 2 | 66% | 22% | |
| `e064176`, 17:35 (partial) | 62% | 17% | |

The platform RT thread itself uses only ~4% of CPU 3 (RT_DEBUG_NOTES §3.2: ~167 µs per 4 ms cycle); the PLC RT
thread ~5% of CPU 2. The rest is non-RT work pinned there because `run_platform_control.launch.py` /
`run_z_recovery_control.launch.py` call `os.sched_setaffinity(0, {3})` (and `run_sickPLC.launch.py` `{2}`) on the
`ros2 launch` process, and everything it starts inherits it:
- all the other threads of the `ros2_control_node` (DDS `dds.*`, executor/service threads, the `RealtimePublisher`
  threads, **including the one publishing `/state_controller/drive_states`**, controller_manager statistics);
- the nodes of that launch file that do not set their own affinity: spawners (transient), `homing_completion_publisher`,
  `ft_offset_updater`, `bag_recorder_node`, the platform `robot_state_publisher`, the launch process, `stdbuf`/`tee`
  (since `cfe7b62` only `ethercat_checker`, `filter_commands`, `fct_manager` move themselves off).
- on CPU 2 the same for the PLC process + `robot_state_publisher`, `sonar_teach`, `ethercat_slaves_status_check`.

Consequences:
1. **Non-RT publishers on CPU 3 can be delayed** (CPU 3 ~2/3 busy, bursts such as the DDS discovery of a new
   participant): most likely cause of the >0.2 s drive-states gaps that trip the brake watchdog (§1c).
2. RT jitter through shared caches/locks with the busy non-RT threads on the same CPU (the RT thread itself is FIFO
   and preempts them, so it is not starved: the platform loop is fine, −26/+26 µs; the PLC loop dispersion grows with
   load, std 2.4 → 7.0 µs).
3. Isolated CPUs are not load-balanced (`isolcpus=domain`): those threads cannot move elsewhere.

**Will the next steps fix it? Yes — step 3 (§5.1) is exactly this**: after it CPU 3 should host only the RT thread
(expected ~5–10% busy) and CPU 2 only the PLC RT thread; everything else runs on the housekeeping CPUs with
balancing. Cost: ~0.5–0.7 CPU more on the housekeeping CPUs (today ~50% busy after the load reduction); step 2
(§5.2, plc_manager drive_states only during START) frees ~0.3 CPU there, and option B (§5.4) adds CPUs 6–7 if needed.

Diagnostic before/after (who runs on CPU 2/3 and how much):
```bash
ps -eLo tid,psr,pcpu,cls,rtprio,comm --sort=-pcpu | awk 'NR==1 || $2==2 || $2==3' | head -25
```

## 2. Robot facts learned today (not in RT_DEBUG_NOTES.md)

- The teach pendant runs `C:\Fit4Med\ps_scripts\fmrr_bringup.ps1` (same MD5 as the repo, `DD0DD3ED…`) in
  **`service` mode**: `systemctl --user start fit4med-bringup@192.168.1.2.service`.
- The unit is a **per-IP instance file** `~/.config/systemd/user/fit4med-bringup@192.168.1.2.service`, not the repo
  template. Differs from the repo only by `TimeoutStopSec=15` (repo: 60). Align it.
- `user@1000.service` has `LimitRTPRIO=99`, `LimitMEMLOCK=infinity`: the RT threads get FIFO 50 and mlock also
  when started by the unit (verified: `FF 50` on CPU 2 and 3, `Successful set up FIFO`).
- **`Linger=no`** for `fit4med`: the user manager (and the stack) may stop ~10 s after the last session closes.
  Not yet checked whether this ever happened (§4).
- The test drop-in `fit4med-bringup@192.168.1.2.service.d/10-cpu-affinity.conf` was **removed** (default in code now).
- Only consumer of `/PLC_controller/plc_states`: `plc_manager`.
- `plc_manager` **also subscribes to `/state_controller/drive_states` at 250 Hz**
  (`ModeOfOperationMonitor`, `tecnobody_workbench_utils/utils.py:542`, inside the platform readiness monitor).
  It is only needed as success check of the START transition (CSP mode reached); at run time
  `check_env_running(False)` does not use it. This is most of `plc_manager`'s remaining ~50% CPU.
- The 17:37:36–38 `Releasing master` + `Failed to receive AL state datagram` is `reset_coe_faults` grabbing and
  releasing master 1 at the start of each platform start, not a bus fault.
- Before `cfe7b62`, `plc_manager` (starved on CPU 7, keep-last-10 queue) **skipped part of the `plc_states`**:
  ~183 callbacks/s for 500 messages/s. Short PLC input edges could be missed by the ROS supervision.

## 3. Measurements (statistics: `/…/statistics/full`, read period; values cumulative since process start)

| Run | PLC 500 Hz worst / std | Platform 250 Hz worst / std | Housekeeping CPUs 0,1,4,5 | Notes |
|---|---|---|---|---|
| Baseline 16:13 (`cpu_hogs_20261112-161333`) | −86/+70 µs, 2.8 µs | −94/+100 µs, 2.7 µs | ~3% | CPU 7 100%, CPU 6 73%; plc_manager wait 56% |
| Step 1 (drop-in, plc_manager → 0,1,4,5) `…-163103` | −108/+107, 2.4 | −82/+83, 2.5 | ~37% | plc_manager 111% CPU (no more skipped messages), wait 2% |
| Step 2 (`cfe7b62`) `…-170313` | −72/+75, 4.9 | −94/+93, 3.6 | **~68%** | CPU 6/7 0%; every Python node wait 10–14% |
| `e064176`, normal load (PLC stats include an estop + platform restart) | (−175/+170), 5.7 | **−26/+26, 2.9** | ~50% (partial window `…-173520`) | plc_manager 52% |
| `e064176`, heavy trajectories ~17:38–17:50 | (unchanged), **7.0** | **−43/+40, 2.7** | ? | no kernel EtherCAT message after 17:37:48 (to confirm, §4) |

Reading: the platform (drive) loop is now clearly better and robust to load. The PLC loop dispersion grows with
load: CPU 2 still hosts the whole PLC process (`run_sickPLC.launch.py` pins the launch to `{2}`: robot_state_publisher,
sonar_teach, ethercat_slaves_status_check, DDS/executor threads of ros2_control_node). Step 3 targets this.
All runs: 0 lost frames, 0 overruns. A clean PLC number (stack restart, no estop, stats at 11–12 min) is still missing.

## 4. To collect (if not done before leaving the machine)

```bash
# heavy-load window and kernel events (expected: nothing after 17:37:48)
ls -td ~/.ros/fit4med_diagnostics/cpu_hogs_* | head -1          # summary.txt of the heavy-load run, if any
journalctl -k --since "2026-11-12 17:37:50" | grep -i ethercat | tail -10
# step 3 prerequisite: does the robot's controller_manager support cpu_affinity?
strings /opt/ros/jazzy/lib/controller_manager/ros2_control_node | grep -iE "cpu_affinity|thread_priority"
# Linger=no: did the stack ever stop with no explicit stop request?
journalctl --user -u 'fit4med-bringup@*' --no-pager | grep -E "Stopping|Stopped|Started" | tail -20
journalctl -u user@1000.service --no-pager | tail -20
```

## 5. Next steps, in order

0. **Start here: step 3 (§5.1) for the CPU 3 overload (§1d) and the false drive-states trips (§1c).**
   Rule until it is deployed: no `ros2` CLI on the robot while the platform is RUNNING (a trip stops the machine,
   safely, key re-arm). Also open: why `plc_manager` sometimes stops running `state_callback` for >0.3 s (§1b; it
   delays the estop STOP).
1. **Step 3 — pin only the RT threads (fixes §1d).**
   a. Prerequisite on the robot: `strings /opt/ros/jazzy/lib/controller_manager/ros2_control_node | grep -iE
      "cpu_affinity|thread_priority"` (the WSL Jazzy binary has `realtime_tools::set_current_thread_affinity`).
      Check in the Jazzy sources/behaviour that `cpu_affinity` is applied **inside the RT thread only**, not to the
      whole process.
   b. Launch files `run_sickPLC`, `run_platform_control`, `run_z_recovery_control`, `run_PLC_safemod`: replace
      `os.sched_setaffinity(0, {2|3|2,3})` with `pin_to_housekeeping_cpus()`; controller_manager parameters
      (`tecnobody_workbench/config/*.yaml`) `cpu_affinity: [2]` (PLC) / `[3]` (platform, z_recovery),
      `thread_priority: 50`. safemod uses `{2,3}` today: decide which CPU it gets.
   c. Same commit: `run_rosbridge.launch.py` passes `cpu_affinity` = `','.join(housekeeping_cpus())` to
      `rosbridge_websocket_launch.xml` (`taskset -c` accepts `0,1,4,5`); fix the `cfe7b62` CHANGELOG wording about
      `run_rosbridge_websocket.launch.py` (that file is not used by the bring-up).
   d. Verify after deploy: `ps -eLo tid,cls,rtprio,psr,comm | grep ros2_control | grep FF` → `FF 50` on CPU 2 and 3;
      the §1d diagnostic shows nothing else on 2/3; `mpstat -P 2,3` ≈ 5–10%. **If the RT thread is not on its CPU
      (parameter not applied), stop and revert**: an RT thread floating on the housekeeping CPUs is worse than today.
      Fallback if the parameter is not supported: set the RT thread affinity from outside after start
      (`taskset -cp 3 <tid of the FF 50 thread>`) in the launch/bring-up script.
   e. Then the false-trip test of §1c: platform RUNNING, no motion, run `ros2 topic echo --once …` ~10 times →
      expected no `Brake closed and SW ESTOP raised`. Optional discriminating probe: an independent subscriber
      logging `/state_controller/drive_states` inter-arrival gaps during the test.
   f. Measure: `rt_cpu_hogs.sh`, statistics at 11–12 min (§6); housekeeping %wait (cost ~+0.5–0.7 CPU there).
2. **plc_manager: drive_states subscription only during the platform START transition** (create on
   `platform_controller_readiness_monitor.reset()`, destroy once `ok()`), or another low-rate source of the mode.
3. **Clean measurement**: restart the stack from the pendant (no estop), platform in OP, usual scenario;
   `rt_cpu_hogs.sh -d 600` 1–2 min after start; both statistics at 11–12 min; `%wait` extraction (§6).
   Then `bash_scripts/rt_latency_test.sh` with the stack running (histogram instead of a single max).
4. **Option B** (`isolcpus/nohz_full/rcu_nocbs=2-3` only) only if the housekeeping CPUs are still saturated
   after 1–3. The code already follows `/sys/devices/system/cpu/isolated`, no code change needed.
5. **Unit file**: align the robot instance unit with the repo template (`TimeoutStopSec=60`); consider installing
   the template and `loginctl enable-linger fit4med` (decide after §4 Linger check).
6. **Branches**: rebase/merge with `e901b4a`; push `rt-cpu-layout` when the user asks.
7. **Update `RT_DEBUG_NOTES.md`** (Windows clone) with §2–§3 of this file, plus: GRUB is pinned to rt11 with
   `GRUB_SAVEDEFAULT=false` and cmdline has `irqaffinity=0,1,4,5`; RTC workaround `fit4med-clock.service` +
   `fit4med-clock-save.timer` is installed; "hyperthreading ruled out" holds only for synthetic busy loops
   (test 2); the startup `Slave did not sync` is slow DC convergence, not jitter; add register `0x0134`
   (AL status code) to the post-failure dump; `rt_cpu_hogs.sh` prints `CPUS=4` instead of 8 (script bug).
8. Later / unchanged from RT_DEBUG_NOTES: no `printf`/`cout` in the driver RT loop; `reset_coe_faults` must call
   `ecrt_master_application_time()` before `ecrt_master_activate()`; build `rtla` from the kernel sources
   (`tools/tracing/rtla`, custom kernel); `ethercat_checker` drive_states rate; capture a real failure
   (time, what was seen, `ethercat reg_read -m1 -p<n>` of 0x0134, 0x0300, 0x0310, session folder).
9. **GUI after an estop — fixed robot-side in `fa33268`, verified 18:34–18:37** (3 consecutive estops with a
   movement running: each `RUNNING + STOP` → GUI `ROS_DISCONNECTED` in the same second → key → `ROS_CONNECTED`,
   no popup, no `ROS_CONNECTION_FAILED`). Cause: `plc_manager` sent the pending STOP to the GUI only after
   `kill_env` had stopped rosbridge; the GUI (polling every 0.2 s) often saw the drop first → unexpected-loss path.
   Fix: status with pending STOP/FAIL sent before `fsm.trigger()`, rosbridge stopped 0.5 s after the platform.
   Still open, low priority, GUI on the pendant: the unexpected-loss dialog (only for a real rosbridge crash now)
   has the focus/z-order bug below. History: every estop stops rosbridge with the platform stack, so the GUI showed
   "Restart GUI Required" (`rehab_gui/FMRRMainProgram.py` `onRuntimeRosCommunicationLost`) and the operator must
   restart it (seen 18:10 after the 18:00 estop test). Consider an automatic reconnection when the stack restarts.
   **Bug**: the GUI froze inside that dialog (OK did not respond; UDP status still updating; pendant clock 18:11 =
   robot ~18:07, the **pendant clock is 4 min ahead of the robot**). The GUI's UDP/ROS side kept working (it sent
   `ROS_CONNECTED` at 18:08:45 after the key restart). Recovered by killing the GUI (Task Manager) and restarting it:
   `ROS_DISCONNECTED` → `ROS_CONNECTED` at 18:12:06-07 while RUNNING, no FAIL. Likely the Qt
   thread blocked in `ros_manager.stopRosCommunication()` (called before `dialog.exec_()`) — **less likely**: the
   on-screen clock/"last UDP" labels kept updating, so the Qt thread was alive. User report: the 1st estop press
   restarts fine, the **2nd press** shows the dialog and the GUI stops responding; "never happened before today".
   **Main hypothesis: two nested modal dialogs.** The ROS-loss dialog is ApplicationModal + `WindowStaysOnTopHint`
   (`FMRRMainProgram.py:206`). If a movement/training is running, its stop request fails because rosbridge is gone and
   `TrainingProtocolWindow._onStopCompleted(False)` opens `QMessageBox.warning("Stop", "Arresto non confermato…")`
   (`TrainingProtocolWindow.py:639`), not on top: it becomes the active modal but is hidden behind the on-top dialog,
   so OK is ignored. Today's link: `plc_manager` (no longer starved on CPU 7) now stops the platform and rosbridge
   faster after the estop, before the GUI stops the movement through UDP — the GUI race was there, now it shows.
   Confirm: estop with a training running; when "frozen" press Esc/Enter or Alt+Tab (a hidden "Stop" box should
   appear/close, then OK works). **18:2x test: reproduced; Alt+Tab and back makes OK clickable** → window
   focus/z-order problem in the GUI, not a hang. After OK the key does not start: expected interlock — the GUI keeps
   reporting `ROS_CONNECTION_FAILED`, `plc_manager` START guard `_ros_gui_disconnected` refuses until the GUI is
   restarted (verify `Guard condition failed` / `Last received message` in the journal). Fix (GUI, pendant): no stop-failure box when ROS is lost; no `WindowStaysOnTopHint`
   on the ROS-loss dialog (or show it non-modal with `open()`). Get the GUI session log of that time (`fmrr_retrieve_logs.ps1` /
   pendant GUI logs) and check `RosCommunicationManager.stopRosCommunication` for blocking calls on the UI thread.
10. Pre-existing, unrelated: `test_eth_checker_brake.py::test_node_exit_closes_the_released_brake[2]` and `[15]`
   fail on `8dc7ba7` too.

## 6. How to measure (same protocol every time)

- Same scenario each run (bring-up from the pendant, platform in OP, GUI connected, same task); note the start time.
- `~/fit4med_ws/src/Fit4Med/bash_scripts/rt_cpu_hogs.sh -d 600` (or always the same `-d`).
- Statistics 11–12 min after the start of the processes (they are cumulative and include activation):
  ```bash
  ros2 topic echo --once /plc_controller_manager/statistics/full | grep -A1 -E "read_cycle/periodicity/(max|min|standard_deviation)|total_time"
  ros2 topic echo --once /controller_manager/statistics/full | grep -A1 -E "tecnobody.stats/read_cycle/periodicity/(max|min|standard_deviation)|controller_manager.total_time"
  ```
  Period error = 1/f − T: e.g. 522.4 Hz on a 2 ms period = −86 µs; std in µs ≈ T² · std(Hz).
- `%wait` per thread:
  ```bash
  cd ~/.ros/fit4med_diagnostics; d=$(ls -td cpu_hogs_* | head -1)
  grep -E "^Average:.*(plc_manager_nod|ethercat_checke|filter_commands|python3)$" $d/pidstat_cpu.txt | awk '$4!="-" && $9>1 {print $4, "wait="$8, "cpu="$9, $NF}'
  ```
- A shell needs `ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp` to see the topics.

## 7. Practical gotchas

- Deploy: stack **stopped** first; `--dry-run` per folder from WSL `~/fit4med_ws/src/Fit4Med`
  (`--folder Fit4Med/<pkg>`; the script does `rsync --delete --checksum`); only the expected `<f` lines, no
  `*deleting`; then the real sync and `colcon build --packages-select …` on the robot.
- The assistant cannot ssh to the robot directly (key refused); the sync script works from the user's WSL shell.
- WSL: the GUI venv `~/.vfit4med/bin` is first in `PATH` (no `catkin_pkg`, no `flake8`): for colcon/pytest strip it
  (`PATH=$(echo "$PATH" | tr ':' '\n' | grep -v vfit4med | paste -sd:)`).
- WSL clone had stale CRLF copies of 4 files (`git ls-files --eol | grep w/crlf`): fixed by rewriting them from HEAD;
  check again before a sync.
- The colcon "already built in underlay" warning on the robot is harmless (`.bashrc` sources the same workspace).
