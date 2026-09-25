<!-- Copyright 2026 CNR-STIIMA -->
<!--  SPDX-License-Identifier: CC0-1.0 -->

# FMRREHAB: Functional Movement Robotic Rehabilitation platform (a Fit4Med project)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-brightgreen)](https://docs.ros.org/en/jazzy/)
[![C++ Standard](https://img.shields.io/badge/C%2B%2B-17-blue)]()
[![Python Version](https://img.shields.io/badge/Python-3.12-blue)]()
[![ROS 2 Build](https://github.com/CNR-STIIMA-IRAS/Fit4Med/actions/workflows/ros2.yml/badge.svg)](https://github.com/CNR-STIIMA-IRAS/Fit4Med/actions/workflows/ros2.yml)
[![License check (REUSE)](https://github.com/CNR-STIIMA-IRAS/Fit4Med/actions/workflows/reuse.yml/badge.svg)](https://github.com/CNR-STIIMA-IRAS/Fit4Med/actions/workflows/reuse.yml)

**Fmrrehab** is a comprehensive rehabilitation robotics platform built on ROS 2 with advanced motion control, force-torque sensing, and safety-critical PLC integration. The system coordinates a 3-DOF rehabilitation arm through a real-time control architecture with emergency stop chains and sensor-based impedance control.


## System Architecture
Dual Controller Manager Setup
The system uses two separate controller-manager instances for safety isolation:

### Safety control stack (always running)      
The safety PLC module installed in the FMRREHAB platform is managed by the following ros2 nodes at a 500 Hz control loop:
PLC Controller Manager<br>
└─ PLC Controller
    - Reads/writes 8 GPIO command outputs   
    - Reads 8 GPIO state inputs          
    - Publishes PlcStates for subscribers        

### Motion control stack (started by the user)
The following controllers are automathically loaded and will run at 250 Hz:
- Joint Trajectory Controller
- Forward Velocity Controller 
- Impedance Controller
- Force/Torque Sensor Broadcaster   

### Rehabilitation GUI       
(PyQt5 Interface with SSH Remote Access)<br>
High-Level user interface with SSH connection for remote launching and UDP 
client for status monitoring


## Repository Structure

### Core Packages

#### 1. **tecnobody_workbench** - Central Configuration Package
Master configuration package containing all URDF definitions, controller YAML configs, and launch files.

#### 2. **plc_manager** - Safety State Machine
ROS 2 node managing system lifecycle through PLC state transitions.

#### 3. **plc_controller** - ROS 2 Control Plugin
C++ controller plugin implementing GPIO bridge between ROS 2 and Safety PLC.

#### 4. **tecnobody_workbench_utils** - Core Control Nodes
This package provides the primary motion control and sensor management modules needed by the motion control stack.

#### 5. **rehab_gui** - User Interface
Multi-Platform user interface developed with QT managing robot high-level control by the user and rehabilitation exercise configuration.

### Supporting Packages

#### 6. bash_scripts - ROS 2 Environment Management

launch_ros2_env.sh<br>
Launches ros2 control framework and default controllers, managing two cases:<br>
 - first launch: homing process performed 
 - launch after an emergency stop by the user: no homing needed

kill_ros_apps.sh<br>
 - Gracefully unspawn all active controllers
 - Unload inactive controllers
 - Coordinate with PLC manager for clean shutdown

fit4med_to_robot_sync.sh / fit4med_from_robot_sync.sh<br>
 - To robot: make each folder of the workspace `src` containing your checkout identical on the robot (robot 192.168.1.1, `--dry-run` to preview). Robot folders you don't have locally are left untouched. It first offers a backup, and asks before deleting files that exist only on the robot.
 - From robot: pull the robot's changes, keeping local files that are newer.

fit4med_backup.sh / fit4med_restore.sh<br>
 - Back up the robot sources to `/home/fit4med/bkp/YYYYMMDD/HHMM/`, or list the backups and restore one.
 - Run them on the robot, or from your PC (they work through ssh; `--host <ip>` / `--local` to force).

#### 7. ps_scripts - PowerShell Remote Execution (Windows GUI)
 - fmrr_gui.ps1 - launch GUI
 - fmrr_bringup.ps1 - SSH into Linux, start ROS2 stack via PLC launcher
 - fmrr_cleanup.ps1 - kill the local GUI (and whatever holds UDP 5005), then SSH into Linux and shut down all nodes (`-Target gui|remote` to do only one side)
 - fmrr_scp.ps1 - Transfer files remotely between Windows & Linux
 - fmrr_to_robot_sync.ps1 / fmrr_backup.ps1 / fmrr_restore.ps1 - Windows versions of the robot sync, backup and restore scripts (see bash_scripts/README.md)

#### 8. tecnobody_msgs - Custom Message and Services Types
 - PlcController.msg - Command messages to PLC
 - PlcStates.msg - State feedback from PLC
 - SetExercise.srv - Exercise execution request
 - SetTrajectory.srv - Tajectory execution request


## Setting up a GUI PC from scratch (Windows)

The GUI is a plain Python/PyQt5 application: it never uses `rclpy`, it talks to
the platform over **rosbridge (TCP 9090)** and receives status over **UDP 5005**
(sent by `plc_manager` from UDP 5006, where the GUI replies with its ROS state).
No ROS 2 installation is required on the Windows PC.

Two addresses matter and only one of them ever changes:

| Setting | Value | Where |
|---|---|---|
| `--remote-ip` | `192.168.1.1` (ROS 2 PC) — never changes | `ps_scripts/fmrr_gui.ps1` |
| `gui_ip` | the static IP of *this* Windows PC | `-GuiIp` argument of `ps_scripts/fmrr_bringup.ps1` |

### 1. Network

Give the PC a **static** address on the platform subnet `192.168.1.0/24`.
`192.168.1.1` is the ROS 2 PC and `192.168.1.2` is the platform's own GUI PC, so
a second machine needs a free address (e.g. `192.168.1.57`). That address is the
`gui_ip`: `plc_manager` sends the UDP status stream to it, and nothing else
tells it where the GUI lives.

Allow the status stream in, from an **Administrator** PowerShell:

```powershell
New-NetFirewallRule -DisplayName "FMRREHAB GUI status" -Direction Inbound -Protocol UDP -LocalPort 5005 -Action Allow
```

The bringup/cleanup scripts use the Windows OpenSSH client (included in Windows
10/11) to log into `fit4med@192.168.1.1`, so make sure `ssh 192.168.1.1` works
before going further.

### 2. Install uv

```powershell
winget install --id=astral-sh.uv -e
```

or, without winget:

```powershell
irm https://astral.sh/uv/install.ps1 | iex
```

Close and reopen PowerShell, then check `uv --version`. Python itself does not
need to be installed: uv downloads the interpreter this project asks for
(CPython >= 3.10, < 3.13).

### 3. Clone the repository

```powershell
git clone https://github.com/CNR-STIIMA-IRAS/Fit4Med.git
cd Fit4Med
```

Any folder works — the PowerShell scripts resolve their paths relative to their
own location, so the repository no longer has to sit in `C:\Fit4Med`.

### 4. Create the environment

```powershell
uv sync
```

This creates `.venv` in the repository root with PyQt5, roslibpy, NumPy, SciPy,
Matplotlib, psutil, PyYAML and rich, as pinned by `pyproject.toml` / `uv.lock`.
Movements and protocols are read from `rehab_gui/Movements` and
`rehab_gui/Protocols` inside the clone.

### 5. Allow the scripts to run (once per machine)

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

Alternatively, launch each script as
`powershell -ExecutionPolicy Bypass -File ps_scripts\fmrr_gui.ps1`.

### 6. Start the platform, passing this PC's address

```powershell
.\ps_scripts\fmrr_bringup.ps1 -GuiIp 192.168.1.57
```

This SSHes into the ROS 2 PC and runs
`ros2 launch tecnobody_workbench run_sickPLC.launch.py gui_ip:=192.168.1.57`.
Without `-GuiIp` it falls back to the platform PC (`192.168.1.2`), which is the
behaviour on the machine in the lab. Leave the window open: it holds the ROS 2
session.

### 7. Start the GUI

```powershell
.\ps_scripts\fmrr_gui.ps1
```

The script uses `.venv\Scripts\python.exe` when the repository has an
environment and the system `python` otherwise, and always connects to
`--remote-ip 192.168.1.1`. The equivalent manual command is:

```powershell
uv run python rehab_gui\rehab_gui\FMRRMainProgram.py --remote-ip 192.168.1.1 --maximise-window
```

### 8. Shut down

```powershell
.\ps_scripts\fmrr_cleanup.ps1
```

### If the GUI opens but shows no status

The window and the rosbridge connection work over TCP 9090, while the status
fields are fed by the UDP stream. An empty status panel therefore almost always
means the `gui_ip` given at step 6 is not this PC's address, or the firewall
rule of step 1 is missing.


## Troubleshooting

Common Issues
1. EtherCAT PLC Not Operational
Check EtherCAT slaves:
ethercat slaves # Expected: "FLX0-GETC100 OP"
ethercat master # If not OP, check ethercat master

2. Homing Timeout
- Check motor connection (ESTOP signal must be active)
- Verify safe movement zone (no mechanical obstructions)
- Increase timeout in boot_hw.py if needed

3. GUI Not Receiving Status
- Verify UDP port 5005 is open
- Check network connectivity (SSH must work)
- Verify GUI IP in plc_manager launch: gui_ip:=<windows_ip>

4. Trajectory Execution Hangs
- Check joint_trajectory_controller state: ros2 control list_controllers
- Verify all joints in "active" state
- Check for controller timeouts in /tmp/launch logs

## License
This project is licensed under the Apache-2.0 License. See LICENSE file for details.

## Contact
For questions about this rehabilitation platform, contact [Nicola Pedrocchi](mailto:nicola.pedrocchi@cnr.it?subject=[GITHUB%20Fit4Med]%20Request%20of%20information) and [Adriano Scibilia](mailto:adriano.scibilia@cnr.it?subject=[GITHUB%20Fit4Med]%20Request%20of%20information)