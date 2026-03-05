# FMRREHAB: Functional Movement Robotic Rehabilitation platform (a Fit4Med project)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-brightgreen)](https://docs.ros.org/en/jazzy/)
[![C++ Standard](https://img.shields.io/badge/C%2B%2B-17-blue)]()
[![Python Version](https://img.shields.io/badge/Python-3.12-blue)]()

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

#### 7. ps_scripts - PowerShell Remote Execution (Windows GUI)
 - fmrr_gui.ps1 - launch GUI
 - fmrr_bringup.ps1 - SSH into Linux, start ROS2 stack via PLC launcher
 - fmrr_cleanup.ps1 - SSH into Linux, gracefully shut down all nodes
 - fmrr_scp.ps1 - Transfer files remotely between Windows & Linux

#### 8. tecnobody_msgs - Custom Message and Services Types
 - PlcController.msg - Command messages to PLC
 - PlcStates.msg - State feedback from PLC
 - SetExercise.srv - Exercise execution request
 - SetTrajectory.srv - Tajectory execution request


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
This project is licensed under the MIT License. See LICENSE file for details.

## Contact
For questions about this rehabilitation platform, contact CNR-STIIMA-IRAS team