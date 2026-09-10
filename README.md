# BlueROV2 Leader Autonomous Mission & Co-Simulation Control

Autonomous leader-vehicle trajectory generation and manual control tools for a BlueROV2-based distributed underwater robotics simulation.

This repository contains the **leader / mission-generation subsystem** of the TÜBİTAK 2209-A supported graduation project:

**BlueROV2 LED-Based Target Tracking System & Distributed Co-Simulation**

The leader follows a predefined coverage mission, transitions to a spiral approach around a simulated leak-source region, and sends pose updates to the visualization environment over UDP.

Related project repositories:

- [BlueROV2 LED Tracking & Perception](https://github.com/eranaydogan/bluerov2-led-tracking-opencv)
- [BlueROV2 Visual Following Control](https://github.com/eranaydogan/bluerov2-led-control)

---

## System Role

The broader project separates leader mission generation, visual perception, and follower control into different subsystems.

```text
Leader mission generator
        ↓
UDP pose stream
        ↓
Unity leader vehicle
        ↓
camera / LED visual observation
        ↓
OpenCV perception
        ↓
UDP target observation
        ↓
Follower controller
        ↓
ArduSub / Gazebo BlueROV2
```

This repository is responsible for the first part of that chain:

```text
mission geometry
      ↓
waypoint generation
      ↓
lookahead steering
      ↓
kinematic motion
      ↓
UDP pose output
```

---

## Repository Structure

```text
.
├── scripts/
│   ├── leader_autonomous_scan.py
│   ├── manual_keyboard_control.py
│   └── legacy/
│       └── manual_keyboard_control_6dof.py
│
├── requirements.txt
├── .gitignore
└── README.md
```

---

# Autonomous Leader Mission

The main autonomous script is:

```text
scripts/leader_autonomous_scan.py
```

It generates a complete simulated leader mission using a lightweight kinematic controller.

The mission consists of:

```text
startup hold
    ↓
coverage scan
    ↓
lookahead-based heading control
    ↓
approach to configured leak region
    ↓
contracting spiral
    ↓
mission stop
    ↓
EMERGENCY trigger
```

---

## Coverage Scan

The scan region is defined by four corner points:

```text
CORNER_A
CORNER_B
CORNER_C
CORNER_D
```

Intermediate scan strips are generated using bilinear interpolation.

The resulting trajectory alternates direction between strips:

```text
→→→→→
      ↓
←←←←←
↓
→→→→→
      ↓
←←←←←
```

Additional outer waypoints are inserted between strips to create smoother U-turn transitions rather than forcing instantaneous heading changes.

---

## Lookahead Steering

During the coverage phase, the vehicle does not simply aim directly at each waypoint.

Instead, a virtual target is placed ahead of the current path progress using a configurable lookahead distance.

```text
vehicle
   ↓
current path position
        ↓
        ↓ LOOKAHEAD_DIST
        ↓
virtual target
```

The target heading is calculated from:

```text
target_heading = atan2(dx, dz)
```

and the vehicle heading changes gradually using a bounded yaw rate.

This produces smoother path transitions and reduces oscillatory waypoint behavior.

---

## Kinematic Motion Model

The leader motion is generated using a lightweight planar kinematic model.

Forward velocity is derived from the current heading:

```text
vx = sin(yaw) * speed
vz = cos(yaw) * speed
```

Velocity transitions are smoothed before integration.

Each UDP update is also protected by a maximum position step:

```text
MAX_POS_STEP_PER_PACKET
```

to prevent large jumps caused by abnormal timing intervals.

---

## Spiral Source Approach

The mission contains a configured simulated leak-source position:

```text
LEAK_X
LEAK_Z
```

When the vehicle enters the configured trigger radius:

```text
R_TRIGGER
```

the normal coverage scan is interrupted and a contracting spiral begins around the target region.

The spiral radius decreases from approximately:

```text
R_TRIGGER → R_STOP
```

over a configurable number of turns.

The angular progression is based on the current spiral radius and cruise speed.

A separate maximum yaw rate is used during the spiral because tighter-radius motion requires greater turning authority than the normal coverage scan.

> The current implementation uses a predefined simulated source location.  
> It does not perform real sensor-based leak detection.

---

## Emergency Trigger

When the vehicle reaches the end of the spiral, or enters the configured fallback distance, the mission stops.

The script then sends:

```text
EMERGENCY
```

over UDP to:

```text
port 5012
```

The trigger is transmitted multiple times to reduce the effect of possible UDP packet loss.

This signal can be used by the broader simulation to initiate emergency behavior such as follower ascent or mission termination.

---

## Mission Modes

The autonomous script provides several operating modes.

### Full mission

```bash
python scripts/leader_autonomous_scan.py --mode mission
```

Runs:

```text
coverage scan
→ simulated source approach
→ contracting spiral
→ emergency trigger
```

### Drive test

```bash
python scripts/leader_autonomous_scan.py --mode drive-test
```

Moves between the four configured region corners and pauses at each location.

This mode was used to verify coordinate alignment between the trajectory generator and the simulation environment.

### Yaw test

```bash
python scripts/leader_autonomous_scan.py --mode yaw-test
```

Provides a simple heading-direction test.

### Move test

```bash
python scripts/leader_autonomous_scan.py --mode move-test
```

Provides a minimal point-to-point movement test.

Yaw output can also be adjusted without modifying the code:

```text
--yaw-sign
--yaw-offset
```

---

# UDP Pose Interface

The leader pose is transmitted at:

```text
60 Hz
```

using UDP.

Default destination:

```text
127.0.0.1:5007
```

The packet format is:

```text
<9f
```

which corresponds to nine little-endian 32-bit floating-point values.

| Index | Field | Description |
|---|---|---|
| 0 | X | Relative horizontal position |
| 1 | Y | Forward-axis representation used by the Unity receiver |
| 2 | Z | Relative vertical position |
| 3 | Roll | Roll angle |
| 4 | Pitch | Pitch angle |
| 5 | Yaw | Heading angle |
| 6 | Time | Time since script start |
| 7 | Sequence | Packet sequence number |
| 8 | Δt | Time since previous update |

Total payload size:

```text
36 bytes
```

The autonomous mission currently keeps roll and pitch fixed while generating planar leader motion.

---

# Manual Keyboard Control

The current manual test controller is:

```text
scripts/manual_keyboard_control.py
```

It was used for coordinate and motion verification before or alongside autonomous mission testing.

Controls:

```text
W / S       → forward / backward
A / D       → lateral motion
Left / Right Arrow → yaw
ESC         → exit
```

Movement is calculated relative to the current vehicle heading.

Local forward and lateral commands are transformed into global planar motion before the pose is transmitted over UDP.

Run with:

```bash
python scripts/manual_keyboard_control.py
```

Dependency:

```text
pynput
```

---

## Legacy Manual Controller

An earlier six-degree-of-freedom-style keyboard controller is preserved under:

```text
scripts/legacy/manual_keyboard_control_6dof.py
```

It includes:

- translational motion,
- vertical motion,
- roll,
- pitch,
- yaw,
- multiple speed modes,
- exponential velocity smoothing,
- per-packet motion limits.

It is retained as development history and is not the primary manual controller.

---

# Installation

Create a virtual environment:

```bash
python -m venv .venv
```

Linux/macOS:

```bash
source .venv/bin/activate
```

Windows:

```powershell
.venv\Scripts\Activate.ps1
```

Install dependencies:

```bash
pip install -r requirements.txt
```

The autonomous mission itself uses only Python standard-library modules.

`pynput` is required for keyboard-control scripts.

---

# Design Highlights

The project demonstrates several robotics and simulation concepts:

- coverage-path waypoint generation,
- bilinear interpolation,
- lookahead-based path following,
- bounded yaw-rate steering,
- velocity smoothing,
- contracting spiral trajectories,
- mission-state transitions,
- UDP pose streaming,
- simulation coordinate mapping,
- redundant emergency-event transmission,
- modular distributed co-simulation.

---

# Relationship to the Follower System

The leader mission is designed to generate motion that the follower must observe and react to.

The related perception system extracts:

```text
horizontal visual error
relative distance estimate
LED pattern information
```

from the Unity image.

The follower controller then converts those observations into MAVLink commands for the Gazebo / ArduSub BlueROV2.

Together, the repositories represent three major layers:

```text
Leader Mission
      ↓
Perception
      ↓
Follower Control
```

---

# Current Scope and Limitations

This repository is a research and simulation prototype.

Current limitations include:

- leak-source coordinates are predefined rather than sensor-estimated,
- leader motion uses a kinematic model rather than a full underwater dynamic model,
- mission geometry is currently configured directly in the script,
- communication uses UDP without acknowledgement,
- roll and pitch are fixed during the autonomous mission,
- the current coverage strategy is designed for the configured simulation region,
- obstacle avoidance is not implemented.

Future work could replace the predefined source position with sensor-driven search and integrate the leader mission directly with a vehicle dynamics/autopilot layer.

---

# Project Context

**Graduation Project**  
**Funded by TÜBİTAK 2209-A**  
**Role: Project Lead**

The broader project explores distributed simulation, autonomous mission generation, computer vision, relative-distance estimation, closed-loop vehicle control, and emergency behavior for underwater robotic systems.
