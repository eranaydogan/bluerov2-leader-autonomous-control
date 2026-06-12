# BlueROV2 Leader Autonomous Control & Co-Simulation Scripts

This repository contains the autonomous navigation and manual test controllers operating over UDP, developed as part of the TÜBİTAK 2209-A supported project "BlueROV2 LED-Based Target Tracking System & Distributed Co-Simulation".

The system is designed to send kinematics and position data from a Windows-based command center to physics engines and visualization environments (Unity/Unreal Engine/Gazebo) at a rate of 60 packets per second (60 Hz).

## Contents

1. **`gem_leader_scan_v2.py`** - Autonomous Area Scan and Leak Detection (Pure Pursuit)
2. **`Klavyekontrol.py`** - Smooth Manual Keyboard Controller

---

## 1. Leader Autonomous Scan

This script enables the underwater vehicle (BlueROV2) to autonomously scan a designated area, approach the target by following a spiral trajectory when nearing a potential leak zone, and generate an emergency signal (EMERGENCY).

### Algorithms and Logic

* **Bilinear Interpolation:** Used to generate equally spaced strips between 4 defined corner points (`CORNER_A`, `B`, `C`, `D`).
* **Pure Pursuit:** Allows the vehicle to continuously adjust its heading toward a virtual target (carrot) located at a specific lookahead distance (`LOOKAHEAD_DIST`). 
    The heading error is calculated as follows:
    $$\theta_{error} = \arctan2(\Delta X, \Delta Z) - \theta_{current}$$
    The vehicle rotates within maximum yaw rate limits to minimize this error.
* **Spiral Target Approach:** When the vehicle approaches the leak source within the `R_TRIGGER` range, it stops the zigzag scan and follows a narrowing spiral toward the center. 
    Angular progression: $d\theta = (V_{cruise} / r_{current}) \cdot \Delta t$
* **Emergency Trigger:** Once the spiral radius reaches the `R_STOP` value, the vehicle stops and sends the `EMERGENCY` byte string to the simulation/mission controller via port `5012`.

### Usage

```bash
python gem_leader_scan_v2.py --mode mission

```

**Modes:**

* `mission`: Full autonomous area scanning and leak search mission.
* `drive-test`: Used to verify Unity/Simulation coordinates by driving point-to-point and stopping at corners.
* `yaw-test` / `move-test`: Specific test modes for heading and acceleration.

---

## 2. Smooth Keyboard Control

This is an FPS-independent manual control script developed to prevent sudden jumps/spikes when transferring vehicle dynamics to the simulation environment.

### Algorithms and Logic

* **FPS-Independent Exponential Smoothing:** Keyboard inputs are not directly translated to velocity. An asymptotic approach utilizing time delta (`dt`) is applied to reach the target speed:

$$V_{new} = V_{old} + (V_{target} - V_{old}) \times (1 - e^{-R \cdot \Delta t})$$



Here, $R$ (Response) determines the reaction time. This ensures smooth, jitter-free movement between packets.
* **Step Limits:** To prevent errors caused by network lag spikes, the maximum distance traveled per UDP packet is restricted by `MAX_POS_STEP_PER_PACKET`.

### Key Bindings and Modes

| Control Group | Keys | Function |
| --- | --- | --- |
| **Movement (Position)** | `W` / `S` | Forward / Backward |
|  | `D` / `A` | Right / Left |
|  | `E` / `C` | Up / Down |
| **Orientation (Rotation)** | `Up` / `Down` | Pitch |
|  | `Right` / `Left` | Yaw |
|  | `R` / `Q` | Roll |
| **Speed Modes** | `Shift` | Fast Mode |
|  | `(Default)` | Base Mode |
|  | `Ctrl` | Fine Mode |
| **System** | `X` | Reset Position/Rotation |

### Usage

Since this script uses a global keyboard listener, it requires the `pynput` library:

```bash
pip install pynput
python Klavyekontrol.py

```

---

## UDP Packet Format

Both scripts send data to the receiver in the simulation environment via port `5007` using the following C-Struct format (`<9f` - Little Endian, 9x 32-bit Floats).

Total packet size: **36 Bytes**.

| Index | Data | Type | Description |
| --- | --- | --- | --- |
| 0 | `X` | float | Right/Left Position (Unity X) |
| 1 | `Y` | float | Forward/Backward Position (Mapped to Unity Z) |
| 2 | `Z` | float | Height (Unity Y) |
| 3 | `Roll` | float | Roll Angle |
| 4 | `Pitch` | float | Pitch Angle |
| 5 | `Yaw` | float | Yaw Angle |
| 6 | `Time` | float | Time elapsed since startup (s) |
| 7 | `Seq` | float | Packet sequence number |
| 8 | `dT` | float | Time delta from the previous packet (s) |

---

*This system is designed targeting BlueROV2 research and future ROS2 integration.*