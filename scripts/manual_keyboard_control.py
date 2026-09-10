#!/usr/bin/env python3
"""
MANUAL KEYBOARD CONTROLLER — WASD + Yaw
============================================================

Requirement:
    pip install pynput

Controls:
- W / S: Move forward / backward relative to the current heading
- A / D: Strafe left / right relative to the current heading
- Left / Right Arrow: Yaw left / right
- ESC: Exit
"""

import socket
import struct
import time
import math

from pynput import keyboard


# ============================================================
# CONFIGURATION
# ============================================================

UDP_IP = "127.0.0.1"
UDP_PORT = 5007

SEND_HZ = 60.0
DT = 0.8 / SEND_HZ

# Initial world coordinates used as the reference origin.
START_X = -152.0
START_Y = -145.35
START_Z = 923.0

# Motion parameters.
MOVE_SPEED = 1.0       # World units per second
YAW_RATE = 5.0         # Degrees per second


# ============================================================
# UDP PACKET FORMAT
# ============================================================

packet_format = struct.Struct("<9f")


# ============================================================
# KEYBOARD STATE
# ============================================================

pressed_keys = set()


def on_press(key):
    """Store pressed keys in the active-key set."""
    pressed_keys.add(key)


def on_release(key):
    """Remove released keys and stop the listener when ESC is released."""
    if key in pressed_keys:
        pressed_keys.remove(key)

    if key == keyboard.Key.esc:
        return False


# ============================================================
# HELPERS
# ============================================================

def wrap_deg(angle):
    """Wrap an angle to the range [-180, 180) degrees."""
    return (angle + 180.0) % 360.0 - 180.0


def clamp(value, lo, hi):
    """Clamp a value to the given interval."""
    return max(lo, min(hi, value))


def send_pose(sock, wx, wz, height, yaw, t, seq, dt):
    """
    Convert world coordinates to the relative pose representation
    expected by the Unity receiver and transmit the packet over UDP.

    Roll and pitch are kept fixed at zero in this controller.
    """
    data = packet_format.pack(
        float(wx - START_X),       # Unity X: right
        float(-(wz - START_Z)),    # Unity Z: forward
        float(height - START_Y),   # Unity Y: up
        0.0,                       # Roll
        0.0,                       # Pitch
        float(yaw),                # Yaw
        float(t),
        float(seq),
        float(dt),
    )

    sock.sendto(
        data,
        (UDP_IP, UDP_PORT),
    )


# ============================================================
# MAIN
# ============================================================

def main():
    sock = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    )

    # Initial position and orientation.
    wx, wz = START_X, START_Z
    height = START_Y

    # Vehicle initially faces the -Z direction.
    yaw = 180.0

    seq = 0

    start_time = time.perf_counter()
    last_time = start_time

    # Start the keyboard listener in a non-blocking background thread.
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release,
    )
    listener.start()

    print("=" * 60)
    print(
        f"MANUAL KEYBOARD CONTROL ACTIVE "
        f"-> UDP TARGET: {UDP_IP}:{UDP_PORT}"
    )
    print(
        "W/S: Forward/Backward | "
        "A/D: Left/Right | "
        "Arrow Left/Right: Yaw | "
        "ESC: Exit"
    )
    print("=" * 60)

    try:
        while listener.running:
            loop_start = time.perf_counter()
            now = loop_start

            # Clamp abnormally large time steps to prevent motion jumps.
            dt = clamp(
                now - last_time,
                0.0,
                0.05,
            )
            last_time = now

            # --------------------------------------------------------
            # 1. ORIENTATION UPDATE — YAW ONLY
            # --------------------------------------------------------

            if keyboard.Key.left in pressed_keys:
                yaw -= YAW_RATE * dt

            if keyboard.Key.right in pressed_keys:
                yaw += YAW_RATE * dt

            yaw = wrap_deg(yaw)

            # --------------------------------------------------------
            # 2. LOCAL MOTION COMMANDS — WASD
            # --------------------------------------------------------

            move_fwd = 0.0
            move_side = 0.0

            if hasattr(keyboard, "KeyCode"):
                if keyboard.KeyCode.from_char("w") in pressed_keys:
                    move_fwd += MOVE_SPEED

                if keyboard.KeyCode.from_char("s") in pressed_keys:
                    move_fwd -= MOVE_SPEED

                if keyboard.KeyCode.from_char("d") in pressed_keys:
                    move_side += MOVE_SPEED

                if keyboard.KeyCode.from_char("a") in pressed_keys:
                    move_side -= MOVE_SPEED

            # --------------------------------------------------------
            # 3. LOCAL-TO-WORLD KINEMATIC TRANSFORMATION
            # --------------------------------------------------------

            yaw_rad = math.radians(yaw)

            # Project body-relative forward/lateral velocity
            # into the world X-Z plane.
            dx = (
                move_fwd * math.sin(yaw_rad)
                + move_side * math.cos(yaw_rad)
            )

            dz = (
                move_fwd * math.cos(yaw_rad)
                - move_side * math.sin(yaw_rad)
            )

            wx += dx * dt
            wz += dz * dt

            # --------------------------------------------------------
            # 4. PACK AND TRANSMIT POSE
            # --------------------------------------------------------

            try:
                send_pose(
                    sock,
                    wx,
                    wz,
                    height,
                    yaw,
                    now - start_time,
                    seq,
                    dt,
                )

            except Exception as exc:
                print(
                    f"[ERROR] UDP transmission failed: {exc}"
                )
                break

            # Print status approximately twice per second.
            if seq % int(SEND_HZ / 2) == 0:
                print(
                    f"Position: ({wx:+.2f}, {wz:+.2f}) "
                    f"| Yaw: {yaw:+.1f}"
                )

            seq += 1

            # Maintain the configured update frequency.
            sleep_time = (
                DT
                - (
                    time.perf_counter()
                    - loop_start
                )
            )

            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass

    finally:
        print(
            "\n[INFO] Shutting down manual controller..."
        )

        listener.stop()
        sock.close()


if __name__ == "__main__":
    main()
