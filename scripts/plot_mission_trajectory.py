import math
import numpy as np
import matplotlib.pyplot as plt


# ============================================================
# SAME PARAMETERS AS leader_autonomous_scan.py
# ============================================================

START_X = -152.0
START_Y = -145.35
START_Z = 923.0

CORNER_A = (-152.0, 915.0)
CORNER_B = (-100.0, 915.0)
CORNER_C = (-100.0, 860.0)
CORNER_D = (-150.0, 860.0)

LEAK_X = -114.991
LEAK_Z = 871.2

R_TRIGGER = 14.0
R_STOP = 6.0
SPIRAL_TURNS = 2.5

NUM_STRIPS = 5
TURN_RADIUS = 9.0


# ============================================================
# HELPERS (same logic)
# ============================================================

def lerp2(a, b, t):
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
    )

def bilinear(A, B, C, D, u, v):
    bottom = lerp2(A, B, u)
    top = lerp2(D, C, u)
    return lerp2(bottom, top, v)

def build_scan_waypoints():
    A, B, C, D = CORNER_A, CORNER_B, CORNER_C, CORNER_D
    R = TURN_RADIUS
    wps = []

    for s in range(NUM_STRIPS):
        v = s / (NUM_STRIPS - 1) if NUM_STRIPS > 1 else 0.0

        left = bilinear(A, B, C, D, 0.0, v)
        right = bilinear(A, B, C, D, 1.0, v)

        if s % 2 == 0:
            entry, exit_ = left, right
            out_x = right[0] + R
        else:
            entry, exit_ = right, left
            out_x = left[0] - R

        wps.append(entry)
        wps.append(exit_)

        if s < NUM_STRIPS - 1:
            v_next = (s + 1) / (NUM_STRIPS - 1)
            z_mid = (
                exit_[1] +
                bilinear(A, B, C, D, 0.0, v_next)[1]
            ) / 2.0
            wps.append((out_x, z_mid))

    return wps


# ============================================================
# PLOT
# ============================================================

def make_spiral():
    theta = np.linspace(0, 2 * math.pi * SPIRAL_TURNS, 400)
    r = np.linspace(R_TRIGGER, R_STOP, theta.size)

    x = LEAK_X + r * np.sin(theta)
    z = LEAK_Z + r * np.cos(theta)
    return x, z

def main():
    waypoints = build_scan_waypoints()

    fig, ax = plt.subplots(figsize=(10, 8))

    # Scan area polygon
    area_x = [CORNER_A[0], CORNER_B[0], CORNER_C[0], CORNER_D[0], CORNER_A[0]]
    area_z = [CORNER_A[1], CORNER_B[1], CORNER_C[1], CORNER_D[1], CORNER_A[1]]
    ax.plot(area_x, area_z, linewidth=2, label="Scan Area")

    # Coverage path
    wp_x = [p[0] for p in waypoints]
    wp_z = [p[1] for p in waypoints]
    ax.plot(wp_x, wp_z, marker="o", linewidth=1.8, markersize=4, label="Coverage Path")

    # Start point
    ax.scatter([START_X], [START_Z], s=100, marker="s", label="Start")
    ax.annotate("Start", (START_X, START_Z), xytext=(6, 6), textcoords="offset points")

    # Waypoint numbering (optional, small)
    for i, (x, z) in enumerate(waypoints, start=1):
        ax.annotate(str(i), (x, z), xytext=(4, 4), textcoords="offset points", fontsize=8)

    # Leak / target point
    ax.scatter([LEAK_X], [LEAK_Z], s=180, marker="*", label="Leak Source / Target")
    ax.annotate("Leak Source", (LEAK_X, LEAK_Z), xytext=(8, -12), textcoords="offset points")

    # Trigger and stop circles
    trig = plt.Circle((LEAK_X, LEAK_Z), R_TRIGGER, fill=False, linestyle="--", linewidth=1.8, label="R_TRIGGER")
    stop = plt.Circle((LEAK_X, LEAK_Z), R_STOP, fill=False, linestyle=":", linewidth=1.8, label="R_STOP")
    ax.add_patch(trig)
    ax.add_patch(stop)

    # Spiral
    sx, sz = make_spiral()
    ax.plot(sx, sz, linewidth=2.0, label="Contracting Spiral")

    # Spiral end / emergency point
    ax.scatter([sx[-1]], [sz[-1]], s=80, marker="X", label="Emergency Trigger")
    ax.annotate("Emergency Trigger", (sx[-1], sz[-1]), xytext=(8, 8), textcoords="offset points")

    # Direction arrows along coverage path
    for i in range(0, len(waypoints) - 1, 2):
        x1, z1 = waypoints[i]
        x2, z2 = waypoints[i + 1]
        mx = 0.5 * (x1 + x2)
        mz = 0.5 * (z1 + z2)
        dx = x2 - x1
        dz = z2 - z1
        ax.arrow(mx, mz, dx * 0.12, dz * 0.12, head_width=1.2, head_length=1.8, length_includes_head=True)

    # Formatting
    ax.set_title("Autonomous Leader Mission Trajectory")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Z [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    plt.tight_layout()
    plt.savefig("docs/media/mission_trajectory.png", dpi=220, bbox_inches="tight")
    plt.show()


if __name__ == "__main__":
    main()
