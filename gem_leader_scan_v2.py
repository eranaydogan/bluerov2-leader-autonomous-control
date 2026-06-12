#!/usr/bin/env python3
"""
LIDER OTONOM ALAN TARAMA SCRIPTI
============================================================

- Pozisyon: dunya-frame delta (wx-START_X, -(wz-START_Z), height-START_Y)
- Otonom Ucus Kinematigi (Pure Pursuit & Salinim Engelleme)
- Baslangic heading'i ilk serit yonune (wp0 -> wp1) gore kurulur
"""

import socket
import struct
import time
import math
import argparse

# ============================================================
# CONFIG
# ============================================================

EMERGENCY_IP = "127.0.0.1"
EMERGENCY_PORT = 5012

START_X = -152.0
START_Y = -145.35
START_Z = 923.0

CORNER_A = (-152.0, 915.0)
CORNER_B = (-100.0, 915.0)
CORNER_C = (-100.0, 860.0)
CORNER_D = (-150.0, 860.0)

VEL_SMOOTH_TAU = 0.4

LEAK_X = -114.991
LEAK_Z = 871.2
# ---- Sizinti spiral ayarlari ----
R_TRIGGER = 14.0        # bu menzile girince spirale gec (tarama biter)
R_STOP = 6              # spiral bu yaricapa inince dur + emergency
SPIRAL_TURNS = 2.5      # R_TRIGGER'dan R_STOP'a inerken kac tam tur

# Spiral icin OZEL yaw rate: en kucuk yaricapi (R_STOP) donebilmeli.
# Tarama icin MAX_YAW_RATE_RAD (genis/yumusak) kullanilir; spiral'de
# daha keskin donus gerektigi icin bu daha yuksek deger devreye girer.
# (CRUISE_SPEED asagida tanimlandiktan SONRA hesaplaniyor -- bkz. altta.)

NUM_STRIPS = 5

# ---- Hareket Kinematigi ---- cruise speed 0.80 Max_pose_step_per_packet 0.032 iken iyi sonuc veriyor
SEND_HZ = 60.0
CRUISE_SPEED = 1
TURN_RADIUS = 9
MAX_YAW_RATE_RAD = CRUISE_SPEED / TURN_RADIUS
LOOKAHEAD_DIST = 10.0   # 16 -> 12: biraz daha siki U donusu (cok dusurme: salinim geri gelir)

# >>> SPIRAL'E OZEL YAW RATE (CRUISE_SPEED ve R_STOP tanimli oldugu icin BURADA) <<<
# R_STOP yaricapli cemberi donmek icin gereken min yaw = CRUISE_SPEED / R_STOP.
# 0.7 carpani -> bir miktar fazladan donus kapasitesi (guvenlik payi).
SPIRAL_MAX_YAW_RATE = CRUISE_SPEED / (R_STOP * 0.7)

MAX_POS_STEP_PER_PACKET = 0.04
WAYPOINT_TOL = 0.5
STARTUP_HOLD_SEC = 6

# Emergency'yi spiral tam bitmese bile garantiye almak icin:
# leak'e GERCEK mesafe bu degerin altina inerse de emergency gonderilir.
EMERGENCY_DIST_FALLBACK = R_STOP + 4.0   # ~8 birim: spiral takilsa bile emergency garantili

# ============================================================
UDP_IP = "127.0.0.1"
UDP_PORT = 5007
DT = 1.0 / SEND_HZ
packet_format = struct.Struct("<9f")

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0

def wrap_rad(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def lerp2(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)

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
        left  = bilinear(A, B, C, D, 0.0, v)   # sol kenar (x kucuk)
        right = bilinear(A, B, C, D, 1.0, v)   # sag kenar (x buyuk)

        if s % 2 == 0:
            entry, exit_ = left, right
            out_x = right[0] + R          # sagdan cik, saga tasarak don
        else:
            entry, exit_ = right, left
            out_x = left[0] - R           # soldan cik, sola tasarak don

        wps.append(entry)
        wps.append(exit_)

        # son serit degilse: alanin disinda bir donus tepe noktasi
        if s < NUM_STRIPS - 1:
            v_next = (s + 1) / (NUM_STRIPS - 1)
            z_mid = (exit_[1] + bilinear(A, B, C, D, 0.0, v_next)[1]) / 2.0
            wps.append((out_x, z_mid))     # disari tasan U donus noktasi
    return wps

def compute_yaw(dir_x, dir_z, yaw_sign, yaw_offset):
    raw = math.degrees(math.atan2(dir_x, dir_z))
    return wrap_deg(yaw_sign * raw + yaw_offset)

def send_pose(sock, wx, wz, height, yaw, t, seq, dt):
    data = packet_format.pack(
        float(wx - START_X),       # px -> Unity X (right)
        float(-(wz - START_Z)),    # py -> fallback'te -py = Unity Z (forward)
        float(height - START_Y),   # pz -> Unity Y (up)
        0.0, 0.0, float(yaw),
        float(t), float(seq), float(dt))
    sock.sendto(data, (UDP_IP, UDP_PORT))

def send_emergency(reason_text, final_d):
    """Unity'ye emergency tetigi gonder (5012). UDP kaybina karsi 5 kez."""
    try:
        emer_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for _ in range(5):
            emer_sock.sendto(b"EMERGENCY", (EMERGENCY_IP, EMERGENCY_PORT))
        emer_sock.close()
        print(f"\n>>> SIZINTIYA ULASILDI ({reason_text}, merkeze {final_d:.1f} birim) "
              f"-- DURULUYOR + EMERGENCY GONDERILDI (5012).\n")
    except Exception as e:
        print(f"[UYARI] Emergency gonderilemedi: {e}")

# --------------------------- main ---------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["mission", "yaw-test", "move-test", "drive-test"], default="mission")
    ap.add_argument("--yaw-sign", type=float, default=1.0)
    ap.add_argument("--yaw-offset", type=float, default=0.0)
    args = ap.parse_args()

    if args.mode == "drive-test":
        waypoints = [CORNER_A, CORNER_B, CORNER_C, CORNER_D]
    elif args.mode == "yaw-test":
        waypoints = [(START_X + 50.0, START_Z)]
    elif args.mode == "move-test":
        waypoints = [CORNER_A]
    else:
        waypoints = build_scan_waypoints()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    wx, wz = START_X, START_Z
    height = START_Y
    vx, vz = 0.0, 0.0
    wp_idx = 0
    mission_done = False
    emergency_sent = False

    # ---- Spiral durumu ----
    spiral_active = False
    spiral_theta = 0.0          # merkez etrafindaki aci (rad)
    spiral_r = 0.0              # merkeze olan anlik yaricap

    prev_wp = (START_X, START_Z)

    is_waiting = False
    wait_start_time = 0.0
    WAIT_DURATION = 15.0

    kinematic_yaw_rad = math.pi   # -Z yonu (yaw=180)

    print("=" * 70)
    print(f"MOD: {args.mode}    HEDEF UDP: {UDP_IP}:{UDP_PORT}")
    print(f"SPIRAL: R_TRIGGER={R_TRIGGER} R_STOP={R_STOP} TURNS={SPIRAL_TURNS} "
          f"yaw_rate(tarama)={MAX_YAW_RATE_RAD:.3f} yaw_rate(spiral)={SPIRAL_MAX_YAW_RATE:.3f}")
    print("=" * 70)

    start_time = time.perf_counter()
    last_time = start_time
    seq = 0

    try:
        hold_until = time.perf_counter() + STARTUP_HOLD_SEC
        while time.perf_counter() < hold_until:
            dir_x = math.sin(kinematic_yaw_rad)
            dir_z = math.cos(kinematic_yaw_rad)
            visual_yaw = compute_yaw(dir_x, dir_z, args.yaw_sign, args.yaw_offset)
            send_pose(sock, wx, wz, height, visual_yaw, time.perf_counter() - start_time, seq, DT)
            seq += 1
            time.sleep(DT)

        while True:
            loop_start = time.perf_counter()
            now = loop_start
            dt = clamp(now - last_time, 0.0, 0.05)
            last_time = now

            if mission_done:
                target_vx = target_vz = 0.0

            elif spiral_active:
                # ---- SPIRAL MODU: sizinti etrafinda daralarak merkeze ----
                dr_per_rad = (R_TRIGGER - R_STOP) / (SPIRAL_TURNS * 2.0 * math.pi)

                eff_r = max(spiral_r, 1.0)
                dtheta = (CRUISE_SPEED / eff_r) * dt
                spiral_theta = wrap_rad(spiral_theta + dtheta)
                spiral_r = max(R_STOP, spiral_r - dr_per_rad * dtheta)

                carrot_x = LEAK_X + spiral_r * math.sin(spiral_theta)
                carrot_z = LEAK_Z + spiral_r * math.cos(spiral_theta)

                dx = carrot_x - wx
                dz = carrot_z - wz
                dist_carrot = math.hypot(dx, dz)

                if dist_carrot > 0.001:
                    target_heading_rad = math.atan2(dx, dz)
                    err_rad = wrap_rad(target_heading_rad - kinematic_yaw_rad)
                    # SPIRAL'E OZEL yuksek yaw rate (keskin donus icin)
                    step_rad = clamp(err_rad, -SPIRAL_MAX_YAW_RATE * dt, SPIRAL_MAX_YAW_RATE * dt)
                    kinematic_yaw_rad = wrap_rad(kinematic_yaw_rad + step_rad)

                target_vx = math.sin(kinematic_yaw_rad) * CRUISE_SPEED
                target_vz = math.cos(kinematic_yaw_rad) * CRUISE_SPEED

                # Merkeze ulasildi mi? (spiral yaricapi R_STOP'a indi VEYA
                # gercek mesafe yeterince kucuk -> emergency'yi GARANTIYE al)
                real_d = math.hypot(LEAK_X - wx, LEAK_Z - wz)
                if (spiral_r <= R_STOP + 0.05) or (real_d <= EMERGENCY_DIST_FALLBACK):
                    mission_done = True
                    if not emergency_sent:
                        emergency_sent = True
                        reason = "spiral tamam" if spiral_r <= R_STOP + 0.05 else "mesafe-yedek"
                        send_emergency(reason, real_d)

            else:
                if is_waiting:
                    target_vx = target_vz = 0.0
                    time_left = WAIT_DURATION - (now - wait_start_time)

                    if time_left <= 0:
                        is_waiting = False
                        prev_wp = waypoints[wp_idx]
                        wp_idx += 1
                        if wp_idx >= len(waypoints):
                            mission_done = True
                            print("\n>>> TEST BITTI. TUM NOKTALAR DOLASILDI.\n")
                        else:
                            print(f"\n>>> BEKLEME BITTI. HEDEF: WP {wp_idx+1}\n")

                    elif seq % int(SEND_HZ) == 0:
                        print(f"*** BEKLEMEDE... Kalan Sure: {time_left:.1f} sn | Unity Pozisyonunu Not Al! ***")

                else:
                    if args.mode == "mission" and not spiral_active \
                            and math.hypot(LEAK_X - wx, LEAK_Z - wz) < R_TRIGGER:
                        spiral_active = True
                        spiral_r = math.hypot(wx - LEAK_X, wz - LEAK_Z)
                        spiral_theta = math.atan2(wx - LEAK_X, wz - LEAK_Z)
                        print(f"\n>>> SIZINTI MENZILINDE (r={spiral_r:.1f}) "
                              f"-- SPIRAL BASLIYOR, merkeze yaklasiliyor.\n")

                    target = waypoints[wp_idx]
                    path_x = target[0] - prev_wp[0]
                    path_z = target[1] - prev_wp[1]
                    path_len = math.hypot(path_x, path_z)

                    reached = False
                    progress = 0.0
                    u_x, u_z = 0.0, 0.0

                    if path_len > 0.01:
                        u_x = path_x / path_len
                        u_z = path_z / path_len
                        v_x = wx - prev_wp[0]
                        v_z = wz - prev_wp[1]
                        progress = (v_x * u_x) + (v_z * u_z)

                    dist_to_target = math.hypot(target[0] - wx, target[1] - wz)

                    if args.mode == "drive-test":
                        if dist_to_target < WAYPOINT_TOL:
                            reached = True
                    else:
                        if path_len > 0.01:
                            if progress >= (path_len - WAYPOINT_TOL):
                                reached = True
                        else:
                            if dist_to_target < WAYPOINT_TOL:
                                reached = True

                    if reached:
                        if args.mode == "drive-test":
                            is_waiting = True
                            wait_start_time = now
                            point_name = ["CORNER_A", "CORNER_B", "CORNER_C", "CORNER_D"][wp_idx]
                            print("\n" + "="*60)
                            print(f"!!! ARAC {point_name} NOKTASINA ULASTI VE DURDU !!!")
                            print("LUTFEN SU ANKI UNITY KOORDINATLARINI NOT AL.")
                            print("ARAC 15 SANIYE SONRA DIGER NOKTAYA HAREKET EDECEK.")
                            print("="*60 + "\n")
                        else:
                            if wp_idx < len(waypoints) - 1:
                                prev_wp = waypoints[wp_idx]
                                wp_idx += 1
                                print(f"\n>>> WP {wp_idx} GECILDI. YENI HEDEF: WP {wp_idx+1}\n")
                            else:
                                mission_done = True
                                print("\n>>> TUM SERITLER TARANDI -- DURULUYOR.\n")
                        target_vx = target_vz = 0.0
                    else:
                        if path_len > 0.01:
                            carrot_prog = progress + LOOKAHEAD_DIST
                            carrot_prog = clamp(carrot_prog, 0.0, path_len)
                            carrot_x = prev_wp[0] + u_x * carrot_prog
                            carrot_z = prev_wp[1] + u_z * carrot_prog
                        else:
                            carrot_x = target[0]
                            carrot_z = target[1]

                        dx = carrot_x - wx
                        dz = carrot_z - wz
                        dist_carrot = math.hypot(dx, dz)

                        if dist_carrot > 0.001:
                            target_heading_rad = math.atan2(dx, dz)
                            err_rad = wrap_rad(target_heading_rad - kinematic_yaw_rad)
                            step_rad = clamp(err_rad, -MAX_YAW_RATE_RAD * dt, MAX_YAW_RATE_RAD * dt)
                            kinematic_yaw_rad = wrap_rad(kinematic_yaw_rad + step_rad)

                        target_vx = math.sin(kinematic_yaw_rad) * CRUISE_SPEED
                        target_vz = math.cos(kinematic_yaw_rad) * CRUISE_SPEED

            a = clamp(dt / VEL_SMOOTH_TAU, 0.0, 1.0)
            vx += (target_vx - vx) * a
            vz += (target_vz - vz) * a

            step_x = clamp(vx * dt, -MAX_POS_STEP_PER_PACKET, MAX_POS_STEP_PER_PACKET)
            step_z = clamp(vz * dt, -MAX_POS_STEP_PER_PACKET, MAX_POS_STEP_PER_PACKET)
            wx += step_x
            wz += step_z

            dir_x = math.sin(kinematic_yaw_rad)
            dir_z = math.cos(kinematic_yaw_rad)
            visual_yaw = compute_yaw(dir_x, dir_z, args.yaw_sign, args.yaw_offset)

            send_pose(sock, wx, wz, height, visual_yaw, now - start_time, seq, dt)

            if seq % int(SEND_HZ) == 0 and not is_waiting:
                if spiral_active and not mission_done:
                    state = f"SPIRAL r={spiral_r:.1f}"
                elif mission_done:
                    state = "DONE"
                else:
                    state = f"wp {wp_idx + 1}/{len(waypoints)}"
                print(f"seq={seq:05d} {state:14s} Pos=({wx:+.3f},{wz:+.3f}) vis_yaw={visual_yaw:+6.1f}")

            seq += 1
            sl = DT - (time.perf_counter() - loop_start)
            if sl > 0:
                time.sleep(sl)

    except KeyboardInterrupt:
        print("\n[BILGI] Kullanici tarafindan durduruldu.")
    finally:
        sock.close()

if __name__ == "__main__":
    main()