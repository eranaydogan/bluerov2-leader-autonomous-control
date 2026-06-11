import socket
import struct
import time
import threading
import math
from pynput import keyboard

# ================= UDP AYARLARI =================
UDP_IP = "127.0.0.1"
UDP_PORT = 5007

SEND_HZ = 60.0
DT = 1.0 / SEND_HZ

# ================= HAREKET AYARLARI =================
# Unity sahnesi hassas olduğu için çok küçük başlıyoruz.
# Çok yavaş kalırsa BASE_MOVE_SPEED'i 0.05 -> 0.10 yap.
BASE_MOVE_SPEED = 0.10
FAST_MOVE_SPEED = 0.40
FINE_MOVE_SPEED = 0.06

BASE_VERTICAL_SPEED = 0.04
FAST_VERTICAL_SPEED = 0.12
FINE_VERTICAL_SPEED = 0.01

BASE_ROT_SPEED = 20.0         # degree / second
FAST_ROT_SPEED = 15.0
FINE_ROT_SPEED = 1.5

# Hızlanma/yavaşlama yumuşaklığı.
# Büyük değer = daha hızlı tepki.
# Küçük değer = daha yumuşak hareket.
MOVE_RESPONSE = 3.0
ROT_RESPONSE = 4.0

# Ani büyük sıçramayı engelleyen paket başı limitler.
MAX_POS_STEP_PER_PACKET = 0.004      # Unity unit / packet
MAX_ROT_STEP_PER_PACKET = 0.50      # degree / packet

# Başlangıç değerleri: Unity receiver relative moddaysa 0'dan başlamak doğru.
x, y, z = 0.0, 0.0, 0.0
roll, pitch, yaw = 0.0, 0.0, 0.0

# Mevcut hızlar
vx, vy, vz = 0.0, 0.0, 0.0
vroll, vpitch, vyaw = 0.0, 0.0, 0.0
# =====================================================

pressed_keys = set()
lock = threading.Lock()


def on_press(key):
    with lock:
        try:
            pressed_keys.add(key.char.lower())
        except AttributeError:
            pressed_keys.add(key)


def on_release(key):
    with lock:
        try:
            pressed_keys.discard(key.char.lower())
        except AttributeError:
            pressed_keys.discard(key)


def is_pressed(k):
    with lock:
        return k in pressed_keys


def get_axis(positive_key, negative_key):
    return int(is_pressed(positive_key)) - int(is_pressed(negative_key))


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def smooth_towards(current, target, response, dt):
    # FPS bağımsız exponential smoothing
    alpha = 1.0 - math.exp(-response * dt)
    return current + (target - current) * alpha


def wrap_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def speed_mode():
    # Ctrl = hassas/yavaş
    if is_pressed(keyboard.Key.ctrl_l) or is_pressed(keyboard.Key.ctrl_r):
        return "FINE"

    # Shift = hızlı
    if is_pressed(keyboard.Key.shift) or is_pressed(keyboard.Key.shift_l) or is_pressed(keyboard.Key.shift_r):
        return "FAST"

    return "BASE"


def get_speeds():
    mode = speed_mode()

    if mode == "FINE":
        return FINE_MOVE_SPEED, FINE_VERTICAL_SPEED, FINE_ROT_SPEED, mode

    if mode == "FAST":
        return FAST_MOVE_SPEED, FAST_VERTICAL_SPEED, FAST_ROT_SPEED, mode

    return BASE_MOVE_SPEED, BASE_VERTICAL_SPEED, BASE_ROT_SPEED, mode


listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.daemon = True
listener.start()

print("=" * 70)
print(f"HEDEF: {UDP_IP}:{UDP_PORT}")
print("")
print("HAREKET KONTROLLERİ:")
print("  W / S        : X ileri / geri")
print("  D / A        : Y sağ / sol")
print("  E / C        : Z yukarı / aşağı")
print("")
print("YÖNELİM KONTROLLERİ:")
print("  Up / Down    : Pitch")
print("  Right / Left : Yaw")
print("  R / Q        : Roll")
print("")
print("HIZ MODLARI:")
print("  Normal       : çok yavaş")
print("  Shift        : biraz hızlı")
print("  Ctrl         : çok hassas/fine")
print("")
print("DİĞER:")
print("  X            : pose reset")
print("  Ctrl+C       : çıkış")
print("=" * 70)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
packet_format = struct.Struct("<9f")

start_time = time.perf_counter()
last_time = start_time
seq = 0

try:
    while True:
        loop_start = time.perf_counter()
        now = loop_start

        real_dt = now - last_time
        last_time = now

        # Çok büyük dt oluşursa sıçrama olmasın.
        real_dt = clamp(real_dt, 0.0, 0.05)

        move_speed, vertical_speed, rot_speed, mode = get_speeds()

        # Reset
        if is_pressed("x"):
            x, y, z = 0.0, 0.0, 0.0
            roll, pitch, yaw = 0.0, 0.0, 0.0
            vx, vy, vz = 0.0, 0.0, 0.0
            vroll, vpitch, vyaw = 0.0, 0.0, 0.0

        # ================= INPUT AXES =================
        # Eksen düzeltmesi: Unity alıcısının koordinat dönüşümünden sonra
        # W/S -> ileri/geri, D/A -> sağ/sol doğru çıksın diye
        # forward ve lateral kanalları burada eşleştiriyoruz.
        forward_input = get_axis("w", "s")   # W ileri, S geri (kullanıcı niyeti)
        lateral_input = get_axis("a", "d")   # D sağ, A sol (kullanıcı niyeti)

        ax = lateral_input    # Unity'de forward gibi davranan kanal -> lateral input ver
        ay = forward_input    # Unity'de lateral gibi davranan kanal -> forward input ver
        az = get_axis("e", "c")

        apitch = get_axis(keyboard.Key.up, keyboard.Key.down)
        ayaw = get_axis(keyboard.Key.right, keyboard.Key.left)
        aroll = get_axis("r", "q")

        # ================= TARGET VELOCITIES =================
        target_vx = ax * move_speed
        target_vy = ay * move_speed
        target_vz = az * vertical_speed

        target_vpitch = apitch * rot_speed
        target_vyaw = ayaw * rot_speed
        target_vroll = aroll * rot_speed

        # ================= SMOOTH VELOCITIES =================
        vx = smooth_towards(vx, target_vx, MOVE_RESPONSE, real_dt)
        vy = smooth_towards(vy, target_vy, MOVE_RESPONSE, real_dt)
        vz = smooth_towards(vz, target_vz, MOVE_RESPONSE, real_dt)

        vpitch = smooth_towards(vpitch, target_vpitch, ROT_RESPONSE, real_dt)
        vyaw = smooth_towards(vyaw, target_vyaw, ROT_RESPONSE, real_dt)
        vroll = smooth_towards(vroll, target_vroll, ROT_RESPONSE, real_dt)

        # ================= STEP LIMIT =================
        step_x = clamp(vx * real_dt, -MAX_POS_STEP_PER_PACKET, MAX_POS_STEP_PER_PACKET)
        step_y = clamp(vy * real_dt, -MAX_POS_STEP_PER_PACKET, MAX_POS_STEP_PER_PACKET)
        step_z = clamp(vz * real_dt, -MAX_POS_STEP_PER_PACKET, MAX_POS_STEP_PER_PACKET)

        step_pitch = clamp(vpitch * real_dt, -MAX_ROT_STEP_PER_PACKET, MAX_ROT_STEP_PER_PACKET)
        step_yaw = clamp(vyaw * real_dt, -MAX_ROT_STEP_PER_PACKET, MAX_ROT_STEP_PER_PACKET)
        step_roll = clamp(vroll * real_dt, -MAX_ROT_STEP_PER_PACKET, MAX_ROT_STEP_PER_PACKET)

        # ================= INTEGRATE =================
        x += step_x
        y += step_y
        z += step_z

        pitch = wrap_angle_deg(pitch + step_pitch)
        yaw = wrap_angle_deg(yaw + step_yaw)
        roll = wrap_angle_deg(roll + step_roll)

        current_time = time.perf_counter() - start_time

        data = packet_format.pack(
            float(x), float(y), float(z),
            float(roll), float(pitch), float(yaw),
            float(current_time),
            float(seq),
            float(real_dt),
        )

        sock.sendto(data, (UDP_IP, UDP_PORT))

        if seq % int(SEND_HZ) == 0:
            print(
                f"seq={seq:05d} mode={mode:4s} "
                f"pos=({x:+.4f}, {y:+.4f}, {z:+.4f}) "
                f"rot=({roll:+.2f}, {pitch:+.2f}, {yaw:+.2f}) "
                f"vel=({vx:+.4f}, {vy:+.4f}, {vz:+.4f})"
            )

        seq += 1

        elapsed = time.perf_counter() - loop_start
        sleep_time = DT - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print("\n[BİLGİ] Kullanıcı tarafından durduruldu.")

except Exception as e:
    print(f"\n[HATA] Beklenmeyen bir sorun oluştu: {e}")

finally:
    print("Soket kapatılıyor...")
    sock.close()
    listener.stop()