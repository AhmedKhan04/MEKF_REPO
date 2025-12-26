import serial
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from matplotlib.animation import FuncAnimation

# ---------------- CONFIG ----------------
PORT = "COM3"
BAUD = 115200

# Arduino packet: header(2) + t_ms(4) + seq(1) + 6 floats(24) + crc(2) = 33 bytes
# But with __attribute__((packed)), check actual size
PACKET_SIZE = 33  # Adjust based on Arduino's printed "Packet size"
PRINT_RATE_HZ = 200
WINDOW_SECONDS = 3
MAX_SAMPLES = 1000
# ---------------------------------------

def crc16_ccitt(data):
    """CRC-16 CCITT calculation matching Arduino"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc

def find_sync(ser):
    """Find packet header 0xABCD"""
    buffer = bytearray()
    while True:
        byte = ser.read(1)
        if not byte:
            continue
        buffer.append(byte[0])
        if len(buffer) >= 2:
            if buffer[-2] == 0xCD and buffer[-1] == 0xAB:  # Little-endian
                return
            buffer.pop(0)

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

print("Flushing startup text...")
ser.reset_input_buffer()
time.sleep(0.5)
ser.reset_input_buffer()

print("Syncing to packet header...")
find_sync(ser)
print("Synchronized!")

# Buffers
t_buf  = deque(maxlen=MAX_SAMPLES)
ax_buf = deque(maxlen=MAX_SAMPLES)
ay_buf = deque(maxlen=MAX_SAMPLES)
az_buf = deque(maxlen=MAX_SAMPLES)
gx_buf = deque(maxlen=MAX_SAMPLES)
gy_buf = deque(maxlen=MAX_SAMPLES)
gz_buf = deque(maxlen=MAX_SAMPLES)

t0 = None
last_print = 0.0
good_packets = 0
bad_packets = 0

# ---------- Plot setup ----------
fig, axs = plt.subplots(2, 3, figsize=(12, 8))
fig.suptitle("Live MPU6050 IMU Data")

labels = [
    "ax (m/s²)", "ay (m/s²)", "az (m/s²)",
    "gx (rad/s)", "gy (rad/s)", "gz (rad/s)"
]

buffers = [ax_buf, ay_buf, az_buf, gx_buf, gy_buf, gz_buf]
lines = []

for ax, label in zip(axs.flat, labels):
    line, = ax.plot([], [])
    ax.set_title(label)
    ax.grid(True)
    lines.append(line)

for ax in axs[1]:
    ax.set_xlabel("Time (s)")
other = False 
# ---------- Update loop ----------
def update(frame):
    global t0, last_print, good_packets, bad_packets

    # Read rest of packet (already read 2-byte header in sync)
    raw = ser.read(PACKET_SIZE - 2)
    if len(raw) != PACKET_SIZE - 2:
        print(len(raw))
        bad_packets += 1
        find_sync(ser)
        return

    # Parse packet
    try:
        t_ms = struct.unpack("<I", raw[0:4])[0]
        seq = raw[4]
        floats = struct.unpack("<6f", raw[5:29])
        crc_received = struct.unpack("<H", raw[29:31])[0]
    except struct.error:
        print(len(raw))
        bad_packets += 1
        find_sync(ser)
        return

    if bad_packets < 3:
        print(f"\n=== DEBUG PACKET ===")
        print(f"Raw (31 bytes): {raw.hex()}")
        print(f"With header (33 bytes): {(b'\\xCD\\xAB' + raw).hex()}")
        print(f"For CRC (31 bytes): {(b'\\xCD\\xAB' + raw[:-2]).hex()}")
        print(f"CRC bytes: {raw[-2:].hex()}")

    # Validate CRC
    packet_without_crc = bytes([0xCD, 0xAB])  + raw[:-2]
    crc_calc = crc16_ccitt(packet_without_crc)
    
    if crc_calc != crc_received:
        bad_packets += 1
        find_sync(ser)
        return

    good_packets += 1
    ax, ay, az, gx, gy, gz = floats

    if t0 is None:
        t0 = t_ms

    ts = (t_ms - t0) * 1e-3

    t_buf.append(ts)
    ax_buf.append(ax)
    ay_buf.append(ay)
    az_buf.append(az)
    gx_buf.append(gx)
    gy_buf.append(gy)
    gz_buf.append(gz)

    # Terminal print
    now = time.time()
    if now - last_print > 1.0 / PRINT_RATE_HZ:
        print(
            f"t={ts:7.3f} seq={seq:3d} | "
            f"ax={ax:+7.3f} ay={ay:+7.3f} az={az:+7.3f} | "
            f"gx={gx:+7.3f} gy={gy:+7.3f} gz={gz:+7.3f} | "
            f"Good={good_packets} Bad={bad_packets}"
        )
        last_print = now

    # Update plots
    if len(t_buf) > 1:
        t_arr = np.array(t_buf)
        for line, buf, ax in zip(lines, buffers, axs.flat):
            line.set_data(t_arr, buf)
            ax.set_xlim(max(0, t_arr[-1] - WINDOW_SECONDS), t_arr[-1])
            ax.relim()
            ax.autoscale_view(scalex=False)

ani = FuncAnimation(fig, update, interval=5,  cache_frame_data=False)

try:
    plt.show()
finally:
    ser.close()