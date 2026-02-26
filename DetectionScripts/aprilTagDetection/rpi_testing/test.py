"""
UART diagnostic script — run on RPi to check serial port access and FC comms.
"""
import sys
import time

PORT = "/dev/serial0"
BAUD_RATES = [57600, 115200, 921600]

# ------------------------------------------------------------------
# Step 1: Check port permissions
# ------------------------------------------------------------------
print("=== Step 1: Port permissions ===")
import os
import stat

try:
    info = os.stat(PORT)
    print(f"  {PORT} exists")
    print(f"  Owner: uid={info.st_uid}  gid={info.st_gid}")
    print(f"  Mode:  {oct(stat.S_IMODE(info.st_mode))}")
    print(f"  Current user uid: {os.getuid()}  groups: {os.getgroups()}")
except FileNotFoundError:
    print(f"  ERROR: {PORT} does not exist — UART not enabled")
    sys.exit(1)

# ------------------------------------------------------------------
# Step 2: Try opening the port at each baud rate
# ------------------------------------------------------------------
print("\n=== Step 2: Opening port ===")
import serial

opened_port = None
for baud in BAUD_RATES:
    try:
        ser = serial.Serial(PORT, baud, timeout=2)
        print(f"  OK — opened at {baud} baud")
        opened_port = ser
        break
    except PermissionError:
        print(f"  FAIL at {baud} — Permission denied")
        print("  Fix: sudo usermod -a -G dialout $USER  then log out and back in")
        sys.exit(1)
    except Exception as e:
        print(f"  FAIL at {baud} — {e}")

if opened_port is None:
    print("  Could not open port at any baud rate")
    sys.exit(1)

# ------------------------------------------------------------------
# Step 3: Listen for any incoming bytes (FC heartbeat / MAVLink traffic)
# ------------------------------------------------------------------
print(f"\n=== Step 3: Listening for data on {PORT} @ {opened_port.baudrate} baud (5 seconds) ===")
print("  If FC is connected and sending MAVLink, you should see raw bytes below.")
print("  MAVLink packets start with 0xFD (253) for v2 or 0xFE (254) for v1.\n")

deadline = time.time() + 5.0
received = bytearray()

while time.time() < deadline:
    waiting = opened_port.in_waiting
    if waiting:
        chunk = opened_port.read(waiting)
        received.extend(chunk)
        print(f"  Received {len(chunk)} bytes: {list(chunk[:16])}{'...' if len(chunk) > 16 else ''}")
    time.sleep(0.1)

if not received:
    print("  No data received — check:")
    print("    1. FC is powered on")
    print("    2. TX/RX wires not swapped (RPi TX -> FC RX, RPi RX -> FC TX)")
    print("    3. FC UART configured for MAVLink (SERIALx_PROTOCOL=2)")
    print(f"   4. FC baud rate matches {opened_port.baudrate} (SERIALx_BAUD)")
else:
    print(f"\n  Total: {len(received)} bytes received")
    has_mavlink_v2 = 0xFD in received
    has_mavlink_v1 = 0xFE in received
    if has_mavlink_v2:
        print("  MAVLink v2 marker (0xFD) detected — FC is talking MAVLink!")
    if has_mavlink_v1:
        print("  MAVLink v1 marker (0xFE) detected — FC is talking MAVLink!")
    if not has_mavlink_v2 and not has_mavlink_v1:
        print("  Data received but no MAVLink markers found — baud rate may be wrong")

opened_port.close()
print("\nDone.")
