"""
UART loopback test — TX shorted to RX on GPIO 14/15.
Sends bytes and checks if they come back. Confirms RPi UART is working.
"""
import serial
import time

PORT     = "/dev/serial0"
BAUD     = 57600
MESSAGE  = b"AeroDesign UART loopback test 1234"

print(f"Opening {PORT} @ {BAUD} baud...")
ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(0.1)

# Flush anything sitting in the buffer
ser.reset_input_buffer()

print(f"Sending {len(MESSAGE)} bytes: {MESSAGE}")
ser.write(MESSAGE)
time.sleep(0.2)  # Give bytes time to travel TX -> RX

waiting = ser.in_waiting
print(f"Bytes waiting to be read: {waiting}")

received = ser.read(len(MESSAGE))
ser.close()

print(f"Received: {received}")

if received == MESSAGE:
    print("\nLOOPBACK OK — TX and RX are wired correctly and UART is working.")
elif len(received) == 0:
    print("\nNOTHING RECEIVED — TX and RX are NOT shorted, or UART is not working.")
else:
    print(f"\nPARTIAL / GARBLED — got {len(received)} of {len(MESSAGE)} bytes.")
    print("Possible baud rate mismatch or loose connection.")
