import serial

ser = serial.Serial("/dev/serial0", 57600)

ser.write(b"hello")
print(ser.read(5))