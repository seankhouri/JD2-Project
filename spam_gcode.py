import serial
import time

PORT = "/dev/tty.usbserial-0001"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

for i in range(100):
    cmd = f"G1 X{i}\n"
    ser.write(cmd.encode("ascii"))
    time.sleep(0.01)
    response = ser.readline().decode(errors="ignore").strip()
    if response:
        print(response)

ser.close()