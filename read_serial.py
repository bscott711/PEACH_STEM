import serial
import sys

try:
    s = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=1)
except Exception as e:
    print("Could not open serial port:", e)
    sys.exit(1)

print("Reading from serial...")
for i in range(30):
    line = s.readline()
    if line:
        try:
            print(line.decode('utf-8', errors='replace').strip())
        except Exception as e:
            print("Err decoding:", e)
    else:
        print("Timeout or no data")
