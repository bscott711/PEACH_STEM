import serial
import time
import sys

try:
    s = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=0.1)
    
    # Pulse RTS to reset the ESP32 (EN pin)
    # On typical ESP32 dev boards, DTR controls GPIO0 and RTS controls EN.
    # Actually, the auto-reset circuit uses both. 
    # To reset into RUN mode: DTR=1, RTS=0, then DTR=0, RTS=0.
    s.setDTR(False)
    s.setRTS(True)
    time.sleep(0.1)
    s.setRTS(False)
    time.sleep(0.1)
    
except Exception as e:
    print("Could not open serial port:", e)
    sys.exit(1)

print("Rebooting and reading from serial...")
lines = 0
while lines < 100:
    line = s.readline()
    if line:
        print(line.decode('utf-8', errors='replace').strip())
        lines += 1
