import socket
import subprocess
import platform

Import("env")

def is_pingable(host):
    system = platform.system().lower()
    if system == 'windows':
        command = ['ping', '-n', '1', '-w', '1000', host]
    else:
        command = ['ping', '-c', '1', '-t', '1', host]
    
    return subprocess.call(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0

print("\033[96m[AUTO-UPLOAD] Checking if OTA device is reachable...\033[0m")

host = "peach-pit-esp32.local"

try:
    # First try to resolve the mDNS name
    ip = socket.gethostbyname(host)
    
    # Then verify it's actually alive (prevent timeout if it's dead but cached)
    if is_pingable(ip):
        print(f"\033[92m[AUTO-UPLOAD] Device reachable at {ip}. Using ESPOTA.\033[0m")
        env.Replace(UPLOAD_PROTOCOL="espota")
        env.Replace(UPLOAD_PORT=host)
    else:
        raise Exception("Host down")
except Exception:
    print("\033[93m[AUTO-UPLOAD] Device not reachable on WiFi. Falling back to USB.\033[0m")
    env.Replace(UPLOAD_PROTOCOL="esptool")
    env.Replace(UPLOAD_PORT="/dev/cu.usbserial-0001")
    env.Replace(UPLOAD_SPEED="115200")
