import socket
import sys

s = socket.socket()
s.settimeout(10.0)
try:
    s.connect(('peach-stem.local', 6666))
    s.settimeout(2.0)
    while True:
        data = s.recv(1024)
        if not data: break
        sys.stdout.write(data.decode('utf-8', 'ignore'))
        sys.stdout.flush()
except socket.timeout:
    pass
except Exception as e:
    print(e)
