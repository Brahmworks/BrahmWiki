# quick_udp_echo_test.py
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2)
try:
    s.sendto(b'hello', ('169.254.1.200', 3333))
    data, addr = s.recvfrom(4096)
    print('RX', data, 'from', addr)
except socket.timeout:
    print('No response (timeout)')
finally:
    s.close()