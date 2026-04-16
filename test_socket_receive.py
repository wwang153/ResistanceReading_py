import socket
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024)
    t_arrival = time.time() # Local Ubuntu time
    
    parts = data.decode().split(',')
    t_mac = float(parts[0])
    lengths = [float(x) for x in parts[1:]]
    
    latency = t_arrival - t_mac
    print(f"Latency: {latency:.4f}s | Data: {lengths}")