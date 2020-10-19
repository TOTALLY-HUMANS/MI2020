import socket
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(bytes("100;-100", "utf-8") , ("127.0.0.1", 3001))
time.sleep(2.0)
sock.sendto(bytes("-100;100", "utf-8") , ("127.0.0.1", 3001))
time.sleep(2.0)
sock.sendto(bytes("0;0", "utf-8") , ("127.0.0.1", 3001))

sock.sendto(bytes("100;-100", "utf-8") , ("127.0.0.1", 3002))
time.sleep(2.0)
sock.sendto(bytes("-100;100", "utf-8") , ("127.0.0.1", 3002))
time.sleep(2.0)
sock.sendto(bytes("0;0", "utf-8") , ("127.0.0.1", 3002))

#reset simulation
sock.sendto(bytes("reset", "utf-8"), ("127.0.0.1", 3000))
