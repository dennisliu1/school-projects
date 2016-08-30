# netstat -lnpu
import serial
import time
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

ser = serial.Serial('/dev/ttyUSB0',115200, xonxoff=True, rtscts=True, timeout=0.1)
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
	try:
	    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
	    print "received message:", data
	    ser.write(data)
	except KeyboardInterrupt:
		break
ser.close()
sock.close()