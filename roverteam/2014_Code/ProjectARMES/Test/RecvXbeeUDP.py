# netstat -lnpu
import serial
import time
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 15005
MESSAGE = "Hello, World!"

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
ser = serial.Serial('/dev/ttyUSB1',115200, xonxoff=True, rtscts=True, timeout=0.1)

while True:
	try:
		data = ser.read(100)
		print 'data',data
		if len(data) > 0:
			sock.sendto(str(data), (UDP_IP, UDP_PORT))
	except KeyboardInterrupt:
		break
ser.close()
sock.close()