import socket

UDP_IP = "192.168.1.103"
UDP_PORT = 5055 #Client
#UDP_PORT = 10000 #Server
#UDP_PORT = 5006 #Rover
MESSAGE = "4979.0,38.37369,110.70870,1309.3"

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
# sock.sendto(MESSAGE, (UDP_IP, 5006))