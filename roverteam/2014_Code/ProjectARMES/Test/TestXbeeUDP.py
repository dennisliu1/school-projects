import serial
from xbee import XBee

serial_port = serial.Serial('/dev/ttyUSB1', 9600)
xbee = XBee(serial_port)
while True:
	try:
		print xbee.wait_read_frame()
	except KeyboardInterrupt:
		break
serial_port.close()