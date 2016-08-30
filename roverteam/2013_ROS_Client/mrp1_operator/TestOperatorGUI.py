import socket
import time
'''
1*.10.* = Voltage
  .11.* = Current
  .12.* = Velocity
50.1.*  = IMU X Vector
50.2.*  = IMU Y Vector
50.3.*  = IMU Z Vector
50.4.*  = IMU Yaw
50.5.*  = IMU Pitch
50.6.*  = IMU Roll
51.1.*  = Compass Heading(degree)
52.1.*  = GPS Reading
53.1.*  = Rover Tilt X
53.2.*  = Rover Tilt Y
70.1.*  = OBC Signal Strength Reading
90.1    = Operator GUI Quit Signal
90.2.*  = Timer time(seconds)
'''
Base_Timer = 3600
Rover_Telemetry_IMU = [0,0,0, 0,0,0]
Rover_Telemetry_Compass = 0
Rover_Telemetry_TiltX = 0.0 # left-/right+ tilt
Rover_Telemetry_TiltY = 0.0 # back-/front+ tilt
#Rover_Telemetry_Voltages = [0,0,0,0,0]
#Rover_Telemetry_Currents = [0,0,0,0,0]
#Rover_Telemetry_Velocities = [0,0,0,0]
Rover_Wheel_Telemetry = []
for i in range(4):
	Rover_Wheel_Telemetry.append([0.0,0.0,0.0])
Rover_Chassis_Telemetry = [0,0]
Rover_Telemetry_GPS = ""
Rover_Telemetry_Signal_Strength = [0,0]

UDP_IP = "127.0.0.1"
UDP_PORT = 15005
#MESSAGE = '14.10.0.100'
MESSAGE = '20.110'

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UDP
print 'start test:'
print 'Signal_Base_Timer'
print 'Signal_Update_Telemetry_IMU'
print 'Signal_Update_Telemetry_Compass'
print 'Signal_Update_Telemetry_Voltages'
print 'Signal_Update_Telemetry_Currents'
print 'Signal_Update_Telemetry_Velocities'
print 'Signal_Update_Telemetry_GPS'
print 'Signal_Update_Telemetry_Signal_Strength'
while True:
	# Wheels
	for i in range(len(Rover_Wheel_Telemetry)):
		for j in range(3):
			if Rover_Wheel_Telemetry[i][j] > 12.0:
				Rover_Wheel_Telemetry[i][j] = -12.0
			else:
				Rover_Wheel_Telemetry[i][j] += 0.1
			sock.sendto(str(i+10)+'_'+str(j+1)+'_'+str(Rover_Wheel_Telemetry[i][j]), (UDP_IP, UDP_PORT))
	# Chassis
	for i in range(len(Rover_Chassis_Telemetry)):
		if Rover_Chassis_Telemetry[i] > 12.0:
			Rover_Chassis_Telemetry[i] = -12.0
		else:
			Rover_Chassis_Telemetry[i] += 0.1
		sock.sendto('14_'+str(i+1)+'_'+str(Rover_Chassis_Telemetry[i]), (UDP_IP, UDP_PORT))
	# IMU
	for i in range(len(Rover_Telemetry_IMU)):
		if Rover_Telemetry_IMU[i] > 127:
			Rover_Telemetry_IMU[i] = -127
		else:
			Rover_Telemetry_IMU[i] += 1
		sock.sendto('50_'+str(i+1)+'_'+str(Rover_Telemetry_IMU[i]), (UDP_IP, UDP_PORT))
	# Compass
	if Rover_Telemetry_Compass > 360:
		Rover_Telemetry_Compass = 0
	else:
		Rover_Telemetry_Compass += 1
	sock.sendto('51_1_'+str(Rover_Telemetry_Compass), (UDP_IP, UDP_PORT))
	# GPS
	Rover_Telemetry_GPS = 'GPS Test'
	sock.sendto('52_1_'+Rover_Telemetry_GPS, (UDP_IP, UDP_PORT))
	# Rover Tilt X/Y
	if Rover_Telemetry_TiltX > 90:
		Rover_Telemetry_TiltX = -90
		Rover_Telemetry_TiltY = -90
	else:
		Rover_Telemetry_TiltX += 1
		Rover_Telemetry_TiltY += 1
	sock.sendto('53_1_'+str(Rover_Telemetry_TiltX), (UDP_IP, UDP_PORT))
	sock.sendto('53_2_'+str(Rover_Telemetry_TiltY), (UDP_IP, UDP_PORT))
	# Signal Strnegth
	if Rover_Telemetry_Signal_Strength[0] > 70:
		Rover_Telemetry_Signal_Strength[0] = 0
		Rover_Telemetry_Signal_Strength[1] = 0
	else:
		Rover_Telemetry_Signal_Strength[0] += 1
		Rover_Telemetry_Signal_Strength[1] += 2
	sock.sendto('70_1_'+str(Rover_Telemetry_Signal_Strength[0])+'/'+str(Rover_Telemetry_Signal_Strength[1]), (UDP_IP, UDP_PORT))
	# Timer
	if Base_Timer < 0:
		Base_Timer = 3600
	else:
		Base_Timer -= 1
	sock.sendto('90_2_'+str(Base_Timer), (UDP_IP, UDP_PORT))
	#sock.sendto(, (UDP_IP, UDP_PORT))
	time.sleep(1)
	print 'loop',Base_Timer
