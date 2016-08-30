#Rover Model Version 0.1
'''
Rover Model:
IMU:
	Euler Angle Rotations: [double x, double y, double z]
	Compass Heading: [double heading]
GPS: 
	Position: [double lati, double long]
ICCE Box:
	Temperature: [double temp]
	Voltage: [double voltage]
	current: [double current]
Drive Motors:
	ForwardLeft/Right, RearLeft/Right:[double voltage]
'''
from collections import namedtuple
import time

MSG_SEP = "_"
MSG_TELEMETRY = "T"
MSG_COMMAND = "C"
MSG_END = "E"

S_SEP = ':'
S_ICCE = 'ICCE'
S_IMU = 'IMU'
S_GPS = 'GPS'
S_COMPASS = 'COMPASS'
S_INTERNAL = 'INTERNAL'
S_DRIVE = 'DRIVE'
S_MOTOR = 'MOTOR'
S_VOLT = 'VOLT'

'''
C_1,0,1_100,1E

001_XXX
00_19,22,15

0_#,#,#,...
10_100,100,100,100
100_100
100_100,0,100,1
10_[speed,dir],[speed,dir],...
'''
N_SEP_VAL = ','
N_SEP = ''
N_ICCE  = 0
N_IMU       = 0
N_IMU_X           = 0
N_IMU_Y           = 1
N_IMU_Z           = 2
N_GPS       = 1
N_GPS_LATI        = 0
N_GPS_LONG        = 1
N_COMPASS   = 2
N_COMPASS_HEADING = 0
N_INTERNAL  = 3
N_INTERNAL_TEMP   = 0
N_INTERNAL_VOLT   = 1
N_INTERNAL_CURR   = 2
N_DRIVE = 1
N_MOTOR_VOLT= 0
N_MOTORFL_VOLT    = 0#[0-100]=power%,[0-1]=forward/reverse
N_MOTORFR_VOLT    = 1
N_MOTORRL_VOLT    = 2
N_MOTORRR_VOLT    = 3
N_SCIENCE=2
N_CORE      = 0
N_CORE_HEIGHT     = 0
N_DRILL     = 1
N_DRILL_SPIN      = 0
N_TRAP      = 2
N_TRAP_OC         = 0
N_SPECT     = 3
N_SPECT_IO        = 0
N_TRAP_OC_OPEN = 0
N_TRAP_OC_CLOSED = 1
N_SPECT_IO_IN  = 0
N_SPECT_IO_OUT  = 1
N_ARM    =3
N_ARM_MAIN  = 0
N_ARM_MAIN_X           = 0
N_ARM_MAIN_Y           = 1
N_ARM_MAIN_Z           = 2
# N_ARM_MAIN_BASE        = 3
# N_ARM_MAIN_ACT1        = 4
# N_ARM_MAIN_ACT2        = 5
N_ARM_END   = 1
N_ARM_END_X            = 0
N_ARM_END_Y            = 1
N_ARM_END_Z            = 2
N_ARM_END_GRIP         = 3
N_ARM_END_ROLL         = 4
# N_ARM_END_YAW          = 5
# N_ARM_END_PITCH        = 6

# N_MOTORFL   = 0
# N_MOTORFL_VOLT    = 0
# N_MOTORFR   = 1
# N_MOTORFR_VOLT    = 0
# N_MOTORRL   = 2
# N_MOTORRL_VOLT    = 0
# N_MOTORRR   = 3
# N_MOTORRR_VOLT    = 0

# ser = serial.Serial('/dev/ttyUSB0',115200,xonxoff=True)
# ser = serial.Serial('/dev/ttyUSB0',115200,xonxoff=True,rtscts=True)
NS_ICCE      = str(N_ICCE)
NS_IMU       = NS_ICCE+N_SEP+str(N_IMU)
NS_IMU_X           = NS_IMU+N_SEP+str(N_IMU_X)
NS_IMU_Y           = NS_IMU+N_SEP+str(N_IMU_Y)
NS_IMU_Z           = NS_IMU+N_SEP+str(N_IMU_Z)
NS_GPS       = NS_ICCE+N_SEP+str(N_GPS)
NS_GPS_LATI        = NS_GPS+N_SEP+str(N_GPS_LATI)
NS_GPS_LONG        = NS_GPS+N_SEP+str(N_GPS_LONG)
NS_COMPASS   = NS_ICCE+N_SEP+str(N_COMPASS)
NS_COMPASS_HEADING = NS_COMPASS+N_SEP+str(N_COMPASS_HEADING)
NS_INTERNAL  = NS_ICCE+N_SEP+str(N_INTERNAL)
NS_INTERNAL_TEMP   = NS_INTERNAL+N_SEP+str(N_INTERNAL_TEMP)
NS_INTERNAL_VOLT   = NS_INTERNAL+N_SEP+str(N_INTERNAL_VOLT)
NS_INTERNAL_CURR   = NS_INTERNAL+N_SEP+str(N_INTERNAL_CURR)
NS_DRIVE     = str(N_DRIVE)
NS_MOTOR_VOLT = NS_DRIVE+N_SEP+str(N_MOTOR_VOLT)
NS_MOTORFL_VOLT    = NS_MOTOR_VOLT+N_SEP+str(N_MOTORFL_VOLT)
NS_MOTORFR_VOLT    = NS_MOTOR_VOLT+N_SEP+str(N_MOTORFR_VOLT)
NS_MOTORRL_VOLT    = NS_MOTOR_VOLT+N_SEP+str(N_MOTORRL_VOLT)
NS_MOTORRR_VOLT    = NS_MOTOR_VOLT+N_SEP+str(N_MOTORRR_VOLT)
NS_SCIENCE   = str(N_SCIENCE)
NS_CORE       = NS_SCIENCE+N_SEP+str(N_CORE)
NS_CORE_HEIGHT     = NS_CORE+N_SEP+str(N_CORE_HEIGHT)
NS_DRILL       = NS_SCIENCE+N_SEP+str(N_DRILL)
NS_DRILL_SPIN      = NS_DRILL+N_SEP+str(N_DRILL_SPIN)
NS_TRAP       = NS_SCIENCE+N_SEP+str(N_TRAP)
NS_TRAP_OC         = NS_TRAP+N_SEP+str(N_TRAP_OC)
NS_SPECT       = NS_SCIENCE+N_SEP+str(N_SPECT)
NS_SPECT_IO        = NS_SPECT+N_SEP+str(N_SPECT_IO)
NS_ARM    = str(N_ARM)
NS_ARM_MAIN  = NS_ARM + N_SEP + str(N_ARM_MAIN)
# NS_ARM_MAIN_BASE        = NS_ARM_MAIN + N_SEP + str(N_ARM_MAIN_BASE)
# NS_ARM_MAIN_ACT1        = NS_ARM_MAIN + N_SEP + str(N_ARM_MAIN_ACT1)
# NS_ARM_MAIN_ACT2        = NS_ARM_MAIN + N_SEP + str(N_ARM_MAIN_ACT2)
NS_ARM_END   = NS_ARM + N_SEP + str(N_ARM_END)
NS_ARM_END_GRIP         = NS_ARM_END + N_SEP + str(N_ARM_END_GRIP)
NS_ARM_END_ROLL         = NS_ARM_END + N_SEP + str(N_ARM_END_ROLL)
# NS_ARM_END_YAW          = NS_ARM_END + N_SEP + str(N_ARM_END_YAW)
# NS_ARM_END_PITCH        = NS_ARM_END + N_SEP + str(N_ARM_END_PITCH)


# NS_MOTORFL   = NS_DRIVE+N_SEP+str(N_MOTORFL)
# NS_MOTORFL_VOLT    = NS_MOTORFL+N_SEP+str(N_MOTORFL_VOLT)
# NS_MOTORFR   = NS_DRIVE+N_SEP+str(N_MOTORFR)
# NS_MOTORFR_VOLT    = NS_MOTORFR+N_SEP+str(N_MOTORFR_VOLT)
# NS_MOTORRL   = NS_DRIVE+N_SEP+str(N_MOTORRL)
# NS_MOTORRL_VOLT    = NS_MOTORRL+N_SEP+str(N_MOTORRL_VOLT)
# NS_MOTORRR   = NS_DRIVE+N_SEP+str(N_MOTORRR)
# NS_MOTORRR_VOLT    = NS_MOTORRR+N_SEP+str(N_MOTORRR_VOLT)

class Component:
	def __init__(self, parent, cNum):
		self.cNum = cNum
		self.parent = parent
		self.child = []

	def info(self):
		return False

class ICCE(Component):
	def __init__(self, parent, cNum):
		Component.__init__(self, parent, cNum)

	def addComponents(self, imu, gps, compass, internal):
		self.child = [imu, gps, compass, internal]
		self.imu = self.child[0]
		self.gps = self.child[1]
		self.compass = self.child[2]
		self.internal = self.child[3]

class IMU(Component):
	def __init__(self, parent, cNum, x, y, z):
		Component.__init__(self, parent, cNum)
		self.child = [x,y,z]
		self.x = self.child[0]
		self.y = self.child[1]
		self.z = self.child[2]
		
class GPS(Component):
	def __init__(self, parent, cNum, lati, long):
		Component.__init__(self, parent, cNum)
		self.child.append(lati)
		self.child.append(long)
		self.lati = self.child[0]
		self.long = self.child[1]

class Compass(Component):
	def __init__(self, parent, cNum, heading):
		Component.__init__(self, parent, cNum)
		self.child.append(heading)
		self.heading = self.child[0]

class Internal(Component):
	def __init__(self, parent, cNum, temp, volt, curr):
		Component.__init__(self, parent, cNum)
		self.child.append(temp)
		self.child.append(volt)
		self.child.append(curr)
		self.temp = self.child[0]
		self.volt = self.child[1]
		self.curr = self.child[2]

class Drive(Component):
	def __init__(self, parent, cNum):
		Component.__init__(self, parent, cNum)
	
	def addComponents(self, volt):
		self.child = []
		self.child.append(volt)

class Voltage(Component):
	def __init__(self, parent, cNum, vFL, vFR, vRL, vRR):
		Component.__init__(self, parent, cNum)
		self.child = []
		self.child.append(vFL)
		self.child.append(vFR)
		self.child.append(vRL)
		self.child.append(vRR)

class Science(Component):
	def __init__(self, parent, cNum):
		Component.__init__(self, parent, cNum)
		self.child = []
	
	def addComponents(self, comp):
		self.child.append(comp)

class Core(Component):
	def __init__(self, parent, cNum):
		Component.__init__(self, parent, cNum)
		self.child = []
		self.child.append(0.0)# 0-100 Motor Speed

class Drill(Component):
	def __init__(self, parent, cNum):
		Component.__init__(self, parent, cNum)
		self.child = []
		self.child.append(0.0)# 0-100 Motor Speed

class Trap(Component):
	def __init__(self, parent, cNum):
		Component.__init__(self, parent, cNum)
		self.child = []
		self.child.append(0.0)# 0-1 Trap Door Open/Close

class Spect(Component):
	def __init__(self, parent, cNum):
		Component.__init__(self, parent, cNum)
		self.child = []
		self.child.append(0.0)# 0-1 Spectrometer In/Out

class Arm(Component):
	def __init__(self, parent, cNum):
		Component.__init__(self, parent, cNum)
		self.child = []
	
	def addComponents(self, comp):
		self.child.append(comp)

class ArmMain(Component):
	def __init__(self, parent, cNum, x,y,z):
		Component.__init__(self, parent, cNum)
		self.child = [x,y,z]
	
	def addComponents(self, comp):
		self.child.append(comp)

# class ArmMainBase(Component):
# 	def __init__(self, parent, cNum, base):
# 		Component.__init__(self, parent, cNum)
# 		self.child = [base]

# class ArmMainActuator(Component):
# 	def __init__(self, parent, cNum, act):
# 		Component.__init__(self, parent, cNum)
# 		self.child = [act]

class ArmEnd(Component):
	def __init__(self, parent, cNum):
		Component.__init__(self, parent, cNum)
		self.child = []
	
	def addComponents(self, comp):
		self.child.append(comp)

class ArmEndClaw(Component):
	def __init__(self, parent, cNum, x,y,z, grip, roll):
		Component.__init__(self, parent, cNum)
		self.child = [x,y,z, grip, roll]

class RoverModel(Component):
	heartbeat="pkt not received - watchdog engaged"
	timeout=30
	max_length=0
	max_timeout=0
	def __init__(self):
		Component.__init__(self, self, 0)
		self.icce = ICCE(self, N_ICCE)
		self.imu = IMU(self.icce, N_IMU, 0.0,0.0,0.0)
		self.gps = GPS(self.icce, N_GPS, 0.0,0.0)
		self.compass = Compass(self.icce, N_COMPASS, 0.0)
		self.internal = Internal(self.icce, N_ICCE, 0.0,0.0,0.0)
		self.icce.addComponents(self.imu, self.gps, self.compass, self.internal)
		self.drive = Drive(self, N_DRIVE)
		self.volt  = Voltage(self, NS_MOTOR_VOLT, 0.0,0.0,0.0,0.0)
		self.drive.addComponents(self.volt)
		self.science = Science(self, N_SCIENCE)
		self.core = Core(self, N_CORE)
		self.drill = Drill(self, N_DRILL)
		self.trap =  Trap(self, N_TRAP)
		self.spect = Spect(self, N_SPECT)
		self.science.addComponents(self.core)
		self.science.addComponents(self.drill)
		self.science.addComponents(self.trap)
		self.science.addComponents(self.spect)
		self.arm = Arm(self, N_ARM)
		self.armMain = ArmMain(self, N_ARM_MAIN, 0.0,0.0,0.0)
		# self.armMainBase = ArmMainBase(self, N_ARM_MAIN_BASE, 0.0)
		# self.armMainAct1 = ArmMainActuator(self, N_ARM_MAIN_ACT1, 0.0)
		# self.armMainAct2 = ArmMainActuator(self, N_ARM_MAIN_ACT2, 0.0)
		# self.armMain.addComponents(self.armMainBase)
		# self.armMain.addComponents(self.armMainAct1)
		# self.armMain.addComponents(self.armMainAct2)
		self.armEndClaw  = ArmEndClaw(self, N_ARM_END, 0.0,0.0,0.0, 0.0,0.0)
		self.arm.addComponents(self.armMain)
		self.arm.addComponents(self.armEndClaw)
		self.child = [self.icce,self.drive,self.science,self.arm]

	def encode(self, path, value):
		msg = ""
		msg += MSG_COMMAND
		msg += path
		msg += MSG_SEP
		msg += value
		chksum=self.getCHKSUM(msg+MSG_SEP)
		msg += MSG_SEP+str(chksum)+MSG_END
		return msg

	def decode(self, msg):
		array = msg[1:len(msg)-1].split(MSG_SEP)
		path = array[0]
		value = array[1]
		chksum = array[2]
		return path, value

	# for Telemetry; If different, update. If not, do nothing
	def updateTelemetry(self, msg):
		path, value = self.decode(msg)
		# print 'split',value.split(',')
		# print 'msg',cmdType, '_', path, '_', value
		return self.set(path, value.split(N_SEP_VAL))

	# for Server side: check if command needs to be pushed
	def isDifferent(self, msg):
		path, value = self.decode(msg)
		nums = path.split(N_SEP)
		if self.child[nums[0]].child[nums[1]].child[nums[2]] == value:
			return False
		return True

	def get(self, path):
		arr = path
		res = self
		for i in range(len(arr)):
			res = res.child[int(arr[i])]
		return res

	def validatePkt(self,pkt):
		validation =False

		size=len(pkt)
		#calculate checksum		


		if len(pkt.split('C'))==2 and len(pkt.split('E'))==2 and len(pkt.split('C')[0])==0 and len(pkt.split('E')[1])==0 :
			#validation=True
			path,value,chksum1=pkt[1:len(pkt)-1].split('_')
			chksum2=self.getCHKSUM(MSG_COMMAND+path+MSG_SEP+value+MSG_SEP)
			#print chksum1
			#print chksum2			
			if int(chksum1)==chksum2:

				validation=True
		
		return validation
			
		
	def getCHKSUM(self,pkt):
		chksum=0
		for i in range(0,len(pkt)-1):
				chksum+=chksum+ord(pkt[i])
		return chksum

	def set(self, path, value):
		nums = path
		isDifferent = False
		print 'path=',nums
		if len(nums) == 2:
			head = self.child[int(nums[0])].child[int(nums[1])]
			for i in range(len(head.child)):
				if head.child[i] != float(value[i]):
					head.child[i] = float(value[i])
					isDifferent = True
		elif len(nums) == 3:
			# print 'child', self.child[int(nums[0])].child[int(nums[1])].child
			if self.child[int(nums[0])].child[int(nums[1])].child[int(nums[2])] != value:
				self.child[int(nums[0])].child[int(nums[1])].child[int(nums[2])] = value
				isDifferent = True
		return isDifferent

	def receivePacket(self):
		received=False
		i=0
		j=0
		timer=time.time()
		while not received:
			data = self.ser.read()
			if data== "C":
				pkt=data
				print pkt
				while not data == "E":
					data=self.ser.read()
					pkt=pkt+data
					j+=1
					timer2=time.time()
					if timer2>timer+self.max_timeout:
						break
				#print pkt
				validation=self.validatePkt(pkt)
				if validation==True:
					received=True
				self.ser.flush()
			i+=1

			if i>self.timeout:
				received =True
				pkt=self.heartbeat			
		return pkt
