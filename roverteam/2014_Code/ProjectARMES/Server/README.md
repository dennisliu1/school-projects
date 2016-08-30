The 2013-2014 ARMES Project Server Framework.

The Server talks to the rover and handles all communications between the clients and the rover.

This framework is split into three sections:
1. Rover Communications
2. Server synchronization
3. Client(s) Communications

All sections use the RoverModel to synchronize the rover's status (aka RoverModel).

Telemetry pipeline:
Rover ->> Server ->> Client(s)

Command pipeline:
Rover <<- Server <<- Client(s)

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