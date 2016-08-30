package driveArmControl;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;

import javax.swing.JSlider;

import com.centralnexus.input.Joystick;

import driveArmControl.DriveArmPanel.ArmMotorDisplay;

public class GamepadInterpret implements Runnable 
{
	public final static int ARM_PORT = 30002;
	public final static String ARM_ADDRESS = "192.168.80.103";
	public DatagramSocket base;
	private int port;
	private InetAddress address;
	
	private Thread roverthread;
	private Joystick joy;
	public int interval;
	public Float deadZone1;
	private int button;
private final byte X_ID = 1, Y_ID = 2, Z_ID = 1, YAW_ID = 2, ROLL_ID = 1, PITCH_ID = 2, GRIPPER_ID = 1, ROLLER_ID = 2, ROLLER_TURN_ID = 1;
private final byte PARENT_ID[] = {2, 3, 4, 5, 6};
private final int NUM_BUTTONS = 12, NUM_AXIS = 5, ARM_MOTOR_NUM = 9, PARENT_ID_OFFSET = PARENT_ID[0], MOTOR_ID_OFFSET = X_ID;
public boolean bButton[] = new boolean[NUM_BUTTONS], bCalibratedMotor[][] = new boolean[PARENT_ID.length][2]; 
public float leftX, leftY, rightX, rightY, dPadX, dPadY;
public int multiSpeed, currArmMotorPos[][] = new int[PARENT_ID.length][2], maxArmMotorPos[][] = new int[PARENT_ID.length][2];

private boolean bMotorBX,bMotorEX, bMotorBY,bMotorEY, bMotorBZ,bMotorEZ, bMotorBYaw,bMotorEYaw, bMotorBRoll,bMotorERoll, 
			bMotorBPitch,bMotorEPitch, bMotorBGripper,bMotorEGripper, bMotorBRoller,bMotorERoller, bMotorBRollerTurn,bMotorERollerTurn;
private final String DIR_LEFT = "L", DIR_RIGHT = "R", DIR_UP ="Up", DIR_DOWN = "Dwn", DIR_STOP = "n/a";

	private final int BREAK = 0, MAX_TURN = 126;
	private final byte FORWARD_COMMAND = 0x32;
	private final byte FORWARD_PULSE_COMMAND = 0x31;
	private final byte TURN_COMMAND = 0x35;
	private final byte COMMIT_COMMAND = (byte)0xFF;
	private final byte BRAKE_ALL_COMMAND = (byte) 0x90;
	private final byte SET_SPEED_COMMAND = (byte) 0xC1;
	private final byte SET_POSITION_COMMAND = (byte) 0xC2;
	private final byte READY_COMMAND = (byte) 0x0F;
	private final byte EMERGENCY_STOP_COMMAND = (byte) 0x0;
	private final byte CALIBRATE_COMMAND = (byte) 0xC0;
	public byte[] packet = new byte[4];
	// Set to negative if motor turns clockwise for negative speed
	private final int CLOCKWISE_MOTOR_POSITIVE = 1; 
	
	private final int NORMAL_DRIVE_MODE = 1;
	private final int FRONT_WHEEL_DRIVE_MODE = 3;
	private final int FLASH_MODE = 2;
	private final int READY_MODE = 4;
	private final int EMERGENCY_STOP_MODE = 5;
	
	private int started = 0;
	public JSlider speedDisplay;
	private ArmMotorDisplay feedbackPanel[][];
	//public static final int TURN_PERCENT = 200;
	
	DriveArmPanel DAPanel;
	
	
	//Constructor: 
	//connect to joystick and start running itself as a thread
	public GamepadInterpret(String ip, int port, JSlider speedDisplay, ArmMotorDisplay[][] gamepadFeedback, DatagramSocket sock, DriveArmPanel armPanel) {
		multiSpeed = speedDisplay.getValue();
		for(int i = 0; i < NUM_BUTTONS; i++) {
			bButton[i] = false;//default for buttons
		}
		for(int i = 0; i < bCalibratedMotor.length; i++) {
			for(int j = 0; j< bCalibratedMotor[0].length; j++) {
				bCalibratedMotor[i][j] = false;
				currArmMotorPos[i][j] = -1;
				maxArmMotorPos[i][j] = -1;
			}
		}
		
		try {
			this.joy = Joystick.createInstance(); //connect to joystick
			this.interval = 100;
			roverthread = new Thread(this);
			this.address = InetAddress.getByName(ip);
			this.port = port;
			this.base = sock;
		} catch (Exception e) {
			e.printStackTrace();
		}
		this.speedDisplay = speedDisplay;
		this.feedbackPanel = gamepadFeedback;
		this.DAPanel = armPanel;
	}//constructor
	
	//both set cmds assume IDs are valid, not the internal array based ones.
	public void sendSpeedCmd(int parentNum, int motorNum, int speed, String dir) {
		this.packet[0] = (byte) ((parentNum << 4) | motorNum);
		this.packet[1] = SET_SPEED_COMMAND;
		this.packet[2] = (byte)0;//get leftX[15:8]
		this.packet[3] = (byte)speed;//get leftX[7:0]
		this.write(packet);
		
		int lightNum = -1;
		if(dir.equals(DIR_LEFT)) {
			lightNum = 1;
		} else if(dir.equals(DIR_RIGHT)) {
			lightNum = 1;
		} else if(dir.equals(DIR_STOP)) {
			lightNum = 0;
		}
		this.feedbackPanel[parentNum-PARENT_ID_OFFSET][motorNum-MOTOR_ID_OFFSET].directionTA.setText(dir);
		this.feedbackPanel[parentNum-PARENT_ID_OFFSET][motorNum-MOTOR_ID_OFFSET].limitSwitchLight.setColor(lightNum);//offcolor(green)
	}
	public void sendPositionCmd(int parentNum, int motorNum, int position, String dir) {
		this.packet[0] = (byte) ((parentNum << 4) | motorNum);
		this.packet[1] = SET_POSITION_COMMAND;
		this.packet[2] = (byte)(position >> 8);//get [15:8]
		this.packet[3] = (byte)(position);//get [7:0]
		this.write(packet);
		
		int lightNum = -1;
		if(dir.equals(DIR_LEFT)) {
			lightNum = 1;
		} else if(dir.equals(DIR_RIGHT)) {
			lightNum = 1;
		} else if(dir.equals(DIR_STOP)) {
			lightNum = 0;
		}
		this.feedbackPanel[parentNum-PARENT_ID_OFFSET][motorNum-MOTOR_ID_OFFSET].directionTA.setText(dir);
		this.feedbackPanel[parentNum-PARENT_ID_OFFSET][motorNum-MOTOR_ID_OFFSET].limitSwitchLight.setColor(lightNum);//offcolor(green)
	}
	
	//Phase 1: read in joystick input and translate into 
	public void updateFieldsEx(Joystick joystick) throws Exception {
		if(joystick.isButtonDown(joystick.BUTTON1)) bButton[0] = true;
		else bButton[0] = false;
		if(joystick.isButtonDown(joystick.BUTTON2)) bButton[1] = true;
		else bButton[1] = false;
		if(joystick.isButtonDown(joystick.BUTTON3)) bButton[2] = true;
		else bButton[2] = false;
		if(joystick.isButtonDown(joystick.BUTTON4)) bButton[3] = true;
		else bButton[3] = false;
		if(joystick.isButtonDown(joystick.BUTTON5)) bButton[4] = true;
		else bButton[4] = false;
		if(joystick.isButtonDown(joystick.BUTTON6)) bButton[5] = true;
		else bButton[5] = false;
		if(joystick.isButtonDown(joystick.BUTTON7)) bButton[6] = true;
		else bButton[6] = false;
		if(joystick.isButtonDown(joystick.BUTTON8)) bButton[7] = true;
		else bButton[7] = false;
		if(joystick.isButtonDown(joystick.BUTTON9)) bButton[8] = true;
		else bButton[8] = false;
		if(joystick.isButtonDown(joystick.BUTTON10)) bButton[9] = true;
		else bButton[9] = false;
		if(joystick.isButtonDown(joystick.BUTTON11)) bButton[10] = true;
		else bButton[10] = false;
		if(joystick.isButtonDown(joystick.BUTTON12)) bButton[11] = true;
		else bButton[11] = false;
		
		button = convertButton(joystick.getButtons());
		leftX = joy.getX() * multiSpeed;
		leftY = -joy.getY() * multiSpeed;
		rightX = joy.getR() * multiSpeed; //-1 <> 1
		rightY = -joy.getZ() * multiSpeed;//1 ^v -1
		dPadX = joy.getU() * multiSpeed;
		dPadY = -joy.getV() * multiSpeed;
		System.out.printf("X:%.4f Y:%.4f U:%.4f V:%.4f R:%.4f Z:%.4f\n", 
			joy.getX(), joy.getY(), joy.getU(), joy.getV(), joy.getR(), joy.getZ());
		//-----------	Gamepad Configuration	--------------//
		bMotorBX = ((int)leftX < 0);
		bMotorEX = ((int)leftX > 0);
		bMotorBY = ((int)leftY < 0);
		bMotorEY = ((int)leftY > 0);
		bMotorBZ = ((int)rightY < 0);
		bMotorEZ = ((int)rightY > 0);
		bMotorBYaw = ((int)dPadX < 0);
		bMotorEYaw = ((int)dPadX > 0);
		bMotorBRoll = bButton[3];
		bMotorERoll = bButton[2];
		bMotorBPitch = ((int)dPadY < 0);
		bMotorEPitch = ((int)dPadY > 0);
		bMotorBGripper = bButton[6];
		bMotorEGripper = bButton[7];
		bMotorBRoller = bButton[4];
		bMotorERoller = bButton[5];
		bMotorBRollerTurn = bButton[1];
		bMotorERollerTurn = bButton[0];
		//-----------	Gamepad Configuration	--------------//
		
		makeByteArray();
		//System.out.printf("X:%.4f Y:%.4f Z:%.4f DPADX:%.4f DPADY:%.4f\n", leftX,leftY, rightY, dPadX,dPadY);
		//System.out.printf("X:%b,%b Y:%b,%b, Z:%b,%b Yaw:%b,%b, Roll:%b,%b Pitch:%b,%b Grip:%b,%b Roller:%b,%b Turn:%b,%b\n",bMotorBX,bMotorEX, bMotorBY,bMotorEY, bMotorBZ,bMotorEZ, bMotorBYaw,bMotorEYaw, bMotorBRoll,bMotorERoll, 
		//	bMotorBPitch,bMotorEPitch, bMotorBGripper,bMotorEGripper, bMotorBRoller,bMotorERoller, bMotorBRollerTurn,bMotorERollerTurn);
		
		/*
		System.out.printf("1:%b 2:%b 3:%b 4:%b 5:%b 6:%b 7:%b 8:%b 9:%b 10:%b 11:%b 12:%b X:%.3f Y:%.3f X2:%.3f Y2:%.3f DX:%.3f DY:%.3f\n", 
				bButton[0], bButton[1], bButton[2], bButton[3], bButton[4], bButton[5], bButton[6], bButton[7], bButton[8], bButton[9], bButton[10], bButton[11],
				leftX, leftY, rightX, rightY, dPadX, dPadY);
				*/
    }//updateFieldsEX method
	
	//Phase 2: Create the byte Array to be sent to the OBC
	//Also calls Phase 3 and sends the bytes to the OBC
	//Checks each button and sends a SET_SPEED or SET_POSITION depending on if motor is calibrated.
	//if button is not pressed, a wait command is sent, but only if motor is not calibrated (so using speed command)
	public void makeByteArray() {
		if(bMotorBX) { //X Axis 0
			this.packet[0] = (byte) ((PARENT_ID[0] << 4) | X_ID);
			if(bCalibratedMotor[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1] <= maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1]-multiSpeed) 
					currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1] += leftX;
				else currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1] = maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1];
				sendPositionCmd(PARENT_ID[0], X_ID, currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1], DIR_LEFT);
				DAPanel.setXSliderPos(currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1]);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[0], X_ID, (int)leftX, DIR_LEFT);
			}
		} else if(bMotorEX) { //X Axis
			this.packet[0] = (byte) ((PARENT_ID[0] << 4) | X_ID);
			if(bCalibratedMotor[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1] <= maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1]-multiSpeed) 
					currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1] += leftX;
				else currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1] = maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1];
				sendPositionCmd(PARENT_ID[0], X_ID, currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1], DIR_RIGHT);
				DAPanel.setXSliderPos(currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1]);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[0], X_ID, (int)leftX, DIR_RIGHT);
			}
		} else if(!bCalibratedMotor[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1]) {
			sendSpeedCmd(PARENT_ID[0], X_ID, 0, DIR_STOP);
		}
	//------------------------------------------------------------------------------------------------------------
		if(bMotorBY) { //Y Axis 0
			this.packet[0] = (byte) ((PARENT_ID[0] << 4) | Y_ID);
			if(bCalibratedMotor[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1] <= maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1]-multiSpeed) 
					currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1] += leftY;
				else currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1] = maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1];
				sendPositionCmd(PARENT_ID[0], Y_ID, currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1], DIR_LEFT);
				DAPanel.setYSliderPos(currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1]);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[0], Y_ID, (int)leftY, DIR_LEFT);
			}
		} else if(bMotorEY) { //Y Axis
			this.packet[0] = (byte) ((PARENT_ID[0] << 4) | Y_ID);
			if(bCalibratedMotor[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1] <= maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1]-multiSpeed) 
					currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1] += leftY;
				else currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1] = maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1];
				sendPositionCmd(PARENT_ID[0], Y_ID, currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1], DIR_RIGHT);
				DAPanel.setYSliderPos(currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1]);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[0], Y_ID, (int)leftY, DIR_RIGHT);
			}
		} else if(!bCalibratedMotor[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1]) {
			sendSpeedCmd(PARENT_ID[0], Y_ID, 0, DIR_STOP);
		}
	//----------------------------------------------------------------------------------------------------------------------
		if(bMotorBZ) { //Z Axis 1
			this.packet[0] = (byte) ((PARENT_ID[1] << 4) | Z_ID);
			if(bCalibratedMotor[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1] <= maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1]-multiSpeed) 
					currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1] += rightY;
				else currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1] = maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1];
				sendPositionCmd(PARENT_ID[1], Z_ID, currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1], DIR_LEFT);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[1], Z_ID, (int)rightY, DIR_LEFT);
			}
			this.write(packet);
		} else if(bMotorEZ) { //Z Axis
			this.packet[0] = (byte) ((PARENT_ID[1] << 4) | Z_ID);
			if(bCalibratedMotor[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1] <= maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1]-multiSpeed) 
					currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1] += rightY;
				else currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1] = maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1];
				sendPositionCmd(PARENT_ID[1], Z_ID, currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1], DIR_RIGHT);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[1], Z_ID, (int)rightY, DIR_RIGHT);
			}
		} else if(!bCalibratedMotor[PARENT_ID[1]][Z_ID-1]) {
			sendSpeedCmd(PARENT_ID[1], Z_ID, 0, DIR_STOP);
		}
	//-------------------------------------------------------------------------------------------------------------------
		if(bMotorBYaw) { //Yaw 1
			this.packet[0] = (byte) ((PARENT_ID[1] << 4) | YAW_ID);
			if(bCalibratedMotor[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1] <= maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1]-multiSpeed) 
					currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1] += dPadX;
				else currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1] = maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1];
				sendPositionCmd(PARENT_ID[1], YAW_ID, currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1], DIR_LEFT);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[1], YAW_ID, (int)dPadY, DIR_LEFT);
			}
		} else if(bMotorEYaw) { //Pitch (up/down)
			this.packet[0] = (byte) ((PARENT_ID[1] << 4) | YAW_ID);
			if(bCalibratedMotor[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1] <= maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1]-multiSpeed) 
					currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1] += dPadX;
				else currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1] = maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1];
				sendPositionCmd(PARENT_ID[1], YAW_ID, currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1], DIR_RIGHT);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[1], YAW_ID, (int)dPadY, DIR_RIGHT);
			}
		} else if(!bCalibratedMotor[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1]) {
			sendSpeedCmd(PARENT_ID[1], YAW_ID, 0, DIR_STOP);
		}
	//----------------------------------------------------------------------------------------------------------------------
		if(bMotorBRoll) { // roll 2
			this.packet[0] = (byte) ((PARENT_ID[2] << 4) | ROLL_ID);
			if(bCalibratedMotor[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1] <= maxArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1]-multiSpeed) 
					currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1] += multiSpeed;
				else currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1] = maxArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1];
				sendPositionCmd(PARENT_ID[2], ROLL_ID, currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1], DIR_LEFT);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[2], ROLL_ID, multiSpeed, DIR_LEFT);
			}
			this.write(packet);
			this.feedbackPanel[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1].directionTA.setText(DIR_LEFT);
			this.feedbackPanel[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1].limitSwitchLight.setColor(1);
		} else if(bMotorERoll) { // four(4) is roll end-effector c-clockwise
			this.packet[0] = (byte) ((PARENT_ID[2] << 4) | ROLL_ID);
			if(bCalibratedMotor[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1] >= multiSpeed) currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1] -= multiSpeed;
				else currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1] = 0;
				sendPositionCmd(PARENT_ID[2], ROLL_ID, currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1], DIR_RIGHT);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[2], ROLL_ID, -1*multiSpeed, DIR_RIGHT);
			}
			this.write(packet);
		} else if(!bCalibratedMotor[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1]) {//send wait command when button is let go
			sendSpeedCmd(PARENT_ID[2], ROLL_ID, 0, DIR_STOP);
		}
	//-----------------------------------------------------------------------------------------------------------------------------
		if(bMotorBPitch) { //pitch 2
			this.packet[0] = (byte) ((PARENT_ID[2] << 4) | PITCH_ID);
			if(bCalibratedMotor[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1] <= maxArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1]-multiSpeed) 
					currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1] += dPadY;
				else currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1] = maxArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1];
				sendPositionCmd(PARENT_ID[2], PITCH_ID, currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1], DIR_LEFT);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[2], PITCH_ID, (int)dPadX, DIR_LEFT);
			}
		} else if(bMotorEPitch) { 
			this.packet[0] = (byte) ((PARENT_ID[2] << 4) | PITCH_ID);
			if(bCalibratedMotor[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1] <= maxArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1]-multiSpeed)
					currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1] += dPadY;
				else currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1] = maxArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1];
				sendPositionCmd(PARENT_ID[2], PITCH_ID, currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1], DIR_RIGHT);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[2], PITCH_ID, (int)dPadX, DIR_RIGHT);
			}
		} else if(!bCalibratedMotor[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1]) {
			sendSpeedCmd(PARENT_ID[2], PITCH_ID, 0, DIR_STOP);
		}
	//----------------------------------------------------------------------------------------------------------------------------
		if(bMotorBGripper) { // gripper 3
			this.packet[0] = (byte) ((PARENT_ID[3] << 4) | GRIPPER_ID);
			if(bCalibratedMotor[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1] <= maxArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1]-multiSpeed) 
					currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1] += multiSpeed;
				else currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1] = maxArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1];
				sendPositionCmd(PARENT_ID[3], GRIPPER_ID, currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1], DIR_LEFT);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[3], GRIPPER_ID, multiSpeed, DIR_LEFT);
			}
		} else if(bMotorEGripper) { // eight(8) is open the grippers (move right)
			this.packet[0] = (byte) ((PARENT_ID[3] << 4) | GRIPPER_ID);
			if(bCalibratedMotor[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1] >= multiSpeed) 
					currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1] -= multiSpeed;
				else currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1] = 0;
				sendPositionCmd(PARENT_ID[3], GRIPPER_ID, currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1], DIR_RIGHT);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[3], GRIPPER_ID, multiSpeed, DIR_RIGHT);
			}
		} else if(!bCalibratedMotor[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1]) {//send wait command when button is let go
			sendSpeedCmd(PARENT_ID[3], GRIPPER_ID, 0, DIR_STOP);
		}
	//--------------------------------------------------------------------------------------------------------------------------
		if(bMotorBRoller) { // roller  3
			this.packet[0] = (byte) ((PARENT_ID[3] << 4) | ROLLER_ID);
			if(bCalibratedMotor[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1] <= maxArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1]-multiSpeed) 
					currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1] += multiSpeed;
				else currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1] = maxArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1];
				sendPositionCmd(PARENT_ID[3], ROLLER_ID, currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1], DIR_LEFT);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[3], ROLLER_ID, multiSpeed, DIR_LEFT);
			}
		} else if(bMotorERoller) { // six(6) is open the rollers (move right)
			this.packet[0] = (byte) ((PARENT_ID[3] << 4) | ROLLER_ID);
			if(bCalibratedMotor[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1] >= multiSpeed) 
					currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1] -= multiSpeed;
				else currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1] = 0;
				sendPositionCmd(PARENT_ID[3], ROLLER_ID, currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1], DIR_RIGHT);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[3], ROLLER_ID, multiSpeed, DIR_RIGHT);
			}
		} else if(!bCalibratedMotor[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1]) {//send wait command when button is let go
			sendSpeedCmd(PARENT_ID[3], ROLLER_ID, 0, DIR_STOP);
		}
	//-----------------------------------------------------------------------------------------------------------------------------
		if(bMotorBRollerTurn) {// roller turn 4
			this.packet[0] = (byte) ((PARENT_ID[4] << 4) | ROLLER_TURN_ID);
			if(bCalibratedMotor[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1] <= maxArmMotorPos[PARENT_ID[4]][ROLLER_TURN_ID-1]-multiSpeed) 
					currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1] += multiSpeed;
				else currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1] = maxArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1];
				sendPositionCmd(PARENT_ID[4], ROLLER_TURN_ID, currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1], DIR_LEFT);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[4], ROLLER_TURN_ID, multiSpeed, DIR_LEFT);
			}
			this.feedbackPanel[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1].directionTA.setText(DIR_LEFT);
			this.feedbackPanel[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1].limitSwitchLight.setColor(1);
		} else if(bMotorERollerTurn) { // two(2) is turn rollers to push cord down
			this.packet[0] = (byte) ((PARENT_ID[4] << 4) | ROLLER_TURN_ID);
			if(bCalibratedMotor[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1]) {//validated, use SET_POSITION
				if(currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1] >= multiSpeed) 
					currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1] -= multiSpeed;
				else currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1] = 0;
				sendPositionCmd(PARENT_ID[4], ROLLER_TURN_ID, currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1], DIR_RIGHT);
			} else {//Invalidated, use SET_SPEED
				sendSpeedCmd(PARENT_ID[4], ROLLER_TURN_ID, -1*multiSpeed, DIR_RIGHT);
			}
		} else if(!bCalibratedMotor[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1]) {//send wait command when button is let go
			sendSpeedCmd(PARENT_ID[4], ROLLER_TURN_ID, 0, DIR_STOP);
		}
		if(bButton[8]) {
			if(multiSpeed > speedDisplay.getMinimum()) multiSpeed--;
			speedDisplay.setValue(multiSpeed);
			System.out.printf("multiSpeed:%d\n",multiSpeed);
		}
		if(bButton[9]) {
			if(multiSpeed < speedDisplay.getMaximum()) multiSpeed++;
			speedDisplay.setValue(multiSpeed);
			System.out.printf("multiSpeed:%d\n",multiSpeed);
		}
		
		
		
		
		
		
		
		
		
		
		
//------------------------------------------------------------------------------------------------------------------------------------------
		
//		if(bMotorBX) { //X Axis 0
//			this.packet[0] = (byte) ((PARENT_ID[0] << 4) | X_ID);
//			if(bCalibratedMotor[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1] <= maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1]-multiSpeed) 
//					currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1] += leftX;
//				else currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1] = maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1];
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte)(0);//get leftX[15:8]
//				this.packet[3] = (byte)((int)leftX);	 //get leftX[7:0]
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1].directionTA.setText(DIR_LEFT);
//			this.feedbackPanel[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1].limitSwitchLight.setColor(1);
//		} else if(bMotorEX) { //X Axis
//			this.packet[0] = (byte) ((PARENT_ID[0] << 4) | X_ID);
//			if(bCalibratedMotor[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1] <= maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1]-multiSpeed) 
//					currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1] += leftX;
//				else currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1] = maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1];
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte)(0);//get leftX[15:8]
//				this.packet[3] = (byte)((int)leftX);	 //get leftX[7:0]
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1].directionTA.setText(DIR_RIGHT);
//			this.feedbackPanel[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1].limitSwitchLight.setColor(1);
//		} else if(!bCalibratedMotor[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1]) {
//			this.packet[0] = (byte) ((PARENT_ID[0] << 4) | X_ID);
//			this.packet[1] = SET_SPEED_COMMAND;
//			this.packet[2] = (byte)0;//get leftX[15:8]
//			this.packet[3] = (byte)0;//get leftX[7:0]
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1].directionTA.setText(DIR_STOP);
//			this.feedbackPanel[PARENT_ID[0]-PARENT_ID_OFFSET][X_ID-1].limitSwitchLight.setColor(0);//offcolor(green)
//		}
//	//------------------------------------------------------------------------------------------------------------
//		if(bMotorBY) { //Y Axis 0
//			this.packet[0] = (byte) ((PARENT_ID[0] << 4) | Y_ID);
//			if(bCalibratedMotor[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1] <= maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1]-multiSpeed) 
//					currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1] += leftY;
//				else currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1] = maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1];
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte)(0);//get leftX[15:8]
//				this.packet[3] = (byte)((int)leftY);	 //get leftX[7:0]
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1].directionTA.setText(DIR_LEFT);
//			this.feedbackPanel[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1].limitSwitchLight.setColor(1);
//		} else if(bMotorEY) { //Y Axis
//			this.packet[0] = (byte) ((PARENT_ID[0] << 4) | Y_ID);
//			if(bCalibratedMotor[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1] <= maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1]-multiSpeed) 
//					currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1] += leftY;
//				else currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1] = maxArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1];
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte)(0);//get leftX[15:8]
//				this.packet[3] = (byte)((int)leftY);	 //get leftX[7:0]
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1].directionTA.setText(DIR_RIGHT);
//			this.feedbackPanel[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1].limitSwitchLight.setColor(1);
//		} else if(!bCalibratedMotor[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1]) {
//			this.packet[0] = (byte) ((PARENT_ID[0] << 4) | Y_ID);
//			this.packet[1] = SET_SPEED_COMMAND;
//			this.packet[2] = (byte)0;//get leftX[15:8]
//			this.packet[3] = (byte)0;//get leftX[7:0]
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1].directionTA.setText(DIR_STOP);
//			this.feedbackPanel[PARENT_ID[0]-PARENT_ID_OFFSET][Y_ID-1].limitSwitchLight.setColor(0);
//		}
//	//----------------------------------------------------------------------------------------------------------------------
//		if(bMotorBZ) { //Z Axis 1
//			this.packet[0] = (byte) ((PARENT_ID[1] << 4) | Z_ID);
//			if(bCalibratedMotor[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1] <= maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1]-multiSpeed) 
//					currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1] += rightY;
//				else currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1] = maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1];
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte)(0);//get leftX[15:8]
//				this.packet[3] = (byte)((int)rightY);	 //get leftX[7:0]
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1].directionTA.setText(DIR_LEFT);
//			this.feedbackPanel[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1].limitSwitchLight.setColor(1);
//		} else if(bMotorEZ) { //Z Axis
//			this.packet[0] = (byte) ((PARENT_ID[1] << 4) | Z_ID);
//			if(bCalibratedMotor[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1] <= maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1]-multiSpeed) 
//					currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1] += rightY;
//				else currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1] = maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1];
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte)(0);//get leftX[15:8]
//				this.packet[3] = (byte)((int)rightY);	 //get leftX[7:0]
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1].directionTA.setText(DIR_RIGHT);
//			this.feedbackPanel[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1].limitSwitchLight.setColor(1);
//		} else if(!bCalibratedMotor[PARENT_ID[1]][Z_ID-1]) {
//			this.packet[0] = (byte) ((PARENT_ID[1] << 4) | Z_ID);
//			this.packet[1] = SET_SPEED_COMMAND;
//			this.packet[2] = (byte)0;//get leftX[15:8]
//			this.packet[3] = (byte)0;//get leftX[7:0]
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1].directionTA.setText(DIR_STOP);
//			this.feedbackPanel[PARENT_ID[1]-PARENT_ID_OFFSET][Z_ID-1].limitSwitchLight.setColor(0);
//		}
//	//-------------------------------------------------------------------------------------------------------------------
//		if(bMotorBYaw) { //Yaw 1
//			this.packet[0] = (byte) ((PARENT_ID[1] << 4) | YAW_ID);
//			if(bCalibratedMotor[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1] <= maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1]-multiSpeed) 
//					currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1] += dPadX;
//				else currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1] = maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1];
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte)(0);//get leftX[15:8]
//				this.packet[3] = (byte)((int)dPadY);	 //get leftX[7:0]
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1].directionTA.setText(DIR_LEFT);
//			this.feedbackPanel[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1].limitSwitchLight.setColor(1);
//		} else if(bMotorEYaw) { //Pitch (up/down)
//			this.packet[0] = (byte) ((PARENT_ID[1] << 4) | YAW_ID);
//			if(bCalibratedMotor[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1] <= maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1]-multiSpeed) 
//					currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1] += dPadX;
//				else currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1] = maxArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1];
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte)(0);//get leftX[15:8]
//				this.packet[3] = (byte)((int)dPadY);	 //get leftX[7:0]
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1].directionTA.setText(DIR_RIGHT);
//			this.feedbackPanel[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1].limitSwitchLight.setColor(1);
//		} else if(!bCalibratedMotor[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1]) {
//			this.packet[0] = (byte) ((PARENT_ID[1] << 4) | YAW_ID);
//			this.packet[1] = SET_SPEED_COMMAND;
//			this.packet[2] = (byte)0;//get leftX[15:8]
//			this.packet[3] = (byte)0;//get leftX[7:0]
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1].directionTA.setText(DIR_STOP);
//			this.feedbackPanel[PARENT_ID[1]-PARENT_ID_OFFSET][YAW_ID-1].limitSwitchLight.setColor(0);
//		}
//	//----------------------------------------------------------------------------------------------------------------------
//		if(bMotorBRoll) { // roll 2
//			this.packet[0] = (byte) ((PARENT_ID[2] << 4) | ROLL_ID);
//			if(bCalibratedMotor[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1] <= maxArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1]-multiSpeed) 
//					currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1] += multiSpeed;
//				else currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1] = maxArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1];
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte) 0;
//				this.packet[3] = (byte) multiSpeed;
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1].directionTA.setText(DIR_LEFT);
//			this.feedbackPanel[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1].limitSwitchLight.setColor(1);
//		} else if(bMotorERoll) { // four(4) is roll end-effector c-clockwise
//			this.packet[0] = (byte) ((PARENT_ID[2] << 4) | ROLL_ID);
//			if(bCalibratedMotor[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1] >= multiSpeed) currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1] -= multiSpeed;
//				else currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1] = 0;
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte) 0;
//				this.packet[3] = (byte) -multiSpeed;
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1].directionTA.setText(DIR_RIGHT);
//			this.feedbackPanel[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1].limitSwitchLight.setColor(1);
//		} else if(!bCalibratedMotor[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1]) {//send wait command when button is let go
//			this.packet[0] = (byte) ((PARENT_ID[2] << 4) | ROLL_ID);
//			this.packet[1] = SET_SPEED_COMMAND;
//			this.packet[2] = (byte) 0;
//			this.packet[3] = (byte) 0;
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1].directionTA.setText(DIR_STOP);
//			this.feedbackPanel[PARENT_ID[2]-PARENT_ID_OFFSET][ROLL_ID-1].limitSwitchLight.setColor(0);
//		}
//	//-----------------------------------------------------------------------------------------------------------------------------
//		if(bMotorBPitch) { //pitch 2
//			this.packet[0] = (byte) ((PARENT_ID[2] << 4) | PITCH_ID);
//			if(bCalibratedMotor[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1] <= maxArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1]-multiSpeed) 
//					currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1] += dPadY;
//				else currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1] = maxArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1];
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte)(0);//get leftX[15:8]
//				this.packet[3] = (byte)((int)dPadX);	 //get leftX[7:0]
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1].directionTA.setText(DIR_LEFT);
//			this.feedbackPanel[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1].limitSwitchLight.setColor(1);
//		} else if(bMotorEPitch) { //Yaw (left/right)
//			this.packet[0] = (byte) ((PARENT_ID[2] << 4) | PITCH_ID);
//			if(bCalibratedMotor[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1] <= maxArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1]-multiSpeed)
//					currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1] += dPadY;
//				else currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1] = maxArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1];
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte)(0);//get leftX[15:8]
//				this.packet[3] = (byte)((int)dPadX);	 //get leftX[7:0]
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1].directionTA.setText(DIR_RIGHT);
//			this.feedbackPanel[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1].limitSwitchLight.setColor(1);
//		} else if(!bCalibratedMotor[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1]) {
//			this.packet[0] = (byte) ((PARENT_ID[2] << 4) | PITCH_ID);
//			this.packet[1] = SET_SPEED_COMMAND;
//			this.packet[2] = (byte)0;//get leftX[15:8]
//			this.packet[3] = (byte)0;//get leftX[7:0]
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1].directionTA.setText(DIR_STOP);
//			this.feedbackPanel[PARENT_ID[2]-PARENT_ID_OFFSET][PITCH_ID-1].limitSwitchLight.setColor(0);
//		}
//	//----------------------------------------------------------------------------------------------------------------------------
//		if(bMotorBGripper) { // gripper 3
//			this.packet[0] = (byte) ((PARENT_ID[3] << 4) | GRIPPER_ID);
//			if(bCalibratedMotor[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1] <= maxArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1]-multiSpeed) 
//					currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1] += multiSpeed;
//				else currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1] = maxArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1];
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte) 0;
//				this.packet[3] = (byte) multiSpeed;
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1].directionTA.setText(DIR_LEFT);
//			this.feedbackPanel[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1].limitSwitchLight.setColor(1);
//		} else if(bMotorEGripper) { // eight(8) is open the grippers (move right)
//			this.packet[0] = (byte) ((PARENT_ID[3] << 4) | GRIPPER_ID);
//			if(bCalibratedMotor[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1] >= multiSpeed) 
//					currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1] -= multiSpeed;
//				else currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1] = 0;
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte) 0;
//				this.packet[3] = (byte) -multiSpeed;
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1].directionTA.setText(DIR_RIGHT);
//			this.feedbackPanel[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1].limitSwitchLight.setColor(1);
//		} else if(!bCalibratedMotor[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1]) {//send wait command when button is let go
//			this.packet[0] = (byte) ((PARENT_ID[3] << 4) | GRIPPER_ID);
//			this.packet[1] = SET_SPEED_COMMAND;
//			this.packet[2] = (byte) 0;
//			this.packet[3] = (byte) 0;
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1].directionTA.setText(DIR_STOP);
//			this.feedbackPanel[PARENT_ID[3]-PARENT_ID_OFFSET][GRIPPER_ID-1].limitSwitchLight.setColor(0);
//		}
//	//--------------------------------------------------------------------------------------------------------------------------
//		if(bMotorBRoller) { // roller  3
//			this.packet[0] = (byte) ((PARENT_ID[3] << 4) | ROLLER_ID);
//			if(bCalibratedMotor[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1] <= maxArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1]-multiSpeed) 
//					currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1] += multiSpeed;
//				else currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1] = maxArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1];
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte) 0;
//				this.packet[3] = (byte) multiSpeed;
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1].directionTA.setText(DIR_LEFT);
//			this.feedbackPanel[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1].limitSwitchLight.setColor(1);
//		} else if(bMotorERoller) { // six(6) is open the rollers (move right)
//			this.packet[0] = (byte) ((PARENT_ID[3] << 4) | ROLLER_ID);
//			if(bCalibratedMotor[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1] >= multiSpeed) 
//					currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1] -= multiSpeed;
//				else currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1] = 0;
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte) 0;
//				this.packet[3] = (byte) -multiSpeed;
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1].directionTA.setText(DIR_RIGHT);
//			this.feedbackPanel[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1].limitSwitchLight.setColor(1);
//		} else if(!bCalibratedMotor[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1]) {//send wait command when button is let go
//			this.packet[0] = (byte) ((PARENT_ID[3] << 4) | ROLLER_ID);
//			this.packet[1] = SET_SPEED_COMMAND;
//			this.packet[2] = (byte) 0;
//			this.packet[3] = (byte) 0;
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1].directionTA.setText(DIR_STOP);
//			this.feedbackPanel[PARENT_ID[3]-PARENT_ID_OFFSET][ROLLER_ID-1].limitSwitchLight.setColor(0);
//		}
//	//-----------------------------------------------------------------------------------------------------------------------------
//		if(bMotorBRollerTurn) {// one(1) is turn rollers to pull cord up 4
//			this.packet[0] = (byte) ((PARENT_ID[4] << 4) | ROLLER_TURN_ID);
//			if(bCalibratedMotor[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1] <= maxArmMotorPos[PARENT_ID[4]][ROLLER_TURN_ID-1]-multiSpeed) 
//					currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1] += multiSpeed;
//				else currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1] = maxArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1];
//				
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte) 0;
//				this.packet[3] = (byte) multiSpeed;
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1].directionTA.setText(DIR_LEFT);
//			this.feedbackPanel[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1].limitSwitchLight.setColor(1);
//		} else if(bMotorERollerTurn) { // two(2) is turn rollers to push cord down
//			this.packet[0] = (byte) ((PARENT_ID[4] << 4) | ROLLER_TURN_ID);
//			if(bCalibratedMotor[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1]) {//validated, use SET_POSITION
//				if(currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1] >= multiSpeed) 
//					currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1] -= multiSpeed;
//				else currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1] = 0;
//				this.packet[1] = SET_POSITION_COMMAND;
//				this.packet[2] = (byte)((int)currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1] >> 8);//get leftX[15:8]
//				this.packet[3] = (byte)((int)currArmMotorPos[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1]);	 //get leftX[7:0]
//			} else {//Invalidated, use SET_SPEED
//				this.packet[1] = SET_SPEED_COMMAND;
//				this.packet[2] = (byte) 0;
//				this.packet[3] = (byte) -multiSpeed;
//			}
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1].directionTA.setText(DIR_RIGHT);
//			this.feedbackPanel[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1].limitSwitchLight.setColor(1);
//		} else if(!bCalibratedMotor[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1]) {//send wait command when button is let go
//			this.packet[0] = (byte) ((PARENT_ID[4] << 4) | ROLLER_TURN_ID);
//			this.packet[1] = SET_SPEED_COMMAND;
//			this.packet[2] = (byte) 0;
//			this.packet[3] = (byte) 0;
//			this.write(packet);
//			this.feedbackPanel[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1].directionTA.setText(DIR_STOP);
//			this.feedbackPanel[PARENT_ID[4]-PARENT_ID_OFFSET][ROLLER_TURN_ID-1].limitSwitchLight.setColor(0);
//		}
//		if(bButton[8]) {
//			if(multiSpeed > speedDisplay.getMinimum()) multiSpeed--;
//			speedDisplay.setValue(multiSpeed);
//			System.out.printf("multiSpeed:%d\n",multiSpeed);
//		}
//		if(bButton[9]) {
//			if(multiSpeed < speedDisplay.getMaximum()) multiSpeed++;
//			speedDisplay.setValue(multiSpeed);
//			System.out.printf("multiSpeed:%d\n",multiSpeed);
//		}
		//System.out.printf("X:%.4f Y:%.4f Z:%.4f DPADX:%.4f DPADY:%.4f\n", leftX,leftY, rightY, dPadX,dPadY);
//		System.out.printf("X:%b,%b Y:%b,%b, Z:%b,%b Yaw:%b,%b, Roll:%b,%b Pitch:%b,%b Grip:%b,%b Roller:%b,%b Turn:%b,%b\n",bMotorBX,bMotorEX, bMotorBY,bMotorEY, bMotorBZ,bMotorEZ, bMotorBYaw,bMotorEYaw, bMotorBRoll,bMotorERoll, 
//				bMotorBPitch,bMotorEPitch, bMotorBGripper,bMotorEGripper, bMotorBRoller,bMotorERoller, bMotorBRollerTurn,bMotorERollerTurn);	
	}//makeByteArray method
	
	//Phase 3: send the data to the OBC
	public void write(byte[] data)
	{	
		try {
			//System.err.println("SENDING PACKET: IP " + this.address + " PORT " + port + " DATA: " + data[0] + " " + data[1] + " " + data[2] + " " + data[3]);
			base.send(new DatagramPacket(data,4, this.address, this.port));
			//System.out.printf("[0]:%d [1]:%d [2]:%d [3]:%d\n", packet[0],packet[1],packet[2],packet[3]);
		} catch (IOException e) { }//e.printStackTrace(); }	
	}//write method
	
	@Override
	public void run() {
		//this.run_bool = true;
		
		while (true) { // main loop
            joy.poll(); //update joystick values
            try {	//*note running Phase 1 -> Phase 2 -> Phase 3 as well
				this.updateFieldsEx(joy); //Phase 1: change joystick values to rover control values
				try { 
					Thread.sleep(interval); //sleep thread, no need to run constantly
	            } catch(InterruptedException e) { e.printStackTrace(); }
			} catch (Exception e1) { e1.printStackTrace(); }
        }//while(true)
	}//run method
	
	public void setCalibration(int parentNum, int motorNum, int MaxPos) {
		maxArmMotorPos[parentNum][motorNum] = MaxPos;
		bCalibratedMotor[parentNum][motorNum] = true;
	}
	
	private int convertButton(int buttonID) {
		int ans = 0;
		int division=buttonID;
		  
		while(division > 0)
		{
			ans++;
			division = division / 2;
		}
		return ans;
	}
	//Sets the delay between each poll for joystick input
	public void setPollInterval(int pollMillis) {
	    this.interval = pollMillis;
	    joy.setPollInterval(pollMillis);
	}
	
	public void startPolling() { 
		roverthread = new Thread(this);
		roverthread.start(); 
	} 
	public void stopPolling() { roverthread.stop(); } //stop this thread
	
	//set the deadzone for the joystick controls
	public void setDeadZone(double deadZone) 
	{
	    joy.setDeadZone(deadZone);
	    this.updateDeadZone();
	}
	public void updateDeadZone() 
	{
		this.deadZone1 = joy.getDeadZone();
	}
	
	public static void main(String[] args) 
	{
		GamepadInterpret jc = null;
		try {
			jc = new GamepadInterpret(ARM_ADDRESS,ARM_PORT, null, null, new DatagramSocket(), null);
		} catch (SocketException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		jc.setDeadZone(0.04);
		jc.setPollInterval(10000);
		jc.updateDeadZone();
		
		jc.startPolling();
		//jc.stop();
		//jc.stopPolling();
		//jc.run_bool = false;
		//jc.startPolling();
		
	}
}
