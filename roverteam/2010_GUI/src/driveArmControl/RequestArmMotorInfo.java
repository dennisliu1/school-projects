package driveArmControl;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

import driveArmControl.DriveArmPanel.ArmMotorDisplay;

public class RequestArmMotorInfo implements Runnable {

	private final byte X_ID = 1, Y_ID = 2, Z_ID = 1, YAW_ID = 2, ROLL_ID = 1, PITCH_ID = 2, GRIPPER_ID = 1, ROLLER_ID = 2, ROLLER_TURN_ID = 1;
	private final byte PARENT_ID[] = {2, 3, 4, 5, 6};
	private final String CALIBRATED_MSG = "Done", NOT_CALIBRATED_MSG = "Cali", CALIBRATING_MSG = "Busy";
	
	private int port;
	
	private InetAddress address = null;
	public DatagramSocket sock = null;
	//private DatagramPacket speed_packet = null;
	public Thread requestThread;
	private byte[] setPacket = new byte[4];
	private final byte GET_SPEED_COMMAND = 0x61, GET_POSITION_COMMAND = 0x62, CALIBRATE_COMMAND = (byte)0xC0, GET_RANGE_COMMAND = 0x65;
	//private boolean run_bool;
	private int interval = 200;
	private boolean calibratedMotor[][] = new boolean[PARENT_ID.length][2];
	private ArmMotorDisplay[][] displayPanels;
	
	public RequestArmMotorInfo(String ip, int port, ArmMotorDisplay[][] displayPanels, DatagramSocket sock)
	{
		this.sock = sock;
		try {
			this.address = InetAddress.getByName(ip);
		} catch (UnknownHostException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		this.requestThread = new Thread(this);
		this.port = port;
		this.displayPanels = displayPanels;
		for(int i = 0; i < displayPanels.length; i++)
			for( int j = 0; j < displayPanels[0].length; j++)
				calibratedMotor[i][j] = displayPanels[i][j].bCalibrated;
	}//constructor
	
	@Override
	public void run() {
		//this.run_bool = true;
		while (true) {
			try {
				Thread.sleep(this.interval);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
//			this.setPacket[1] = GET_POSITION_COMMAND;
//			this.setPacket[2] = (byte) 0;
//			this.setPacket[3] = (byte) 0;
//			
//			if(calibratedMotor[PARENT_ID[0]][X_ID-1]) {
//				this.setPacket[0] = (byte) ((PARENT_ID[0] << 4) | X_ID);
//				this.write(this.setPacket);
//			}
//			if(calibratedMotor[PARENT_ID[0]][Y_ID-1]) {
//				this.setPacket[0] = (byte) ((PARENT_ID[0] << 4) | Y_ID);
//				this.write(this.setPacket);
//			}
//			if(calibratedMotor[PARENT_ID[1]][Z_ID-1]) {
//				this.setPacket[0] = (byte) ((PARENT_ID[1] << 4) | Z_ID);
//				this.write(this.setPacket);
//			}
//			if(calibratedMotor[PARENT_ID[1]][YAW_ID-1]) {
//				this.setPacket[0] = (byte) ((PARENT_ID[2] << 4) | YAW_ID);
//				this.write(this.setPacket);
//			}
//			if(calibratedMotor[PARENT_ID[2]][ROLL_ID-1]) {
//				this.setPacket[0] = (byte) ((PARENT_ID[2] << 4) | ROLL_ID);
//				this.write(this.setPacket);
//			}
//			if(calibratedMotor[PARENT_ID[2]][PITCH_ID-1]) {
//				this.setPacket[0] = (byte) ((PARENT_ID[3] << 4) | PITCH_ID);
//				this.write(this.setPacket);
//			}
//			if(calibratedMotor[PARENT_ID[3]][GRIPPER_ID-1]) {
//				this.setPacket[0] = (byte) ((PARENT_ID[3] << 4) | GRIPPER_ID);
//				this.write(this.setPacket);
//			}
//			if(calibratedMotor[PARENT_ID[3]][ROLLER_ID-1]) {
//				this.setPacket[0] = (byte) ((PARENT_ID[4] << 4) | ROLLER_ID);
//				this.write(this.setPacket);
//			}
//			if(calibratedMotor[PARENT_ID[4]][ROLLER_TURN_ID-1]) {
//				this.setPacket[0] = (byte) ((PARENT_ID[4] << 4) | ROLLER_TURN_ID);
//				this.write(this.setPacket);
//			}
			
			this.setPacket[1] = GET_RANGE_COMMAND;
			this.setPacket[2] = (byte) 0;
			this.setPacket[3] = (byte) 0;
			for(int i = 0; i < PARENT_ID.length; i++) {
				for(int j = 0; j < 2; j++) {
					if(i == PARENT_ID.length-1 && j==1) break;
					this.setPacket[0] = (byte) ((PARENT_ID[i] << 4) | j+X_ID);
					this.write(this.setPacket);
				}
			}
			
			
		}
	}//run method
	
	public void calibrateMotor(int parentNum, int motorNum) {
		this.setPacket[0] = (byte) ((parentNum << 4) | motorNum);
		this.setPacket[1] = CALIBRATE_COMMAND;
		this.setPacket[2] = (byte) 0;
		this.setPacket[3] = (byte) 0;
		this.write(this.setPacket);
		this.displayPanels[parentNum-PARENT_ID[0]][motorNum-1].cButton.setText(CALIBRATING_MSG);
		this.displayPanels[parentNum-PARENT_ID[0]][motorNum-1].bCalibrated = true;
		//System.out.printf("calibrating Motor %d (debug, packet not sent!)\n",motorNum);
	}//calibrateMotor method
	
	public void start() {
		this.requestThread = new Thread(this);
		this.requestThread.start();
	}
	
	public void stop() {
		this.requestThread.stop();
	}
	
	public void write(byte[] data)
	{
		try {
			this.sock.send(new DatagramPacket(data,4, this.address, this.port));
			//System.err.println(data[0] + " " + (byte)(char)data[1] + " " + data[2] + " " + data[3]);
		} catch (IOException e) {
					// TODO Auto-generated catch block
			e.printStackTrace();
		}
				
	}//write method
}//main class
