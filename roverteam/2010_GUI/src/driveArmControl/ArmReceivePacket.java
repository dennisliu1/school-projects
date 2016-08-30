package driveArmControl;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

import javax.swing.JTextArea;

public class ArmReceivePacket implements Runnable {
	
	private final byte X_ID = 1, Y_ID = 2, Z_ID = 1, YAW_ID = 2, ROLL_ID = 1, PITCH_ID = 2, GRIPPER_ID = 1, ROLLER_ID = 2, ROLLER_TURN_ID = 1;
	private final byte PARENT_ID[] = {2, 3, 4, 5, 6};
	private final int NUM_BUTTONS = 12, NUM_AXIS = 5, ARM_MOTOR_NUM = 9;
	private final String CALIBRATED_MSG = "Done", NOT_CALIBRATED_MSG = "Cali", CALIBRATING_MSG = "Busy";
	
	public int motorPosition[][] = new int[PARENT_ID.length][2], motorMaxPosition[][] = new int[PARENT_ID.length][2];

	//private DatagramSocket base;
	//private int port;
	private int receiveport;
	//private InetAddress address = null;
	public DatagramSocket sock = null;
	private DatagramPacket speed_packet = null;
	
	//private byte[] GET_SPEED = new byte[4];
	private final byte GET_SPEED_COMMAND = 0x61;
	private final byte GET_POSITION_COMMAND = 0x62, CALIBRATE_COMMAND = (byte)0xC0, GET_RANGE_COMMAND = 0x65;
	
	public Thread receiveThread;
	DriveArmPanel feedbackPanel;
	int numMotors;
	private int interval;
	
	byte[] receivedPackets;
	
	public ArmReceivePacket(DriveArmPanel driveArmPanel, int num_Motors, DatagramSocket sock) {
		this.speed_packet = new DatagramPacket(new byte[4], 4);
		this.numMotors = num_Motors;
		this.feedbackPanel = driveArmPanel;
		this.receiveThread = new Thread(this);
		
		for(int i = 0; i< motorPosition.length; i++) {
			for(int j = 0; j< motorPosition[0].length; j++) {
				motorPosition[i][j] = -1;
				motorMaxPosition[i][j] = -1;
			}
		}
		this.sock = sock;
	}//constructor
	//private boolean run_bool = false;
	@Override
	public void run() {
		// TODO Auto-generated method stub
		//Create a new socket to receive packets from OBC to get motor speeds
		try {
			//this.sock = new DatagramSocket(this.receiveport);
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		//run_bool = true;
		while (true)
		{
			try {
				Thread.sleep(this.interval);
			} catch (InterruptedException e1) {
				e1.printStackTrace();
			}
			try {
				System.out.println(this.sock.getLocalAddress() + "    " + this.sock.getLocalPort());
				this.sock.receive(this.speed_packet);
				System.out.printf("Received\n");
			} catch (IOException e) {
				e.printStackTrace();
			}
			
			receivedPackets = this.speed_packet.getData();
			
			if (receivedPackets[1] == this.GET_POSITION_COMMAND) {
				if(receivedPackets[0] == (PARENT_ID[0] << 4 | this.X_ID)) {
					this.motorPosition[PARENT_ID[0]][this.X_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					this.feedbackPanel.setXSliderPos(this.motorPosition[PARENT_ID[0]][this.X_ID-1]);
				} else if(receivedPackets[0] == (PARENT_ID[0] << 4 | this.Y_ID)) {
					this.motorPosition[PARENT_ID[0]][this.Y_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					this.feedbackPanel.setYSliderPos(this.motorPosition[PARENT_ID[0]][this.Y_ID-1]);
					
				} else if(receivedPackets[0] == (PARENT_ID[1] << 4 | this.Z_ID)) {
					this.motorPosition[PARENT_ID[1]][this.Z_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					//this.feedbackPanel.setXSliderPos(this.motorPosition[this.Z_ID-1]);
				} else if(receivedPackets[0] == (PARENT_ID[1] << 4 | this.YAW_ID)) {
					this.motorPosition[PARENT_ID[1]][this.YAW_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					//this.feedbackPanel.setXSliderPos(this.motorPosition[this.YAW_ID-2]);
					
				} else if(receivedPackets[0] == (PARENT_ID[2] << 4 | this.ROLL_ID)) {
					this.motorPosition[PARENT_ID[2]][this.ROLL_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					//this.feedbackPanel.setXSliderPos(this.motorPosition[this.ROLL_ID-2]);
				} else if(receivedPackets[0] == (PARENT_ID[2] << 4 | this.PITCH_ID)) {
					this.motorPosition[PARENT_ID[2]][this.PITCH_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					//this.feedbackPanel.setXSliderPos(this.motorPosition[this.PITCH_ID-2]);
					
				} else if(receivedPackets[0] == (PARENT_ID[3] << 4 | this.GRIPPER_ID)) { 
					this.motorPosition[PARENT_ID[3]][this.GRIPPER_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					//this.feedbackPanel.setXSliderPos(this.motorPosition[this.GRIPPER_ID-2]);
				} else if(receivedPackets[0] == (PARENT_ID[3] << 4 | this.ROLLER_ID)) {
					this.motorPosition[PARENT_ID[3]][this.ROLLER_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					//this.feedbackPanel.setXSliderPos(this.motorPosition[this.ROLLER_ID-2]);
					
				} else if(receivedPackets[0] == (PARENT_ID[4] << 4 | this.ROLLER_TURN_ID)) {
					this.motorPosition[PARENT_ID[4]][this.ROLLER_TURN_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					//this.feedbackPanel.setXSliderPos(this.motorPosition[this.ROLLER_TURN_ID-2]);
				}
				
			} else if (receivedPackets[1] == this.GET_RANGE_COMMAND) {
				if(receivedPackets[0] == (PARENT_ID[0] << 4 | this.X_ID)) {
					this.motorMaxPosition[PARENT_ID[0]][X_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[0]][X_ID-1].cButton.setText(CALIBRATED_MSG);
					//this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[0]][X_ID-1].bCalibrated = true;
					this.feedbackPanel.armControlThread.setCalibration(this.PARENT_ID[0], this.X_ID, this.motorMaxPosition[PARENT_ID[0]][X_ID-1]);
//					if(this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[0]][Y_ID-1].bCalibrated) {
//						this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[0]][Y_ID-1].cButton.setText(CALIBRATED_MSG);
//					} else this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[0]][Y_ID-1].cButton.setText(NOT_CALIBRATED_MSG);
					this.feedbackPanel.XSliderMaxTick = this.motorMaxPosition[PARENT_ID[0]][X_ID-1];
					this.feedbackPanel.setXSliderMaxPos(this.feedbackPanel.XSliderMaxTick);
					this.feedbackPanel.setXSliderPos(0);
				} else if(receivedPackets[0] == (PARENT_ID[0] << 4 | this.Y_ID)) {
					this.motorMaxPosition[PARENT_ID[0]][Y_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[0]][Y_ID-1].cButton.setText(CALIBRATED_MSG);
					//this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[0]][Y_ID-1].bCalibrated = true;
					this.feedbackPanel.armControlThread.setCalibration(this.PARENT_ID[0], this.Y_ID, this.motorMaxPosition[PARENT_ID[0]][Y_ID-1]);
//					if(this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[0]][X_ID-1].bCalibrated) {
//						this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[0]][X_ID-1].cButton.setText(CALIBRATED_MSG);
//					} else this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[0]][X_ID-1].cButton.setText(NOT_CALIBRATED_MSG);
					this.feedbackPanel.YSliderMaxTick = this.motorMaxPosition[PARENT_ID[0]][Y_ID-1];
					this.feedbackPanel.setYSliderMaxPos(this.feedbackPanel.YSliderMaxTick);
					this.feedbackPanel.setYSliderPos(0);
					
				} else if(receivedPackets[0] == (PARENT_ID[1] << 4 | this.Z_ID)) {
					this.motorMaxPosition[PARENT_ID[1]][Z_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[1]][Z_ID-1].cButton.setText(CALIBRATED_MSG);
					this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[1]][Z_ID-1].bCalibrated = true;
					this.feedbackPanel.armControlThread.setCalibration(PARENT_ID[1], this.Z_ID, this.motorMaxPosition[PARENT_ID[1]][Z_ID-1]);
				} else if(receivedPackets[0] == (PARENT_ID[1] << 4 | this.YAW_ID)) {
					this.motorMaxPosition[PARENT_ID[1]][YAW_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[1]][YAW_ID-1].cButton.setText(CALIBRATED_MSG);
					this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[1]][YAW_ID-1].bCalibrated = true;
					this.feedbackPanel.armControlThread.setCalibration(PARENT_ID[1],this.YAW_ID, this.motorMaxPosition[PARENT_ID[1]][YAW_ID-1]);
//					if(this.feedbackPanel.armMotorFeedbackPanel[5].bCalibrated) {
//						this.feedbackPanel.armMotorFeedbackPanel[5].cButton.setText(CALIBRATED_MSG);
//					} else this.feedbackPanel.armMotorFeedbackPanel[5].cButton.setText(NOT_CALIBRATED_MSG);
					
				} else if(receivedPackets[0] == (PARENT_ID[2] << 4 | this.ROLL_ID)) {
					this.motorMaxPosition[PARENT_ID[2]][ROLL_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[2]][ROLL_ID-1].cButton.setText(CALIBRATED_MSG);
					this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[2]][ROLL_ID-1].bCalibrated = true;
					this.feedbackPanel.armControlThread.setCalibration(PARENT_ID[2],this.ROLL_ID, this.motorMaxPosition[PARENT_ID[2]][ROLL_ID-1]);
//					if(this.feedbackPanel.armMotorFeedbackPanel[YAW_ID-1].bCalibrated) {
//						this.feedbackPanel.armMotorFeedbackPanel[YAW_ID-1].cButton.setText(CALIBRATED_MSG);
//					} else this.feedbackPanel.armMotorFeedbackPanel[YAW_ID-1].cButton.setText(NOT_CALIBRATED_MSG);
				} else if(receivedPackets[0] == (PARENT_ID[2] << 4 | this.PITCH_ID)) {
					this.motorMaxPosition[PARENT_ID[2]][PITCH_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[2]][PITCH_ID-1].cButton.setText(CALIBRATED_MSG);
					this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[2]][PITCH_ID-1].bCalibrated = true;
					this.feedbackPanel.armControlThread.setCalibration(PARENT_ID[2], this.PITCH_ID, this.motorMaxPosition[PARENT_ID[2]][PITCH_ID-1]);
//					if(this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[2]][GRIPPER_ID-1].bCalibrated) {
//						this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[2]][GRIPPER_ID-1].cButton.setText(CALIBRATED_MSG);
//					} else this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[2]][GRIPPER_ID-1].cButton.setText(NOT_CALIBRATED_MSG);
					
				} else if(receivedPackets[0] == (PARENT_ID[3] << 4 | this.GRIPPER_ID)) {
					this.motorMaxPosition[PARENT_ID[3]][GRIPPER_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[3]][GRIPPER_ID-1].cButton.setText(CALIBRATED_MSG);
					this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[3]][GRIPPER_ID-1].bCalibrated = true;
					this.feedbackPanel.armControlThread.setCalibration(PARENT_ID[3], this.GRIPPER_ID, this.motorMaxPosition[PARENT_ID[3]][GRIPPER_ID-1]);
//					if(this.feedbackPanel.armMotorFeedbackPanel[PITCH_ID-1].bCalibrated) {
//						this.feedbackPanel.armMotorFeedbackPanel[PITCH_ID-1].cButton.setText(CALIBRATED_MSG);
//					} else this.feedbackPanel.armMotorFeedbackPanel[PITCH_ID-1].cButton.setText(NOT_CALIBRATED_MSG);
				} else if(receivedPackets[0] == (PARENT_ID[4] << 4 | this.ROLLER_ID)) {
					this.motorMaxPosition[PARENT_ID[3]][ROLLER_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
					this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[3]][ROLLER_ID-1].cButton.setText(CALIBRATED_MSG);
					this.feedbackPanel.armMotorFeedbackPanel[PARENT_ID[3]][ROLLER_ID-1].bCalibrated = true;
					this.feedbackPanel.armControlThread.setCalibration(PARENT_ID[3], this.ROLLER_ID, this.motorMaxPosition[PARENT_ID[3]][ROLLER_ID-1]);
					
				} else if(receivedPackets[0] == (PARENT_ID[4] << 4 | this.ROLLER_TURN_ID)) {//isn't actually calibrated, but just in case
					this.motorMaxPosition[PARENT_ID[4]][ROLLER_TURN_ID-1] = (int) (receivedPackets[2] << 8 | receivedPackets[3]);
				}
			}//calibrate settings
			//Setting the text of the text areas
			
		}//while(true)
	}//run method
	
	public void start() {
		this.receiveThread = new Thread(this);
		this.receiveThread.start();
	}
	
	public void stop() {
		this.receiveThread.stop();
	}
	
	public void setPollInterval(int pollMillis) {
	      this.interval = pollMillis;
	      //joy.setPollInterval(pollMillis);
	}
}//class
