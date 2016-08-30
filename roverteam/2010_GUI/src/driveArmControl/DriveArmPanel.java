/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package driveArmControl;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.net.DatagramSocket;
import java.net.SocketException;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JTabbedPane;
import javax.swing.JTextArea;

import JoystickDriveControl.JoystickInterpret;
import JoystickDriveControl.ReceivePackets;
import JoystickDriveControl.RequestMotorSpeed;

/**
 * drive rover:(two tabs)
 * ip port, start/stop buttons, 4 motors
 * ip port, start/stop buttons, 9 motor positions
 * @author Dennis Liu
 * YURT 2010-2011
 * --------------------------------
 * The drive and arm motor display panel. Allows one to connect to the rover's
 * motors and display their current settings, which are in an integer format.
 * There are two tabs, one for drive and one for arm.
 * To use, enter the ip port and press start. The motors will update accordingly.
 * --------------------------------
 * Hierarchy:
 *	DriveArmPanel
 *		drivePanel
 *			driveIPPanel (holds label+field)
 *				driveIP
 *				driveIPInput
 *			driveStartB (start/stop motor update)
 *			driveStopB
 *			driveMotorPanel[] (display motors (holds label+field))
 *				driveMotorLabel[]
 *				driveMotors[]
 *		armPanel
 *			armIPPanel (holds label+field)
 *				armIP
 *				armIPInput
 *			armStartB (start/stop motor update)
 *			armStopB
 *			armMotorPanel[] (display motors (holds label+field))
 *				armMotorLabel[]
 *				armMotors[]
 * --------------------------------
 * Constants:
 * DRIVE_MOTOR_NUM = 4	- number of drive motors
 * ARM_MOTOR_NUM = 9	- number of arm motors
 * FIELD_WIDTH = 100	- width of the fields
 * FIELD_HEIGHT = 20	- height of the fields
 * PANEL_WIDTH = 1000	- main panel's preferred width
 * PANEL_HEIGHT = 100	- main panel's preferred height
 * --------------------------------
 */
public class DriveArmPanel extends JTabbedPane {
	public static final int DRIVE_MOTOR_NUM = 4, ARM_MOTOR_NUM = 9,  FIELD_WIDTH = 100, FIELD_HEIGHT = 15, DEF_WIDTH = 900, DEF_HEIGHT = 100,
		ARM_MOTOR_HEIGHT = 15, DRIVE_IP_PORT = 30001, SLIDER_MAX_TICK = 126, SLIDER_SNAP_TICK = SLIDER_MAX_TICK/14;
	public static final String DRIVE_IP = "192.168.3.2";
	
	private final byte X_ID = 1, Y_ID = 2, Z_ID = 1, YAW_ID = 2, ROLL_ID = 1, PITCH_ID = 2, GRIPPER_ID = 1, ROLLER_ID = 2, ROLLER_TURN_ID = 1;
	private final byte PARENT_ID[] = {2, 3, 4, 5, 6};
	private final int NUM_BUTTONS = 12, NUM_AXIS = 5;
	private final String CALIBRATED_MSG = "Done", NOT_CALIBRATED_MSG = "Cali", CALIBRATING_MSG = "Busy";
	
	int panelWidth, panelHeight;
	JPanel driveIPPanel, drivePortPanel, armIPPanel, armPortPanel;
	JLabel driveIPLabel, drivePortLabel, armIPLabel, armPortLabel;
	JPanel driveMotorPanel[] = new JPanel[DRIVE_MOTOR_NUM], armMotorPanel[][] = new JPanel[PARENT_ID.length][2];
	JLabel[] driveMotorLabel = new JLabel[DRIVE_MOTOR_NUM];
	JTextArea driveIPInput,drivePortInput, armIPInput,armPortInput,
			driveMotorSpeed[] = new JTextArea[DRIVE_MOTOR_NUM],
			driveMotorCurrent[] = new JTextArea[DRIVE_MOTOR_NUM];
	ArmMotorDisplay armMotorFeedbackPanel[][] = new ArmMotorDisplay[PARENT_ID.length][2];
	JButton driveStartB, driveStopB, armStartB, armStopB;
	JPanel driveMainPanel,driveSecPanel,armMainPanel,armSecPanel;
	DriveArmPanel self = this;
	JPanel motorPanel;
	JSlider armMotorSpeedSlider, XSlider, YSlider;
	JPanel XSliderPanel, YSliderPanel;
	JTextArea XSliderTickNum, YSliderTickNum;
	int XSliderCurrTick=0, XSliderMaxTick=0, YSliderCurrTick=0, YSliderMaxTick=0;
	
	DatagramSocket driveSocket, armSocket;
	
	private final String DRIVE_IP_DEF = "192.168.254.1", DRIVE_PORT_DEF = "30001", ARM_IP_DEF = "192.168.254.1", ARM_PORT_DEF = "30002";
	//Drive joystick controller
	private String driveIP = "", drivePort = "", armIP = "", armPort = "";
	private JoystickInterpret driveControlThread =  null;
	private RequestMotorSpeed driveFeedback;
	private ReceivePackets rp;
	
	//Arm gamepad controller
	GamepadInterpret armControlThread;
	private RequestArmMotorInfo armFeedback;
	private ArmReceivePacket rp2;

	public DriveArmPanel(int width, int height, boolean deployable) {
		this.panelWidth = width;
		this.panelHeight = height;
		//this.setPreferredSize( new Dimension( panelWidth, panelHeight));
		// Drive panel
		driveMainPanel = new JPanel();
		driveMainPanel.setLayout( new FlowLayout());
			driveSecPanel = new JPanel();
			driveSecPanel.setLayout( new BoxLayout(driveSecPanel, BoxLayout.Y_AXIS));
			driveSecPanel.setPreferredSize(new Dimension(250, 50));
				//create ip input panel
				driveIPPanel = new JPanel();
				driveIPPanel.setLayout( new BoxLayout(driveIPPanel, BoxLayout.X_AXIS));
					driveIPLabel = new JLabel("Address:");//make label
					driveIPInput = new JTextArea(DRIVE_IP_DEF);//make field
					driveIPInput.setPreferredSize( new Dimension( FIELD_WIDTH, FIELD_HEIGHT));
					driveStartB = new JButton("start");//make buttons
					driveStartB.addActionListener( new DriveStartStopActionListener() );
				driveIPPanel.add(driveIPLabel);//add label+Field together
				driveIPPanel.add(driveIPInput);
				driveIPPanel.add(driveStartB);
			driveSecPanel.add( driveIPPanel );//add label+field and buttons together
				drivePortPanel = new JPanel();
				drivePortPanel.setLayout( new BoxLayout(drivePortPanel, BoxLayout.X_AXIS));
					drivePortLabel = new JLabel("Port:");//make label
					drivePortInput = new JTextArea(DRIVE_PORT_DEF);//make field
					drivePortInput.setPreferredSize( new Dimension( FIELD_WIDTH, FIELD_HEIGHT));
					driveStopB = new JButton("Stop");
					driveStopB.addActionListener( new DriveStartStopActionListener() );
				drivePortPanel.add(drivePortLabel);//add label+Field together
				drivePortPanel.add(drivePortInput);
				drivePortPanel.add(driveStopB);
			driveSecPanel.add( drivePortPanel );//add label+field and buttons together
		driveMainPanel.add(driveSecPanel);

		//add the motor displays
		//Box.createRigidArea( new Dimension(0,0));
		motorPanel = new JPanel();
		motorPanel.setLayout( new FlowLayout());
		JPanel subMotorPanel[] = new JPanel[2];
		subMotorPanel[0] = new JPanel();
		subMotorPanel[0].setLayout(new BoxLayout(subMotorPanel[0],BoxLayout.Y_AXIS));
		subMotorPanel[1] = new JPanel();
		subMotorPanel[1].setLayout(new BoxLayout(subMotorPanel[1],BoxLayout.Y_AXIS));
		for(int i = 0; i < DRIVE_MOTOR_NUM; i++ ) {
			driveMotorPanel[i] = new JPanel();//panel for each label+field
			driveMotorPanel[i].setLayout( new FlowLayout());
				driveMotorLabel[i] = new JLabel("M"+(i+1) );//make label
				driveMotorSpeed[i] = new JTextArea();//make field
				driveMotorSpeed[i].setPreferredSize( new Dimension( FIELD_WIDTH, FIELD_HEIGHT));
				driveMotorCurrent[i] = new JTextArea();//make field
				driveMotorCurrent[i].setPreferredSize( new Dimension( FIELD_WIDTH, FIELD_HEIGHT));
			JPanel vLabels = new JPanel(), vDisplay = new JPanel();
			vLabels.setLayout( new BoxLayout(vLabels, BoxLayout.Y_AXIS));
			//vLabels.add( Box.createRigidArea( new Dimension(1, 15)) );
			vLabels.add( new JLabel("Speed "+i));
			vLabels.add( Box.createRigidArea( new Dimension(1, 10)) );
			vLabels.add( new JLabel("Current "+i));
			vDisplay.setLayout( new BoxLayout(vDisplay, BoxLayout.Y_AXIS));
			//vDisplay.add( driveMotorLabel[i] );//make label+field together
			vDisplay.add( driveMotorSpeed[i] );
			vDisplay.add( Box.createRigidArea( new Dimension(1, 5)) );
			vDisplay.add( driveMotorCurrent[i] );
			if(i == 0 || i == 2) {
				driveMotorPanel[i].add( vLabels );
				driveMotorPanel[i].add( vDisplay );
				subMotorPanel[0].add(driveMotorPanel[i]);
			} else {
				driveMotorPanel[i].add( vDisplay );
				driveMotorPanel[i].add( vLabels );
				subMotorPanel[1].add(driveMotorPanel[i]);
			}
		}
		motorPanel.add( subMotorPanel[0] );// add label+field to main panel
		motorPanel.add( subMotorPanel[1] );// add label+field to main panel
		driveMainPanel.add(motorPanel);
		
		this.addTab( "Drive", driveMainPanel );//add drive panel (done)
		//--------------------------------------------------------------------------
		
		// Arm panel. see above.
		armMainPanel = new JPanel();
		armMainPanel.setLayout( new BoxLayout(armMainPanel, BoxLayout.X_AXIS));
			armSecPanel = new JPanel();
			armSecPanel.setLayout( new BoxLayout(armSecPanel, BoxLayout.Y_AXIS));
			armSecPanel.setPreferredSize(new Dimension(200, 50));
				armIPPanel = new JPanel();
				armIPPanel.setLayout(new FlowLayout());
					armIPLabel = new JLabel("Addr");
					armIPInput = new JTextArea(ARM_IP_DEF);
					armIPInput.setPreferredSize( new Dimension( FIELD_WIDTH, FIELD_HEIGHT));
					armStartB = new JButton("start");
					armStartB.addActionListener(new ArmStartStopActionListener());
				armIPPanel.add(armIPLabel);
				armIPPanel.add(armIPInput);
				armIPPanel.add(armStartB);
			armSecPanel.add( armIPPanel );
				armPortPanel = new JPanel();
				armPortPanel.setLayout(new FlowLayout());
					armPortLabel = new JLabel("Port");
					armPortInput = new JTextArea(ARM_PORT_DEF);
					armPortInput.setPreferredSize( new Dimension( FIELD_WIDTH, FIELD_HEIGHT));
					armStopB = new JButton("Stop");
					armStopB.addActionListener(new ArmStartStopActionListener());
				armPortPanel.add(armPortLabel);
				armPortPanel.add(armPortInput);
				armPortPanel.add(armStopB);
			armSecPanel.add( armPortPanel );
				armMotorSpeedSlider = new JSlider();
				armMotorSpeedSlider.setOrientation(JSlider.HORIZONTAL);
				armMotorSpeedSlider.setMinimum(0);
				armMotorSpeedSlider.setMaximum(SLIDER_MAX_TICK);
				armMotorSpeedSlider.setMajorTickSpacing(SLIDER_SNAP_TICK*2);
				armMotorSpeedSlider.setMinorTickSpacing(SLIDER_SNAP_TICK);
				armMotorSpeedSlider.setPaintTicks(true);
				armMotorSpeedSlider.setPaintLabels(true);
			armSecPanel.add(armMotorSpeedSlider);
		armMainPanel.add(armSecPanel);
		
		
		JPanel subArmPanel[] = new JPanel[4];
		for(int i = 0; i < subArmPanel.length; i++ ) {
			subArmPanel[i] = new JPanel();//init subArmPanel
			subArmPanel[i].setLayout(new BoxLayout(subArmPanel[i], BoxLayout.Y_AXIS));
		}
		
		String s = "";
		for(int i = 0; i < PARENT_ID.length; i++ ) {
			for(int j = 0; j < 2; j++) {
				if(deployable) {
					switch(i) {
					case(0): if(j == 0 )s = "X:"; else if(j==1) s = "Y:";; break;
					case(1): if(j == 0 )s = "Deploy 1:"; else if(j==1) s = "Deploy 2:"; break;
					case(2): if(j == 0 )s = "Deploy 3:"; else if(j==1) s = "Deploy 4:"; break;
					case(3): if(j==0) s = "Deploy 5:"; else if(j==1) s = "Rollers :"; break;
					case(4): s = "Spin Rollers:"; break;
					default: System.out.println("Error in Case!");
						break;
					}
				} else {
					switch(i) {
						case(0): if(j == 0 )s = "X:"; else if(j==1) s = "Y:";; break;
						case(1): if(j == 0 )s = "Z:"; else if(j==1) s = "Yaw L/R  :"; break;
						case(2): if(j == 0 )s = "Roll         :"; else if(j==1) s = "Pitch U/D:"; break;
						case(3): if(j==0) s = "Gripper :"; else if(j==1) s = "Rollers :"; break;
						case(4): s = "Spin Roller:"; break;
						default: System.out.println("Error in Case!");
							break;
					}
				}
				
				armMotorPanel[i][j] = new JPanel();
				//armMotorPanel[i].setSize( 150, ARM_MOTOR_HEIGHT);
				armMotorPanel[i][j].setLayout( new BoxLayout(armMotorPanel[i][j], BoxLayout.X_AXIS));
				armMotorPanel[i][j].add(Box.createRigidArea(new Dimension(1, 1)));
				armMotorFeedbackPanel[i][j] = new ArmMotorDisplay(s,50, ARM_MOTOR_HEIGHT);
				armMotorPanel[i][j].add(armMotorFeedbackPanel[i][j]);
				
				if(i == PARENT_ID.length-1 && j==1) break;
				if(i == 0 || (i==1 && j==0)) {// 0:1/2 + 1:1
					subArmPanel[0].add(armMotorPanel[i][j]); // 0 < i < 2 (0,1,2)
					subArmPanel[0].add(Box.createRigidArea(new Dimension(1, 3)));
				} else if( (i==1 && j==1) || i == 2 ) {// 1:2 + 2:1/2
					subArmPanel[1].add(armMotorPanel[i][j]); // 0 < i < 2 (0,1,2)
					subArmPanel[1].add(Box.createRigidArea(new Dimension(1, 3)));
				} else if( i == 3 || (i==4 && j==0) ) {// 3:1/2 + 4:1
					subArmPanel[2].add(armMotorPanel[i][j]); // 0 < i < 2 (0,1,2)
					subArmPanel[2].add(Box.createRigidArea(new Dimension(1, 3)));
				}
				
//				if(i < 3) {
//					subArmPanel[0].add(armMotorPanel[i][j]); // 0 < i < 2 (0,1,2)
//					subArmPanel[0].add(Box.createRigidArea(new Dimension(1, 3)));
//				}
//				else if(i < 6) {
//					subArmPanel[1].add(armMotorPanel[i][j]);// 2 < i < 6 (3,4,5)
//					subArmPanel[1].add(Box.createRigidArea(new Dimension(1, 3)));
//				}
//				else if(i < 9) {
//					subArmPanel[2].add(armMotorPanel[i][j]);// 6 < i < 7 (6)
//					subArmPanel[2].add(Box.createRigidArea(new Dimension(1, 3)));
//				}
			}
		}
		XSliderPanel = new JPanel();
			XSlider = new JSlider();
			XSlider.setOrientation(JSlider.HORIZONTAL);
			XSlider.setMinimum(0);
			XSliderPanel.add(XSlider);
			XSliderTickNum = new JTextArea(XSliderCurrTick+"/"+XSliderMaxTick);
			XSliderTickNum.setPreferredSize(new Dimension(100, 12));
			XSliderPanel.add(XSliderTickNum);
		subArmPanel[3].add(XSliderPanel);
		YSliderPanel = new JPanel();
			YSlider = new JSlider();
			YSlider.setOrientation(JSlider.HORIZONTAL);
			YSlider.setMinimum(0);
			YSliderPanel.add(YSlider);
			YSliderTickNum = new JTextArea(YSliderCurrTick+"/"+YSliderMaxTick);
			YSliderTickNum.setPreferredSize(new Dimension(100, 12));
			YSliderPanel.add(YSliderTickNum);
		subArmPanel[3].add(YSliderPanel);
		for(int i = 0; i < subArmPanel.length; i++ ) armMainPanel.add(subArmPanel[i]);
		this.addTab( "Arm", armMainPanel );
		
		Thread tabT = new JoyStickTabT();
		tabT.start();
	}//constructor
	
	public void setXSliderPos(int n) {
		XSlider.setValue(n);
		XSliderCurrTick = n;
		XSliderTickNum = new JTextArea(XSliderCurrTick+"/"+XSliderMaxTick);
	}
	public void setYSliderPos(int n) {
		YSlider.setValue(n);
		YSliderCurrTick = n;
		YSliderTickNum = new JTextArea(YSliderCurrTick+"/"+YSliderMaxTick);
	}
	public void setXSliderMaxPos(int n) {
		XSliderMaxTick = n;
		XSlider.setMaximum(n);
		XSlider.setMajorTickSpacing(n/10);
		XSlider.setMinorTickSpacing(n/20);
		XSlider.setPaintTicks(true);
		XSlider.setPaintLabels(true);
		setXSliderPos(0);
	}
	public void setYSliderMaxPos(int n) {
		YSliderMaxTick = n;
		YSlider.setMaximum(n);
		YSlider.setMajorTickSpacing(n/10);
		YSlider.setMinorTickSpacing(n/20);
		YSlider.setPaintTicks(true);
		YSlider.setPaintLabels(true);
		setYSliderPos(0);
	}
	
	
	// CALIBRATED_MSG = "Done", NOT_CALIBRATED_MSG = "Cali", CALIBRATING_MSG = "Busy"
	public class ValidateActionListener implements ActionListener {
		public void actionPerformed(ActionEvent e) {
			for(int i = 0; i < PARENT_ID.length; i++) {
				for(int j = 0; j < 2; j++) {
					if(i == PARENT_ID.length-1 && j==1) break;
					if(e.getSource() == armMotorFeedbackPanel[i][j].cButton && armMotorFeedbackPanel[i][j].cButton.getText().equals(NOT_CALIBRATED_MSG)) {
						//self.armMotorFeedbackPanel[i][j].cButton.setText(CALIBRATING_MSG);
						if(!self.armMotorFeedbackPanel[i][j].bCalibrated) self.armMotorFeedbackPanel[i][j].cButton.setText(CALIBRATING_MSG);
						self.armFeedback.calibrateMotor(i+PARENT_ID[0], j+1);
					}
				}//for 0-1
			}//for 0-5
			
			
			
			
//			if(e.getSource() == armMotorFeedbackPanel[0].cButton && armMotorFeedbackPanel[0].cButton.getText().equals(NOT_CALIBRATED_MSG)) {
//				self.armMotorFeedbackPanel[0].cButton.setText(CALIBRATING_MSG);
//				if(!self.armMotorFeedbackPanel[1].bCalibrated) self.armMotorFeedbackPanel[1].cButton.setText(CALIBRATING_MSG);
//				self.armFeedback.calibrateMotor(X_ID);
//				
//			} else if(e.getSource() == armMotorFeedbackPanel[1].cButton && armMotorFeedbackPanel[1].cButton.getText().equals(NOT_CALIBRATED_MSG) ) {
//				self.armMotorFeedbackPanel[1].cButton.setText(CALIBRATING_MSG);
//				if(!self.armMotorFeedbackPanel[0].bCalibrated) self.armMotorFeedbackPanel[0].cButton.setText(CALIBRATING_MSG);
//				self.armFeedback.calibrateMotor(Y_ID);
//				
//			} else if(e.getSource() == armMotorFeedbackPanel[2].cButton && armMotorFeedbackPanel[2].cButton.getText().equals(NOT_CALIBRATED_MSG) ) {
//				self.armMotorFeedbackPanel[2].cButton.setText(CALIBRATING_MSG);
//				self.armFeedback.calibrateMotor(Z_ID);
//				
//			} else if(e.getSource() == armMotorFeedbackPanel[3].cButton && armMotorFeedbackPanel[3].cButton.getText().equals(NOT_CALIBRATED_MSG) ) {
//				self.armMotorFeedbackPanel[3].cButton.setText(CALIBRATING_MSG);
//				if(!self.armMotorFeedbackPanel[5].bCalibrated) self.armMotorFeedbackPanel[5].cButton.setText(CALIBRATING_MSG);
//				self.armFeedback.calibrateMotor(YAW_ID);
//				
//			} else if(e.getSource() == armMotorFeedbackPanel[4].cButton && armMotorFeedbackPanel[4].cButton.getText().equals(NOT_CALIBRATED_MSG) ) {
//				self.armMotorFeedbackPanel[4].cButton.setText(CALIBRATING_MSG);
//				if(!self.armMotorFeedbackPanel[6].bCalibrated) self.armMotorFeedbackPanel[6].cButton.setText(CALIBRATING_MSG);
//				self.armFeedback.calibrateMotor(PITCH_ID);
//				
//			} else if(e.getSource() == armMotorFeedbackPanel[5].cButton && armMotorFeedbackPanel[5].cButton.getText().equals(NOT_CALIBRATED_MSG) ) {
//				self.armMotorFeedbackPanel[5].cButton.setText(CALIBRATING_MSG);
//				if(!self.armMotorFeedbackPanel[3].bCalibrated) self.armMotorFeedbackPanel[3].cButton.setText(CALIBRATING_MSG);
//				self.armFeedback.calibrateMotor(ROLL_ID);
//				
//			} else if(e.getSource() == armMotorFeedbackPanel[6].cButton && armMotorFeedbackPanel[6].cButton.getText().equals(NOT_CALIBRATED_MSG) ) {
//				self.armMotorFeedbackPanel[6].cButton.setText(CALIBRATING_MSG);
//				if(!self.armMotorFeedbackPanel[4].bCalibrated) self.armMotorFeedbackPanel[4].cButton.setText(CALIBRATING_MSG);
//				self.armFeedback.calibrateMotor(GRIPPER_ID);
//				
//			} else if(e.getSource() == armMotorFeedbackPanel[7].cButton && armMotorFeedbackPanel[7].cButton.getText().equals(NOT_CALIBRATED_MSG) ) {
//				self.armMotorFeedbackPanel[7].cButton.setText(CALIBRATING_MSG);
//				self.armFeedback.calibrateMotor(ROLLER_ID);
//			}
		}//actionPerformed method
	}//ValidateActionListener Class
	
	private class JoyStickTabT extends Thread {
		int curComponentNum = 0;
		JPanel curComponent = null;
		
		public JoyStickTabT() {
			curComponentNum = self.getSelectedIndex();
			curComponent = (JPanel) self.getSelectedComponent();
			try {
				driveSocket = new DatagramSocket();
				armSocket = new DatagramSocket();
			} catch (SocketException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			//driveControlThread = new JoystickInterpret(DRIVE_IP, DRIVE_IP_PORT);
			//driveControlThread.stop();
		}
		
		public void run() {
			while(true) {
				
				//System.out.printf("%s->%s\n", curComponent, self.getSelectedComponent());
				if(self.getSelectedComponent() == driveMainPanel) {
//					self.driveControlThread = new JoystickInterpret("localhost", 30001, driveSocket);
//					self.driveControlThread.setDeadZone(0.04);
//			        self.driveControlThread.setPollInterval(50);
//					self.driveFeedback = new RequestMotorSpeed("localhost", 30001,driveSocket);
//					self.rp = new ReceivePackets(30002, self.driveMotorSpeed, DriveArmPanel.DRIVE_MOTOR_NUM, self.driveMotorCurrent, driveSocket);
					if(self.armControlThread != null) {
						self.armControlThread.stopPolling();
						self.armFeedback.stop();
						self.rp2.stop();
					}
				} else if(self.getSelectedComponent() == armMainPanel) {
//					self.armControlThread = new GamepadInterpret(GamepadInterpret.ARM_ADDRESS,GamepadInterpret.ARM_PORT, armMotorSpeedSlider, armMotorFeedbackPanel, armSocket);
//					self.armControlThread.setDeadZone(0.04);
//			        self.armControlThread.setPollInterval(100);
//					self.armFeedback = new RequestArmMotorInfo("192.168.80.103", 30002, armMotorFeedbackPanel, armSocket);
//					self.rp2 = new ArmReceivePacket(30002, self, DriveArmPanel.DRIVE_MOTOR_NUM, armSocket);
					if(self.driveControlThread != null) {
						self.driveControlThread.stopPolling();
						self.driveFeedback.stop();
						self.rp.stop();
					}
				}
				
				try { Thread.sleep(100); } 
				catch (InterruptedException e) { e.printStackTrace(); }
			}
		}//run
	}
	
	public class DriveStartStopActionListener implements ActionListener {
		public void actionPerformed(ActionEvent e) {
			//two cases: start or stop button
			if (e.getActionCommand().equals(self.driveStartB.getText())) {
                if(self.driveControlThread == null) {
                	driveIP = driveIPInput.getText();
                	drivePort = drivePortInput.getText();

                	self.driveControlThread = new JoystickInterpret(driveIP, Integer.parseInt(drivePort), driveSocket);
					//self.driveControlThread.setDeadZone(0.04);
			        self.driveControlThread.setPollInterval(50);
					self.driveFeedback = new RequestMotorSpeed(driveIP, Integer.parseInt(drivePort),driveSocket);
					self.rp = new ReceivePackets(self.driveMotorSpeed, DriveArmPanel.DRIVE_MOTOR_NUM, self.driveMotorCurrent, driveSocket);
                }
				//self.driveControlThread.updateDeadZone();
                self.driveControlThread.startPolling();
                self.driveFeedback.start();
                self.rp.start();
			} else if (e.getActionCommand().equals(self.driveStopB.getText())) {
				//System.out.println("ho");
				if (self.driveControlThread != null && self.driveFeedback != null && self.rp != null) {
					self.driveControlThread.stopPolling();
					//self.driveControlThread.base.close();
					self.driveFeedback.stop();
					//self.driveFeedback.sock.close();
					self.rp.stop();
					//self.rp.sock.close();
				}
				//rc.stop();
			}
			//System.out.println(e.getActionCommand());
		}//actionPerformed method
	}//DriveActionListener Class
	public class ArmStartStopActionListener implements ActionListener {
		public void actionPerformed(ActionEvent e) {
			//two cases: start or stop button
			if (e.getActionCommand().equals(self.armStartB.getText())) {
                if(self.armControlThread == null) {
                	armIP = armIPInput.getText();
                	armPort = armPortInput.getText();
                	self.armControlThread = new GamepadInterpret(armIP, Integer.parseInt(armPort), armMotorSpeedSlider, armMotorFeedbackPanel, armSocket, self);
					self.armControlThread.setDeadZone(0.04);
			        self.armControlThread.setPollInterval(100);
					self.armFeedback = new RequestArmMotorInfo(armIP, Integer.parseInt(armPort), armMotorFeedbackPanel, armSocket);
					self.rp2 = new ArmReceivePacket(self, DriveArmPanel.DRIVE_MOTOR_NUM, armSocket);
                }
				self.armControlThread.updateDeadZone();
                self.armControlThread.startPolling();
                self.armFeedback.start();
                self.rp2.start();
			} else if (e.getActionCommand().equals(self.driveStopB.getText())) {
				//System.out.println("ho");
				if (self.armControlThread != null && self.armFeedback != null && self.rp2 != null) {
					self.armControlThread.stopPolling();
					self.armFeedback.stop();
					self.rp2.stop();
				}
				//rc.stop();
			}
			//System.out.println(e.getActionCommand());
		}//actionPerformed method
	}//DriveActionListener Class
	
	public class ArmMotorDisplay extends JPanel {
		public LightPanel limitSwitchLight;
		public JLabel label;
		public JTextArea directionTA;
		public int width, height;
		public JButton cButton;
		public boolean bCalibrated = false;
		
		public ArmMotorDisplay(String name, int width, int height) {
			this.width = width;
			this.height = height;
			//this.setSize(width, height);
			label = new JLabel(name);
			directionTA = new JTextArea();
			directionTA.setPreferredSize(new Dimension( (width/10), height));
			limitSwitchLight = new LightPanel(width, height);
			cButton = new JButton(NOT_CALIBRATED_MSG);
			cButton.addActionListener(new ValidateActionListener());
			
			this.setLayout(new BoxLayout(this, BoxLayout.X_AXIS));
			this.add(label);
			this.add(Box.createRigidArea(new Dimension( (width/10), height)));
			this.add(limitSwitchLight);
			this.add(Box.createRigidArea(new Dimension( (width/10), height)));
			this.add(directionTA);
			this.add(Box.createRigidArea(new Dimension( (width/10), height)));
			this.add(cButton);
		}
	}//ArmMotorDisplay class
	
//	public class LightPanel extends JPanel {
//		public int width, height;
//		public Color onColor, offColor, midColor, currColor;
//		private int currState;
//		
//		public LightPanel(int width, int height, Color onColor, Color midColor, Color offColor) {
//			this.width = width;
//			this.height = height;
//			this.onColor = onColor;
//			this.midColor = midColor;
//			this.offColor = offColor;
//			this.currColor = offColor;
//			this.setSize(new Dimension(width, height));
//		}
//		public LightPanel(int width, int height) {
//			this(width, height, Color.red, Color.yellow, Color.green);
//		}
//		public  boolean setColor(int num) {
//			if(num == 0) currColor = offColor;
//			else if(num == 1) currColor = midColor;
//			else if(num == 2) currColor = onColor;
//			else return false;
//			repaint();
//			return true;
//		}
//		
//		public void update(Graphics g) {
//			g.setColor(currColor);
//			for(int i = 0; i < width; i++ ) {
//				g.drawLine(0, i, height, i);
//			}
//		}
//		@Override
//		public void paint(Graphics g) {
//			update(g);
//		}
//	}//LightPanel class
}//drive arm motor class
