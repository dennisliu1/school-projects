package gui;

import gui.TabPanel.CamSettingsPanel;
import gui.component.Compass;
import gui.component.GuideLinesPanel;
import gui.component.LagMeter;
import gui.component.PowerBar;
import gui.component.RoverPicPanel;
import gui.component.SignalPanel;
import gui.component.SpeedBar;
import gui.component.TablePanel;
import gui.component.TiltSensor4;
import gui.component.TimerPanel;
import gui.testing.ArmPicPanel;
import gui.testing.LunabotPicPanel;
import gui.testing.Threader;
import gui.testing.WarningSystem;

import java.awt.Dimension;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.image.BufferedImage;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JLayeredPane;
import javax.swing.JPanel;

import comm.TCP.TCPClient;
import comm.UDP.UDPClient;

import zRovermodel.ClientTelemetry;
import zRovermodel.Component;
import zRovermodel.Robot;
import zRovermodel.SerializedComponent;
import zRovermodel.YURTRover;

/**
 * The main program for the GUI. starts the GUI, creates the frame and adds the transparent components and video.
 * @author Dennis
 */
public class OperatorGUI extends JPanel implements MouseListener {
//	public static final String ROVER_IP = "192.168.251.1";
	public static final String ROVER_IP = "localhost";
	public static final int GUI_DEBUG_MODE = 0;
	public static final int GUI_LUNABOTICS_MODE = 10;
	private static final int GUI_URC_MODE = 20;
	public static final int GUI_URC_CONSTRUCTION_MODE = 21;
	public static final int GUI_URC_SURVEYING_MODE = 22;
	public static final int GUI_URC_SCIENCE_MODE = 23;
	public static final int GUI_URC_ASTRONAUT_MODE = 24;
	
	public static final int DEFAULT_UNUSED_RESET_TIME = -1;
	public static final int DEFAULT_UNUSED_READ_TIME = -1;
	public static final int DEFAULT_RESET_TIME = 10000;
	public static final int DEFAULT_READ_TIME = 1000;
	
	public String defaultIP = "localhost";
	public String defaultPort = "4000";
	
	private int mode;
	private JLayeredPane layers;
	private int width, height;
	private OperatorGUI self = this;
//------------------------------            GUICOMPONENT VARIABLES AND DATA              ---------------------	
	private float trans;
	/** whether the panel is added */private ArrayList<Boolean> panelAdded;
	VideoPanel videoPanel = null;
		float videoPanelTrans;
	VideoPanel pipPanel = null;
		float pipPanelTrans;
	TimerPanel timerPanel = null;
		float timerPanelTrans;
		long[] time;
	Compass compassPanel = null;
		float compassPanelTrans;
		long[] degree;
	LagMeter lagPanel = null;
		float lagPanelTrans;
		int[] latency;
	TiltSensor4 tiltPanel = null;
		float tiltPanelTrans;
		double[] tiltX,tiltY;
	PowerBar powerPanel = null;
		float powerPanelTrans;
		double[] power;
	SpeedBar speedPanel = null;
		float speedPanelTrans;
		ArrayList<double[]> speeds;
	WarningSystem warningPanel1 = null;
		float warningPanel1Trans;
		boolean[] warningCond;
		JLabel warningLabel1;
		BufferedImage warningPic1;
	TablePanel tablePanel = null;
		float tablePanelTrans;
	TablePanel currentPanel = null;
		float currentPanelTrans;
		ArrayList<String> data;
		ArrayList<double[]> statuses, voltage,current;
	TabPanel tabPanel = null;
		float tabPanelTrans;
	RoverPicPanel roverPicPanel = null;
		float roverPicPanelTrans;
	LunabotPicPanel lunaPicPanel = null;
		float lunaPicPanelTrans;
		double[] bucket, conveyer;
	ArmPicPanel armPicPanel = null;
		float armPicPanelTrans;
		int[] armAngles;
		int[] canfields;
	GuideLinesPanel guideLinesPanel;
		float guideLinesPanelTrans;
	SignalPanel signalPanel;
		float signalPanelTrans;
		String[] signal;
	YURTRover rover;
//----------------------------------                                  ----------------------------------------//
	UDPClient client2;
	ClientTelemetry tele2;
	public GUIKeyListener keys;
	TCPClient client;
	CompassTelemetry compassTele;
	
	/**
	 * Constructs the GUI. 
	 * @param width
	 * @param height
	 * @param mode
	 */
	public OperatorGUI(int width, int height, int mode) {
		super();
		this.width = width;
		this.height = height;
		layers = new JLayeredPane();// create layers for GUI
		layers.setPreferredSize(new Dimension(width,height));// set size
		this.setPreferredSize(new Dimension(width,height));
		this.add(layers);
		keys = new GUIKeyListener();// Add key listener to the GUI; only for first focus though; needs to also go to JTabbedPane (where focus is usually in)
		rover = YURTRover.defaultRoverModel();// builds a rover model for the telemetry model to use
		
		ArrayList<SerializedComponent> roverComponents = new ArrayList<SerializedComponent>();
//		roverComponents.add(new YURTRover("Front21", 21, Component._dataType._int, "21"));
//		roverComponents.add(new YURTRover("Rear22", 22, Component._dataType._int, "22"));
//		roverComponents.add(new YURTRover("Luna23", 23, Component._dataType._int, "23"));
//		roverComponents.add(new YURTRover("Bckt110", 110, Component._dataType._int, "110"));
//		roverComponents.add(new YURTRover("Ctrl12", 12, Component._dataType._int, "12"));
		roverComponents.add(new YURTRover("Front22", 22, Component._dataType._int, "21"));
		roverComponents.add(new YURTRover("Rear23", 23, Component._dataType._int, "22"));
		roverComponents.add(new YURTRover("extr12", 12, Component._dataType._int, "23"));
		roverComponents.add(new YURTRover("extr14", 14, Component._dataType._int, "110"));
		roverComponents.add(new YURTRover("extr15", 15, Component._dataType._int, "12"));
		
		for(int i = 0; i < roverComponents.size(); i++) {
			//Motor 1
			roverComponents.get(i).addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "-1", Robot.READ_ONLY));// Device ID
			roverComponents.get(i).addComponent(new YURTRover("Power", 11, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
			roverComponents.get(i).addComponent(new YURTRover("Brake", 12, Component._dataType._int, "1", Robot.READ_WRITE, 0, 1));// [0,1]
			roverComponents.get(i).addComponent(new YURTRover("Position", 13, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
			roverComponents.get(i).addComponent(new YURTRover("Brake", 14, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
			roverComponents.get(i).addComponent(new YURTRover("Driver Vcc", 15, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
			roverComponents.get(i).addComponent(new YURTRover("Load Current", 16, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
			roverComponents.get(i).addComponent(new YURTRover("Timer", 17, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));// [0, 2^32-1]
			//Motor 2
			roverComponents.get(i).addComponent(new YURTRover("Power", 21, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
			roverComponents.get(i).addComponent(new YURTRover("Brake", 22, Component._dataType._int, "1", Robot.READ_WRITE, 0, 1));// [0,1]
			roverComponents.get(i).addComponent(new YURTRover("Position", 23, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
			roverComponents.get(i).addComponent(new YURTRover("Brake", 24, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
			roverComponents.get(i).addComponent(new YURTRover("Driver Vcc", 25, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
			roverComponents.get(i).addComponent(new YURTRover("Load Current", 26, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
			roverComponents.get(i).addComponent(new YURTRover("Timer", 27, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));// [0, 2^32-1]
			
			for(int j = 0; j < roverComponents.get(i).getComponents().size(); j++) {
				((SerializedComponent)roverComponents.get(i).getComponents().get(j)).setResetTimer(DEFAULT_RESET_TIME);
				((SerializedComponent)roverComponents.get(i).getComponents().get(j)).setReadTimer(DEFAULT_UNUSED_READ_TIME);
			}
			rover.addComponent(roverComponents.get(i));
		}
			
					
		((SerializedComponent)roverComponents.get(0).getComponents().get(0)).setResetTimer(DEFAULT_RESET_TIME);
		((SerializedComponent)roverComponents.get(0).getComponents().get(0)).setReadTimer(DEFAULT_READ_TIME);
		((SerializedComponent)roverComponents.get(1).getComponents().get(0)).setResetTimer(DEFAULT_RESET_TIME);
		((SerializedComponent)roverComponents.get(1).getComponents().get(0)).setReadTimer(DEFAULT_READ_TIME);
		((SerializedComponent)roverComponents.get(2).getComponents().get(0)).setResetTimer(DEFAULT_RESET_TIME);
		((SerializedComponent)roverComponents.get(2).getComponents().get(0)).setReadTimer(DEFAULT_READ_TIME);
		((SerializedComponent)roverComponents.get(3).getComponents().get(0)).setResetTimer(DEFAULT_RESET_TIME);
		((SerializedComponent)roverComponents.get(3).getComponents().get(0)).setReadTimer(DEFAULT_READ_TIME);
		((SerializedComponent)roverComponents.get(4).getComponents().get(0)).setResetTimer(DEFAULT_RESET_TIME);
		((SerializedComponent)roverComponents.get(4).getComponents().get(0)).setReadTimer(DEFAULT_READ_TIME);
		
		
		panelAdded = new ArrayList<Boolean>();
		for(int i = 0; i < 13; i++) panelAdded.add(false);// by default, no panel is added
		this.addMouseListener(this);
//----------------------        add GUIComponents        -------------------------//
		this.mode = mode;
		this.initSettings();
		
		this.setDefaultPanels();
		if(mode == GUI_LUNABOTICS_MODE || mode == GUI_DEBUG_MODE) this.addLunaBoticsPanels();
		if(((mode >= GUI_URC_MODE) && (mode <= GUI_URC_MODE+4)) || mode == GUI_DEBUG_MODE) this.addURCPanels();
		if(compassPanel != null) compassTele.setCompassPanel(compassPanel);
		if(tiltPanel != null) compassTele.setTiltPanel(tiltPanel);
	}

	/**
	 * Initializes component repaints (threadController), UDP communications and rover syncing (client2).
	 */
	public void initSettings() {
		//client2 = new UDPClient("192.168.80.111", 5005, 5005);//DEFAULT UDP TELEMETRY SETTINGS
		//client2 = new UDPClient("localhost", 5000,5001);//DEFAULT UDP TELEMETRY SETTINGS
		client2 = new UDPClient(ROVER_IP, 5000,5001);//DEFAULT UDP TELEMETRY SETTINGS
		//client2 = new UDPClient("192.168.254.1", 3840,3840);//DEFAULT UDP TELEMETRY SETTINGS
		tele2 = new ClientTelemetry(client2, rover);// control rover telemetry and UDP connections
		client = new TCPClient(ROVER_IP, 30150);
		trans = 0.8f;
	}
	/**
	 * Creates the default GUIComponents that will be used, regardless of what mode the rover is in.
	 * Currently, they are: speedPanel, PowerPanel, TablePanel, roverPicPanel.
	 * Each of these panels are given their telemetry containers, added to the Telemetry reader (clientTelemetry),
	 * and given a transparency value. Each panel should only be created and used once.
	 */
	public void setDefaultPanels() {
//------------------------            BUILD TELEMETRY HOLDERS          ------------------//
		time = new long[]{0};
		latency = new int[]{0};
		power = new double[]{0};
		armAngles = new int[]{0,0,0};
		canfields = new int[]{0,0,0};
		signal = new String[]{""};
		warningCond = new boolean[]{true};
		data = tele2.buildNames();
		statuses = tele2.buildStatus();
		voltage = tele2.buildVoltages();
		current = tele2.buildCurrents();
		speeds = new ArrayList<double[]>();
		for(int i = 0; i < 4; i++) speeds.add(new double[]{0});
		
		tiltX = new double[]{0};
		tiltY = new double[]{0};
		degree = new long[]{0};
		compassTele = new CompassTelemetry(client, degree, tiltX, tiltY);
//------------------------            TRANSPARENCY SETTINGS          ------------------//
		// invisible = 0f <= i <= 1f = no transparency
		videoPanelTrans = 1f;
		timerPanelTrans = trans;
		lagPanelTrans = trans;
		powerPanelTrans = trans;
		speedPanelTrans = trans;
		tablePanelTrans = trans;
		currentPanelTrans = trans;
		tabPanelTrans = trans;
		warningPanel1Trans = trans;
		roverPicPanelTrans = trans;
		armPicPanelTrans = trans;
		pipPanelTrans = trans;
		guideLinesPanelTrans = trans;
		signalPanelTrans = trans;
		
		videoPanel = new VideoPanel(0,TabPanel.DEFAULT_HEIGHT,width, height, videoPanelTrans);
		layers.add(videoPanel, JLayeredPane.DEFAULT_LAYER);
		panelAdded.set(0, true);
		
		pipPanel = new VideoPanel(width-240-80,TabPanel.DEFAULT_HEIGHT,240+80, 240, 0f);
		layers.add(pipPanel, JLayeredPane.PALETTE_LAYER, 0);
		
		tabPanel = new TabPanel(0,0,width, tabPanelTrans, videoPanel, client2, pipPanel, client);
		tabPanel.addKeyListener(keys);
		tabPanel.setDefaultIP(defaultIP);
		tabPanel.setDefaultPort(defaultPort);
		//tabPanel.makeOneCameraTab("abc","123");
		//tabPanel.setPipPanel("abc", "1234");
		
		layers.add(tabPanel, JLayeredPane.PALETTE_LAYER, 0);
		panelAdded.set(1, true);
		
		timerPanel = new TimerPanel(width-TimerPanel.DEFAULT_WIDTH-SignalPanel.DEFAULT_WIDTH,height-TimerPanel.DEFAULT_HEIGHT, timerPanelTrans, time);
		layers.add(timerPanel, JLayeredPane.PALETTE_LAYER, 1);
		panelAdded.set(2, true);
		
		tablePanel = new TablePanel(0,TabPanel.DEFAULT_HEIGHT,tablePanelTrans,data,statuses, voltage);
		layers.add(tablePanel, JLayeredPane.PALETTE_LAYER, 1);
		panelAdded.set(5, true);
		
		currentPanel = new TablePanel(0,TabPanel.DEFAULT_HEIGHT+100+30,currentPanelTrans,data,statuses, current);
		currentPanel.setTransparency(0);
		layers.add(currentPanel, JLayeredPane.PALETTE_LAYER, 1);
		panelAdded.set(6, true);
		
		speedPanel = new SpeedBar(width-SpeedBar.DEFAULT_WIDTH-TimerPanel.DEFAULT_WIDTH-SignalPanel.DEFAULT_WIDTH, height-SpeedBar.DEFAULT_HEIGHT, speedPanelTrans, speeds);
		layers.add(speedPanel, JLayeredPane.PALETTE_LAYER, 1);
		panelAdded.set(7, true);
		
		roverPicPanel = new RoverPicPanel(0, height-RoverPicPanel.DEFAULT_HEIGHT, roverPicPanelTrans, speeds);
		layers.add(roverPicPanel, JLayeredPane.PALETTE_LAYER, 1);
		panelAdded.set(8, true);
		
		guideLinesPanel = new GuideLinesPanel(0,0, width, height, 0);
		layers.add(guideLinesPanel, JLayeredPane.PALETTE_LAYER, 5);
		
		signalPanel = new SignalPanel(width-SignalPanel.DEFAULT_WIDTH,height-SignalPanel.DEFAULT_HEIGHT,signalPanelTrans, signal);
		layers.add(signalPanel, JLayeredPane.PALETTE_LAYER, 1);
//-----------------------------         BIND TELEMETRY UPDATERS        ----------------------------------------//
		//Speed telemetry
		tele2.addTelemetry(22,21, speeds.get(0));
		tele2.setTelemetryReadTime(22, 11, 1000);
		tele2.addTelemetry(23,21, speeds.get(1));
		tele2.setTelemetryReadTime(22, 21, 1000);
		tele2.addTelemetry(22,11, speeds.get(2));
		tele2.setTelemetryReadTime(23, 11, 1000);
		tele2.addTelemetry(23,11, speeds.get(3));
		tele2.setTelemetryReadTime(23, 21, 1000);
		//Signal Strength Telemetry
		tele2.addSignalTelemetry(signal);
		tele2.setSignalComponent(signalPanel);
		//Timer Telemetry
		tele2.addTimeTelemetry(time);
		tele2.setTimerComponent(timerPanel);
		tele2.setOperatorGUI(this);
		tele2.setTableComponent(tablePanel);
		tele2.setTableTwoComponent(currentPanel);
		//	try {
		//	warningPic1 = ImageIO.read(new File("/home/dennis/workspace/eclipse/OperatorGUI/src/GUI/highTemp.jpg"));
		//	Image image = warningPic1.getScaledInstance(warningPic1.getWidth(), warningPic1.getHeight(), 0);
		//	//Image image = Toolkit.getDefaultToolkit().getImage(getClass().getResource("/home/dennis/workspace/eclipse/OperatorGUI/src/GUI/highTemp.jpg"));
		//	warningPanel1 = new WarningSystem(0,100,image.getWidth(null),image.getHeight(null),warningPanel1Trans, warningCond,image);
		//	threadController.addUpdateThreadFunction(warningPanel1.updateWarnings);
		//	layers.add( warningPanel1, JLayeredPane.PALETTE_LAYER, 1);
		//} catch (Exception ex) {System.out.println("img1 failed: "+ex.fillInStackTrace());}

	}
	/**
	 * Lunabotics components: tilt only.
	 */
	public void addLunaBoticsPanels() {
		if(tiltPanel == null) {
			tiltPanelTrans = trans;
			tiltPanel = new TiltSensor4(width-(int)TiltSensor4.MAX_WIDTH,tabPanel.height+200+(int)TiltSensor4.MAX_HEIGHT, tiltPanelTrans, 1,tiltX, tiltY);
		}
		if(!panelAdded.get(9)) {	
			layers.add(tiltPanel, JLayeredPane.PALETTE_LAYER, 1);
			panelAdded.set(9, true);
		}
//		if(lunaPicPanel == null) {
//			bucket = new double[]{0};
//			conveyer = new double[]{0};
//			lunaPicPanelTrans = trans;
//			lunaPicPanel = new LunabotPicPanel(RoverPicPanel.DEFAULT_WIDTH,height-LunabotPicPanel.DEFAULT_HEIGHT, lunaPicPanelTrans, bucket, conveyer);
//		}
//		if(!panelAdded.get(10)) {
//			threadController.addUpdateThreadFunction(lunaPicPanel.updateSpeed);
//			layers.add(lunaPicPanel, JLayeredPane.PALETTE_LAYER, 1);
//			panelAdded.set(10, true);
//		}
	}
	/**
	 * URC panels: tilt and compass.
	 */
	public void addURCPanels() {
		if(compassPanel == null) {
//			degree = new long[]{0};
			compassPanelTrans = trans;
			compassPanel = new Compass(width/2-Compass.DEFAULT_WIDTH/2,TabPanel.DEFAULT_HEIGHT, compassPanelTrans, degree);
		}
		if(!panelAdded.get(10)) {
			layers.add(compassPanel, JLayeredPane.PALETTE_LAYER, 1);
			panelAdded.set(10, true);
		}
		if(tiltPanel == null) {
//			tiltX = new double[]{0};
//			tiltY = new double[]{0};
			tiltPanelTrans = trans;
			tiltPanel = new TiltSensor4(width-(int)TiltSensor4.MAX_WIDTH, tabPanel.height+200+(int)TiltSensor4.MAX_HEIGHT, tiltPanelTrans, 1,tiltX, tiltY);
		}
		if(!panelAdded.get(9)) {
			layers.add(tiltPanel, JLayeredPane.PALETTE_LAYER, 1);
			panelAdded.set(9, true);
		}
	}

	public void moveLeft() {
		if(tabPanel != null) {
			//no idea why this works; appears to always be true
			if(tabPanel.getSelectedIndex() >= 0 && tabPanel.getSelectedIndex() <= tabPanel.getTabCount())
				tabPanel.setSelectedIndex(tabPanel.getSelectedIndex()-1);
//				tabPanel.changeTab();
		}
	}
	public void moveRight() {
		if(tabPanel != null) {
			//no idea why this works; appears to always be true
			if(tabPanel.getSelectedIndex() >= 0 && tabPanel.getSelectedIndex() <= tabPanel.getTabCount()-2) {
				tabPanel.setSelectedIndex(tabPanel.getSelectedIndex()+1);
//				tabPanel.changeTab();
			}
		}
	}
	public void startVideo() {
		if(tabPanel != null && videoPanel != null)
			if(tabPanel.getSelectedIndex() >= 5 && tabPanel.getSelectedIndex() <= tabPanel.getComponentCount())
				((CamSettingsPanel)tabPanel.curComponent).initPanels[0].playButton.doClick();
	}
	public void stopVideo() {
		if(tabPanel != null && videoPanel != null)
			if(tabPanel.getSelectedIndex() >= 5 && tabPanel.getSelectedIndex() <= tabPanel.getComponentCount())
				((CamSettingsPanel)tabPanel.curComponent).initPanels[0].stopButton.doClick();
	}
	
	/**
	 * Reads keyboard presses for turning on/off the components.
	 * Currently:
	 * 1-0,q,[,],;: turn on off a component
	 * z = start video (video 1 only)
	 * x = stop video (videoo 1 only)
	 * @author dennis
	 *
	 */
	public class GUIKeyListener implements KeyListener {
		
		@Override
		public void keyTyped(KeyEvent e) { 
			if(e.getKeyChar() == '`') {
				tabPanel.flipDrop();
			} else if(e.getKeyChar() == '1') {
				if(tablePanel != null) {
					if(tablePanel.getTransparency() == 0) tablePanel.setTransparency(tablePanelTrans);
					else tablePanel.setTransparency(0f);
					tablePanel.updateDisplay();
				}
				
			} else if(e.getKeyChar() == '2') {
				if(currentPanel != null) {
					if(currentPanel.getTransparency() == 0) currentPanel.setTransparency(currentPanelTrans);
					else currentPanel.setTransparency(0f);
					currentPanel.updateDisplay();
				}
				
			} else if(e.getKeyChar() == '3') {
				if(compassPanel != null) {
					if(compassPanel.getTransparency() == 0) compassPanel.setTransparency(compassPanelTrans);
					else compassPanel.setTransparency(0f);
					compassPanel.updateDisplay();
				}
				
			} else if(e.getKeyChar() == '4') {
				if(pipPanel != null) {
					if(pipPanel.getTransparency() == 0) pipPanel.setTransparency(pipPanelTrans);
					else pipPanel.setTransparency(0f);
					pipPanel.updateDisplay();
				}
				
			} else if(e.getKeyChar() == '5') {
				if(tiltPanel != null) {
					if(tiltPanel.getTransparency() == 0) tiltPanel.setTransparency(tiltPanelTrans);
					else tiltPanel.setTransparency(0f);
					tiltPanel.updateDisplay();
				}
				
			} else if(e.getKeyChar() == '6') {
				if(signalPanel != null) {
					if(signalPanel.getTransparency() == 0) signalPanel.setTransparency(signalPanelTrans);
					else signalPanel.setTransparency(0f);
					signalPanel.updateDisplay();
				}
//				if(lagPanel != null) {
//					if(lagPanel.getTransparency() == 0) lagPanel.setTransparency(lagPanelTrans);
//					else lagPanel.setTransparency(0f);
//					lagPanel.updateDisplay();
//				}
//				if(powerPanel != null) {
//					if(powerPanel.getTransparency() == 0) powerPanel.setTransparency(powerPanelTrans);
//					else powerPanel.setTransparency(0f);
//					powerPanel.updateDisplay();
//				}
				
			} else if(e.getKeyChar() == '7') {
				if(timerPanel != null) {
					if(timerPanel.getTransparency() == 0) timerPanel.setTransparency(timerPanelTrans);
					else timerPanel.setTransparency(0f);
					timerPanel.updateDisplay();
				}
				
			} else if(e.getKeyChar() == '8') {
				if(speedPanel != null) {
					if(speedPanel.getTransparency() == 0) speedPanel.setTransparency(speedPanelTrans);
					else speedPanel.setTransparency(0f);
					speedPanel.updateDisplay();
				}
				
			} else if(e.getKeyChar() == '9') {
				if(roverPicPanel != null) {
					if(roverPicPanel.getTransparency() == 0) roverPicPanel.setTransparency(roverPicPanelTrans);
					else roverPicPanel.setTransparency(0f);
					roverPicPanel.updateDisplay();
				}
				
			} else if(e.getKeyChar() == '0') {
				if(lunaPicPanel != null) {
					if(lunaPicPanel.getTransparency() == 0) lunaPicPanel.setTransparency(lunaPicPanelTrans);
					else lunaPicPanel.setTransparency(0f);
					lunaPicPanel.updateDisplay();
				}
				
			} else if(e.getKeyChar() == '.') {
				if(videoPanel != null) {
					if(videoPanel.getTransparency() == 0) videoPanel.setTransparency(videoPanelTrans);
					else videoPanel.setTransparency(0f);
					videoPanel.updateDisplay();
				}
				
			} else if(e.getKeyChar() == '[') {
				
			} else if(e.getKeyChar() == ']') {
				if(guideLinesPanel != null) {
					if(guideLinesPanel.getTransparency() == 0) guideLinesPanel.setTransparency(guideLinesPanelTrans);
					else guideLinesPanel.setTransparency(0f);
					guideLinesPanel.updateDisplay();
				}
			} else if(e.getKeyCode() == KeyEvent.VK_LEFT) {
				self.moveLeft();
			} else if(e.getKeyCode() == KeyEvent.VK_RIGHT) {
				self.moveRight();
			} else if(e.getKeyChar() == 'z') {
				self.startVideo();
			} else if(e.getKeyChar() == 'x') {
				self.stopVideo();
			}
		}

		@Override
		public void keyPressed(KeyEvent e) {
			
		}//end

		@Override
		public void keyReleased(KeyEvent e) {
			
		}
		
	}//constructor
	
	
//-------------------------------------------                           TESTING AND MAIN                           -------------------------------------------//
	/**
		* @param args the command line arguments
		*/
	public static void main(String[] args) {
		int width = 1024, height = 700;
		
		JFrame frame = new JFrame("Operator GUI");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			OperatorGUI gui = new OperatorGUI(width,height, OperatorGUI.GUI_DEBUG_MODE);
		frame.add(gui);
		frame.setSize(width,height+40);
		frame.addKeyListener(gui.keys);
		frame.setVisible(true);
	}

	@Override
	public void mouseClicked(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mousePressed(MouseEvent e) {
		// TODO Auto-generated method stub
		System.out.printf("X:%d Y:%d\n", e.getX(), e.getY());
	}

	@Override
	public void mouseReleased(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseEntered(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseExited(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}
}//OperatorGUI class