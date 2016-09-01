package GUI;

/*

 */

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

import zRovermodel.ClientTelemetry;
import zRovermodel.YURTRover;
import Comm.TCP.TCPClient;
import Comm.UDP.UDPClient;
import GUI.TabPanel.CamSettingsPanel;

/**
 * Test
 * sudo gst-launch rtspsrc latency=0 location=rtsp://192.168.80.100:9400/stream ! decodebin ! xvimagesink sync=false
 * 
 * Glen
 * sudo gst-launch rtspsrc latency=0 location=rtsp://192.168.80.100:9400/stream ! decodebin ! xvimagesink sync=false
 * @author Dennis
 */
public class OperatorGUI extends JPanel implements MouseListener {
	public static final int GUI_DEBUG_MODE = 0;
	public static final int GUI_LUNABOTICS_MODE = 10;
	private static final int GUI_URC_MODE = 20;
	public static final int GUI_URC_CONSTRUCTION_MODE = 21;
	public static final int GUI_URC_SURVEYING_MODE = 22;
	public static final int GUI_URC_SCIENCE_MODE = 23;
	public static final int GUI_URC_ASTRONAUT_MODE = 24;
	private int mode;
	private JLayeredPane layers;
	private int width, height;
//------------------------------            GUICOMPONENT VARIABLES AND DATA              ---------------------	
	private float trans;
	private ArrayList<Boolean> panelAdded;
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
	Speed speedPanel = null;
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
	YURTRover rover;
//----------------------------------                                  ----------------------------------------//
	Threader threadController;
	UDPClient client2;
	ClientTelemetry tele2;
	public GUIKeyListener keys;
	TCPClient client;
	CompassTelemetry compassTele;
		
	public OperatorGUI(int width, int height, int mode) {
		super();
		this.width = width;
		this.height = height;
		layers = new JLayeredPane();// create layers for GUI
		layers.setPreferredSize(new Dimension(width,height));// set size
		this.setPreferredSize(new Dimension(width,height));
		this.add(layers);
		keys = new GUIKeyListener();//Add key listener to the GUI; only for first focus though; needs to also go to JTabbedPane (where focus is usually in)
		rover = YURTRover.buildRoverModel();
		panelAdded = new ArrayList<Boolean>();
		for(int i = 0; i < 13; i++) panelAdded.add(false);// by default, no panel is added
		this.addMouseListener(this);
//----------------------            init Variables            -------------------------//
		//tele2.addTelemetry(12, 11, speed1);
		//tele2.addTelemetry(12, 14, speed1);
		// TestThread t = new TestThread(time, degree, latency, tiltX,tiltY,power,speed,bucket, conveyer, armAngles, canfields, warningCond,statuses);
		// t.start();
//----------------------        add GUIComponents        -------------------------//
		
//		cameraPanel = new CameraTabbedPane(1, 0,0,width,height);
//		layers.add(cameraPanel, JLayeredPane.DEFAULT_LAYER);
//		cameraPanel.addKeyListener(keys);
		
		this.mode = mode;
		this.initSettings();
		guideLinesPanelTrans = trans;
		guideLinesPanel = new GuideLinesPanel(0,0, width, height, 0);
		threadController.addUpdateThreadFunction(guideLinesPanel.updateDisplayFunction);
		layers.add(guideLinesPanel, JLayeredPane.PALETTE_LAYER, 5);
		
		this.setDefaultPanels();
		if(mode == GUI_LUNABOTICS_MODE || mode == GUI_DEBUG_MODE) this.addLunaBoticsPanels();
		if(((mode >= GUI_URC_MODE) && (mode <= GUI_URC_MODE+4)) || mode == GUI_DEBUG_MODE) this.addURCPanels();
//		armPicPanel = new ArmPicPanel(0,height - RoverPicPanel.DEFAULT_HEIGHT - ArmPicPanel.DEFAULT_HEIGHT, armPicPanelTrans, armAngles, canfields);
//		threadController.addUpdateThreadFunction(armPicPanel.updateArmFunction);
//		layers.add(armPicPanel, JLayeredPane.PALETTE_LAYER, 1);

//		try {
//			warningPic1 = ImageIO.read(new File("/home/dennis/workspace/eclipse/OperatorGUI/src/GUI/highTemp.jpg"));
//			Image image = warningPic1.getScaledInstance(warningPic1.getWidth(), warningPic1.getHeight(), 0);
//			//Image image = Toolkit.getDefaultToolkit().getImage(getClass().getResource("/home/dennis/workspace/eclipse/OperatorGUI/src/GUI/highTemp.jpg"));
//			warningPanel1 = new WarningSystem(0,100,image.getWidth(null),image.getHeight(null),warningPanel1Trans, warningCond,image);
//			threadController.addUpdateThreadFunction(warningPanel1.updateWarnings);
//			layers.add( warningPanel1, JLayeredPane.PALETTE_LAYER, 1);
//		} catch (Exception ex) {System.out.println("img1 failed: "+ex.fillInStackTrace());}
		
		
		threadController.startUpdate();// start repainting functions of GUIComponents
		
	}

	/**
	 * Initializes component repaints (threadController), UDP communications and rover syncing (client2).
	 */
	public void initSettings() {
		threadController = new Threader();
		//client2 = new UDPClient("192.168.80.111", 5005, 5005);//DEFAULT UDP TELEMETRY SETTINGS
		//client2 = new UDPClient("localhost", 5000,5001);//DEFAULT UDP TELEMETRY SETTINGS
		client2 = new UDPClient("192.168.80.121", 3840,3840);//DEFAULT UDP TELEMETRY SETTINGS
		tele2 = new ClientTelemetry(client2, rover);// control rover telemetry and UDP connections
		client = new TCPClient("192.168.80.121", 30150);
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
		
		videoPanel = new VideoPanel(0,TabPanel.DEFAULT_HEIGHT,width, height, videoPanelTrans, "localhost", "4000");
		threadController.addUpdateThreadFunction(videoPanel.videoUpdateFunction);
		layers.add(videoPanel, JLayeredPane.DEFAULT_LAYER);
		panelAdded.set(0, true);
		
		pipPanel = new VideoPanel(0,TabPanel.DEFAULT_HEIGHT,240+80, 240, 0f, "localhost", "5001");
		threadController.addUpdateThreadFunction(pipPanel.videoUpdateFunction);
		layers.add(pipPanel, JLayeredPane.PALETTE_LAYER, 0);
		
		tabPanel = new TabPanel(0,0,width, tabPanelTrans, videoPanel, client2, pipPanel, client);
		tabPanel.addKeyListener(keys);
		//DEFAULT tab camera settings
//		tabPanel.makeOneCameraTab();
//		tabPanel.makeOneCameraTab();//tab 1
//		CamSettingsPanel abc = ((CamSettingsPanel)tabPanel.getComponentAt(tabPanel.getTabCount()-1));
//		abc.initPanels[0].inputs[0].setText("192.168.80.121");
//		abc.initPanels[0].inputs[1].setText("4000");
		
//		videoPanel.addVideoTab(4);
//		tabPanel.addTab("T4b "+(tabPanel.getTabCount()-1), tabPanel.addCameraTab(4));
//		CamSettingsPanel temp = (CamSettingsPanel)tabPanel.getComponentAt(tabPanel.getTabCount()-1);
//		temp.initPanels[0].inputs[0].setText("localhost"); //IP
//		temp.initPanels[0].inputs[0].setText("9400"); //Port
//		tabPanel.setSelectedIndex(tabPanel.getTabCount()-1);
		//DEFAULT tab camera settings
		layers.add(tabPanel, JLayeredPane.PALETTE_LAYER, 0);
		panelAdded.set(1, true);
		
//		timerPanel = new TimerPanel(width-TimerPanel.DEFAULT_WIDTH-PowerBar.DEFAULT_BATTERY_WIDTH,height-TimerPanel.DEFAULT_HEIGHT, timerPanelTrans, time);
//		threadController.addUpdateThreadFunction(timerPanel.updateTimer);
//		layers.add(timerPanel, JLayeredPane.PALETTE_LAYER, 1);
//		panelAdded.set(2, true);
//		
//		lagPanel = new LagMeter(width-LagMeter._width,height-LagMeter._height-PowerBar.DEFAULT_BATTERY_HEIGHT, lagPanelTrans, latency);
//		lagPanel.setLMBounds(0, 100, 350, 500);
//		threadController.addUpdateThreadFunction(lagPanel.muf);
//		layers.add(lagPanel, JLayeredPane.PALETTE_LAYER, 1);
//		panelAdded.set(3, true);
//		
//		powerPanel = new PowerBar(width-PowerBar.DEFAULT_BATTERY_WIDTH, height-PowerBar.DEFAULT_BATTERY_HEIGHT, powerPanelTrans, power);
//		threadController.addUpdateThreadFunction(powerPanel.powerBarUpdateFunction);
//		layers.add(powerPanel, JLayeredPane.PALETTE_LAYER, 1);
//		panelAdded.set(4, true);
		
		tablePanel = new TablePanel(0,TabPanel.DEFAULT_HEIGHT,tablePanelTrans,data,statuses, voltage);
		threadController.addUpdateThreadFunction(tablePanel.updater);
		layers.add(tablePanel, JLayeredPane.PALETTE_LAYER, 1);
		panelAdded.set(5, true);
		currentPanel = new TablePanel(0,TabPanel.DEFAULT_HEIGHT+100+20,currentPanelTrans,data,statuses, current);
		threadController.addUpdateThreadFunction(currentPanel.updater);
		layers.add(currentPanel, JLayeredPane.PALETTE_LAYER, 1);
		panelAdded.set(6, true);
		
//		tele2.addTelemetry(21,11, speeds.get(0));
//		tele2.addTelemetry(21,21, speeds.get(1));
//		tele2.addTelemetry(22,11, speeds.get(2));
//		tele2.addTelemetry(22,21, speeds.get(3));
		speedPanel = new Speed(width-Speed.DEFAULT_WIDTH-TimerPanel.DEFAULT_WIDTH-PowerBar.DEFAULT_BATTERY_WIDTH, height-Speed.DEFAULT_HEIGHT, speedPanelTrans, speeds);
		threadController.addUpdateThreadFunction(speedPanel.uSpeed);
		layers.add(speedPanel, JLayeredPane.PALETTE_LAYER, 1);
		panelAdded.set(7, true);
		
		roverPicPanel = new RoverPicPanel(0, height-RoverPicPanel.DEFAULT_HEIGHT, roverPicPanelTrans, speeds);
		threadController.addUpdateThreadFunction(roverPicPanel.updateSpeed);
		layers.add(roverPicPanel, JLayeredPane.PALETTE_LAYER, 1);
		panelAdded.set(8, true);
	}
	public void addLunaBoticsPanels() {
		if(tiltPanel == null) {
			tiltPanelTrans = trans;
			tiltPanel = new TiltSensor4(width-(int)TiltSensor4.MAX_WIDTH, height-200-(int)TiltSensor4.MAX_HEIGHT, tiltPanelTrans, 1,tiltX, tiltY);
		}
		if(!panelAdded.get(9)) {
			threadController.addUpdateThreadFunction(tiltPanel.tiltMeterUpdateFunction);
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
	public void addURCPanels() {
		if(compassPanel == null) {
//			degree = new long[]{0};
			compassPanelTrans = trans;
			compassPanel = new Compass(width/2-Compass.DEFAULT_WIDTH/2,TabPanel.DEFAULT_HEIGHT, compassPanelTrans, degree);
		}
		if(!panelAdded.get(10)) {
			threadController.addUpdateThreadFunction(compassPanel.updateCompass);
			layers.add(compassPanel, JLayeredPane.PALETTE_LAYER, 1);
			panelAdded.set(10, true);
		}
		if(tiltPanel == null) {
//			tiltX = new double[]{0};
//			tiltY = new double[]{0};
			tiltPanelTrans = trans;
			tiltPanel = new TiltSensor4(width-(int)TiltSensor4.MAX_WIDTH, height-200-(int)TiltSensor4.MAX_HEIGHT, tiltPanelTrans, 1,tiltX, tiltY);
		}
		if(!panelAdded.get(9)) {
			threadController.addUpdateThreadFunction(tiltPanel.tiltMeterUpdateFunction);
			layers.add(tiltPanel, JLayeredPane.PALETTE_LAYER, 1);
			panelAdded.set(9, true);
		}
	}

	public class GUIKeyListener implements KeyListener {
		public GUIKeyListener() {
			//System.out.println("keyed!");
		}
		
		@Override
		public void keyTyped(KeyEvent e) { }

		@Override
		public void keyPressed(KeyEvent e) {
			if(e.getKeyChar() == '`') {
				tabPanel.flipDrop();
			} else if(e.getKeyChar() == '1') {
				if(tablePanel != null)
				if(tablePanel.getTransparency() == 0) tablePanel.setTransparency(tablePanelTrans);
				else tablePanel.setTransparency(0f);
				
			} else if(e.getKeyChar() == 'q') {
				if(currentPanel != null)
				if(currentPanel.getTransparency() == 0) currentPanel.setTransparency(currentPanelTrans);
				else currentPanel.setTransparency(0f);
				
			} else if(e.getKeyChar() == '2') {
				if(compassPanel != null)
				if(compassPanel.getTransparency() == 0) compassPanel.setTransparency(compassPanelTrans);
				else compassPanel.setTransparency(0f);
			} else if(e.getKeyChar() == '3') {
				
			} else if(e.getKeyChar() == '4') {
				if(tiltPanel != null)
				if(tiltPanel.getTransparency() == 0) tiltPanel.setTransparency(tiltPanelTrans);
				else tiltPanel.setTransparency(0f);
			} else if(e.getKeyChar() == '5') {
				if(lagPanel != null)
				if(lagPanel.getTransparency() == 0) lagPanel.setTransparency(lagPanelTrans);
				else lagPanel.setTransparency(0f);
			} else if(e.getKeyChar() == '6') {
				if(powerPanel != null)
				if(powerPanel.getTransparency() == 0) powerPanel.setTransparency(powerPanelTrans);
				else powerPanel.setTransparency(0f);
			} else if(e.getKeyChar() == '7') {
				if(timerPanel != null)
				if(timerPanel.getTransparency() == 0) timerPanel.setTransparency(timerPanelTrans);
				else timerPanel.setTransparency(0f);
			} else if(e.getKeyChar() == '8') {
				if(speedPanel != null)
				if(speedPanel.getTransparency() == 0) speedPanel.setTransparency(speedPanelTrans);
				else speedPanel.setTransparency(0f);
			} else if(e.getKeyChar() == '9') {
				if(lunaPicPanel != null)
				if(lunaPicPanel.getTransparency() == 0) lunaPicPanel.setTransparency(lunaPicPanelTrans);
				else lunaPicPanel.setTransparency(0f);
			} else if(e.getKeyChar() == '0') {
				if(roverPicPanel != null)
				if(roverPicPanel.getTransparency() == 0) roverPicPanel.setTransparency(roverPicPanelTrans);
				else roverPicPanel.setTransparency(0f);
			} else if(e.getKeyChar() == '.') {
				if(videoPanel != null)
				if(videoPanel.getTransparency() == 0) videoPanel.setTransparency(videoPanelTrans);
				else videoPanel.setTransparency(0f);
			} else if(e.getKeyChar() == '[') {
				if(pipPanel != null)
				if(pipPanel.getTransparency() == 0) pipPanel.setTransparency(pipPanelTrans);
				else pipPanel.setTransparency(0f);
				//guideLinesPanelTrans
			} else if(e.getKeyChar() == ']') {
				if(guideLinesPanel != null)
				if(guideLinesPanel.getTransparency() == 0) guideLinesPanel.setTransparency(guideLinesPanelTrans);
				else guideLinesPanel.setTransparency(0f);
				//guideLinesPanelTrans
			} else if(e.getKeyCode() == KeyEvent.VK_LEFT) {
				if(tabPanel != null) {
					//no idea why this works; appears to always be true
					if(tabPanel.getSelectedIndex() >= 6+1 && tabPanel.getSelectedIndex() <= tabPanel.getTabCount())
						tabPanel.setSelectedIndex(tabPanel.getSelectedIndex());
//						tabPanel.changeTab();
				}
			} else if(e.getKeyCode() == KeyEvent.VK_RIGHT) {
				if(tabPanel != null) {
					//no idea why this works; appears to always be true
					if(tabPanel.getSelectedIndex() >= 6 && tabPanel.getSelectedIndex() <= tabPanel.getTabCount()-1) {
						tabPanel.setSelectedIndex(tabPanel.getSelectedIndex());
//						tabPanel.changeTab();
					}
				}
			} 
			else if(e.getKeyChar() == 'z') {
//				JButton button = new JButton();
//				button.doClick();
				if(tabPanel != null && videoPanel != null)
					if(tabPanel.getSelectedIndex() >= 5 && tabPanel.getSelectedIndex() <= tabPanel.getComponentCount())
						((CamSettingsPanel)tabPanel.curComponent).initPanels[0].playButton.doClick();
			}
			else if(e.getKeyChar() == 'x') {
//			JButton button = new JButton();
//			button.doClick();
			if(tabPanel != null && videoPanel != null)
				if(tabPanel.getSelectedIndex() >= 5 && tabPanel.getSelectedIndex() <= tabPanel.getComponentCount())
					((CamSettingsPanel)tabPanel.curComponent).initPanels[0].stopButton.doClick();
		}
		}//end

		@Override
		public void keyReleased(KeyEvent e) { }
	}
	
	
//-------------------------------------------                           TESTING AND MAIN                           -------------------------------------------//
	/**
		* @param args the command line arguments
		*/
	public static void main(String[] args) {
		int width = 1024, height = 700;
		
		JFrame frame = new JFrame("Operator GUI");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			OperatorGUI gui = new OperatorGUI(width,height, OperatorGUI.GUI_LUNABOTICS_MODE);
		frame.add(gui);
		frame.setSize(width,height+40);
		//frame.pack();
		frame.addKeyListener(gui.keys);
		frame.setVisible(true);
		
//		JFrame frameServer2 = new JFrame("UDP Client");
//			UDPClientGUI udp2 = new UDPClientGUI(gui.client2);
//		frameServer2.add(udp2);
//		frameServer2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//		frameServer2.pack();
//		frameServer2.setVisible(true);
		
//		JFrame frameServer3 = new JFrame("UDP Rover");
//		UDPClient client3 = new UDPClient("localhost", 4001, 4000);
//		UDPClientGUI udp3 = new UDPClientGUI(client3);
//		frameServer3.add(udp3);
//			RoverTelemetry rover = new RoverTelemetry(client3);
//		frameServer3.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//		frameServer3.pack();
//		frameServer3.setVisible(true);
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