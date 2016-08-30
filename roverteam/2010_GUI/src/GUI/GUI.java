package GUI;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;

import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;

import cameraTabs.CameraTabbedPane;

import compassTilt.CompassTiltPanel;
import driveArmControl.DeployPanel;
import driveArmControl.DriveArmPanel;


/*
 * compass, tilt(two axis)
 * get rid of add button +
 * add tab tabs +
 * sliding bars for rover tilt sensor (horizontal, vertical, #s) +
 * add compass thingy (heading)
 * add gps info + waypoints to bottom section ?
 * need focuslistener so that when you change tabs, stop all cameras
 *
 *
 * add Camera:
 * ip address, port/channel #, start/stop buttons, show as many as needed ( max 4 cameras)
 *
 * ppasnani@gmail.com
 * drive rover:(two tabs) - done!
 * ip port, start/stop buttons, 4 motors
 * ip port, start/stop buttons, 9 motor positions
 * drive rover:(two tabs) - done!
 * drive rover:(two tabs) - done!
 *
 * obc task manager - done!
 *
 * - drop jlist +
 * - use # cameras with same # of camera +
 * - a way to add tabs +
 * - a way to delete tabs
 */

public class GUI extends JFrame {
	// GUI constants/counters
	public static final int MAX_TABS = 15;
	public static final String CAM_TYPE[] = { "USB", "IP" };
	public int guiMainWidth = 1280, guiMainHeight = 750, guiNumTabs = 1;

	// GUI main parts
	private BorderLayout GUILayout;
	private final CameraTabbedPane cameraTabs;
	private DriveArmPanel DAPanel;
	private CompassTiltPanel CTPanel;
	private JPanel OBCBoxPanel;
	private DeployPanel deployPanel;
	// GUI camera storage

	public GUI( int width, int height, boolean deployable, boolean debug) {
		super("Main Frame");
		GUILayout = new BorderLayout();
		this.setLayout( GUILayout );
		//this.guiMainWidth = width;
		//this.guiMainHeight = height;
		this.setSize(guiMainWidth, guiMainHeight);
		this.setDefaultCloseOperation( JFrame.EXIT_ON_CLOSE );
		//-************************** add tab panel *****************************/
		cameraTabs = new CameraTabbedPane(guiNumTabs);
		this.add( cameraTabs );
		//-******************* add top panel ****************************/
		JPanel northPanel = new JPanel();
		northPanel.setLayout( new BoxLayout(northPanel, BoxLayout.LINE_AXIS));
		DAPanel = new DriveArmPanel(DriveArmPanel.DEF_WIDTH, DriveArmPanel.DEF_HEIGHT, deployable);
		CTPanel = new CompassTiltPanel("192.168.3.1",30150, debug);
		northPanel.add(DAPanel);
		//northPanel.add(deployPanel);
		northPanel.add(CTPanel);
		this.add( northPanel, "North" );
		//-******************* add GPS panel ****************************/
		//GPSWPList = new GPSPanel();
		//this.add( GPSWPList, "South");
		//-********************** add obc box stuff panel ***********************/
		OBCBoxPanel = new JPanel();
		OBCBoxPanel.setBackground( Color.yellow );
		OBCBoxPanel.setPreferredSize( new Dimension( 150, 350));
		this.add( OBCBoxPanel, "West");
		//-*******************add the panels to the frame ***********************/
		//this.add( drivePanel, "North");
		this.repaint();
	}// main constructor
	//-************************ Action Listeners  *******************************/
	
	
	
	/************************	MAIN	*********************************/
	public static final int MAIN_GUI_WIDTH = 1280, MAIN_GUI_HEIGHT = 600;
	public static void main(String[] args) {
		GUI inter = new GUI(MAIN_GUI_WIDTH, MAIN_GUI_HEIGHT, false, false);
		inter.setVisible(true);
		//inter.setAlwaysOnTop(true);
	}
}//class GUI
