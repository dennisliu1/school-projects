/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package cameraTabs;

import java.awt.Dimension;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextArea;
import javax.swing.JTextField;

/**
 * Camera setting panel. Allows to the user to enter in the camera's information,
 * which is used to stream the video to the screen.
 * To use, enter in the ip address and port/channel number, select the camera type,
 * and press start. Press stop to end the stream.
 * @author Dennis Liu
 * YURT 2010-2011
 * --------------------------------
 * Note that this is just the user interface, and not the actual code to stream video.
 * This is used by CameraDisplayPanel, which is where the video is shown.
 * --------------------------------
 * Hierarchy:
 *	CameraSetPanel
 *		fieldButtonsPanel (holds the label+fields) (Top)
 *			fieldSets[] (holds label+field together)
 *				fieldNames[] (label)
 *				fieldInfos[] (field)
 *		addCameraPanel (holds buttons and combobox) (Bottom)
 *			startCamB
 *			stopCamB
 *			camTypeBox
 * --------------------------------
 * Constants:
 * CAM_TYPE[] = { "USB", "IP" }		- types of cameras supported
 * FIELD_NUM = 2					- number of input fields.
 * --------------------------------
 */
public class CameraInitPanel extends JPanel {
	public static final String CAM_TYPE[] = { "IP", "USB" };
	public static final int FIELD_NUM = 3;
	public static final String START_STR = "start", STOP_STR = "stop";
	//top subpanel
	public JPanel topPanel;
	public JPanel fieldSets[];
	public JLabel fieldNames[];
	public JTextField fieldInfos[];
	//bottom subpanel
	public JPanel bottomPanel;
	public JButton startCamB;
	public JButton stopCamB;
	public JComboBox camTypeBox;

	public CameraInitPanel(String ip, String port, String cache, String type) {
		super();//constructor for JPanel
		this.setLayout( new BoxLayout(this, BoxLayout.PAGE_AXIS));//main panel has fields on top, and buttons+combobox below.
		
		topPanel = new JPanel();// initialize variables
		fieldSets = new JPanel[FIELD_NUM];
		fieldNames = new JLabel[FIELD_NUM];
		fieldInfos = new JTextField[FIELD_NUM];
		bottomPanel = new JPanel();
		startCamB = new JButton(START_STR);
		stopCamB = new JButton(STOP_STR);
		camTypeBox = new JComboBox(CAM_TYPE);
		
		topPanel.setLayout( new BoxLayout(topPanel, BoxLayout.LINE_AXIS));// panels are arranged: (label)[field]
		fieldNames[0] = new JLabel("Address"); //field names
		fieldNames[1] = new JLabel("port/ch #:");
		fieldNames[2] = new JLabel("cache:");
		for( int i = 0; i < FIELD_NUM; i++ ) {// add fields
			//make text field
			fieldInfos[i] = new JTextField();
			fieldInfos[i].setPreferredSize( new Dimension( 100, 20 ) );
			//put label+field together
			fieldSets[i] = new JPanel();
			fieldSets[i].add( fieldNames[i] );
			fieldSets[i].add( fieldInfos[i] );
			//add set to top subpanel
			topPanel.add( fieldSets[i] );
		}
		//add start/stop buttons and camera type
		//layout for bottom subpanel is horizontal
		bottomPanel.setLayout( new BoxLayout( bottomPanel, BoxLayout.LINE_AXIS));
		bottomPanel.add( startCamB );//add buttons
		bottomPanel.add( stopCamB );
		camTypeBox.setPreferredSize( new Dimension(50, 20));//set size of combobox
		bottomPanel.add( camTypeBox );//add combobox
		
		fieldInfos[0].setText(ip);
		fieldInfos[1].setText(port);
		fieldInfos[2].setText(cache);
		camTypeBox.setSelectedItem(type);
		
		//make full panel
		this.add( topPanel );
		this.add( bottomPanel );
	}//CameraSetPanel constructor
}//CameraSetPanel class
