/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package cameraTabs;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.JSplitPane;
import javax.swing.JTextArea;


/**
 * Main display panel. This split pane displays the interface for each tab of CameraTabPanel,
 * and is split into top/bottom sections. Top part is to display video, and bottom part is
 * for adding cameras to display.
 * @author Dennis Liu
 * YURT 2010-2011
 * --------------------------------
 * Note that this panel is mainly to integrate the CameraSetPanel and video toegether,
 * and does not actually have the stream video code.
 * This is called by CameraTabPanel for each tab.
 * --------------------------------
 * Hierarchy:
 *	CameraDisplayPanel (top/bottom)
 *		camPanel (Top panel)
 *			camAreas[numCams] (video)
 *		addPanel (Bottom panel)
 *			camSetPlace[numCams/2] (store 2 camSetPanels)
 *				camSetPanels[numCams]
 * --------------------------------
 * Variables:
 * numCams	- number of cameras.
 * --------------------------------
 */
public class CameraDisplayPanel extends JSplitPane {
	int numCams;
	private JPanel topPanel, bottomPanel, camSetPlace[];
	private Canvas camAreas[];
	private CameraInitPanel camInitPanels[];
	private VideoPlayer vPlayer[];
	private USBPlayer usbPlayer[];
	
	public CameraDisplayPanel( int num, String ip, String port) {
		//constructor for splitPane. Want top/bottom split.
		super( JSplitPane.VERTICAL_SPLIT, true );
		this.numCams = num;//save number of cameras
		if( (numCams%2 == 1) && (numCams != 1) ) numCams++;
		
		topPanel = new JPanel();//initialize panels
		bottomPanel = new JPanel();
		camSetPlace = new JPanel[numCams/2];
		camAreas = new Canvas[numCams];//make camera array
		camInitPanels = new CameraInitPanel[numCams]; //make cameraSetPanel array
		//make video canvas and matching cameraInitPanel (not added yet though)
		for( int i = 0; i < numCams; i++ ) {
			//top panel
			camAreas[i] = new Canvas(); //make camera
			//camAreas[i].setBackground(Color.black);
			topPanel.add( camAreas[i] ); //add camera to top panel
			//bottom panel
			camInitPanels[i] = new CameraInitPanel(ip, port);
		}
		if( numCams > 1) {//if > 1 camera, need to use camSetPlace panels
			//camSetPlace panels used so that display looks right
			for( int i = 0; i < numCams; i += 2) {
				camSetPlace[i/2] = new JPanel();//make subpanel
				camSetPlace[i/2].add(camInitPanels[i]);//add 2 camSetPanels to it
				camSetPlace[i/2].add(camInitPanels[i+1]);
				bottomPanel.add( camSetPlace[i/2] );//add subpanel
			}
			//since > 1 camera uses camSetPlace, then to line up camSetPlace panels
			//addPanel should use BoxLayout
			bottomPanel.setLayout( new BoxLayout(bottomPanel, BoxLayout.Y_AXIS));
			topPanel.setLayout( new GridLayout( numCams/2, 2 ));//currently, camPanel assumes either 1 or 4 cameras display.
		} else {//only one camera
			topPanel.setLayout( new GridLayout(1,1));
			bottomPanel.add( camInitPanels[0]);// only 1 camera, no need for complications.
		}// if( numcams > 1 ) else
		//-----------------------	add top/bottom panels together	-------------------------//
		this.setTopComponent(topPanel);//add panels together
		this.setBottomComponent(bottomPanel);
		this.setOneTouchExpandable(true);//make it so clicking split arrows shows 1 panel only.
		//-------------	add connect video control to canvas and initPanels	-----------------//
		vPlayer = new VideoPlayer[numCams];
		usbPlayer = new USBPlayer[numCams];
		
		for(int i =0; i<numCams; i++) {
			camInitPanels[i].startCamB.addActionListener(new CameraActionListener());
			camInitPanels[i].stopCamB.addActionListener(new CameraActionListener());
		}
	}// CameraPanel constructor
	
	public void stopAllVideo() {
		for(int i = 0; i < vPlayer.length; i++) {
			if(vPlayer[i] != null) vPlayer[i].stop();
			if(usbPlayer[i] != null) usbPlayer[i].closeConnection();
		}
	}//ftn stopAllVideo
	
	public class CameraActionListener implements ActionListener {

		@Override
		public void actionPerformed(ActionEvent arg0) {
			for(int i = 0; i < numCams; i++) {
				if( ((JButton)arg0.getSource()).getText().equals(CameraInitPanel.START_STR) ) {
					System.out.printf("Start ");
					if( ((String)camInitPanels[i].camTypeBox.getSelectedItem()).equals(CameraInitPanel.CAM_TYPE[0]) ) {//usb
						System.out.printf("USB ");
						if(vPlayer[i] == null) vPlayer[i] = new VideoPlayer(camAreas[i], camInitPanels[i]);
						vPlayer[i].actionPerformed(arg0);
						System.out.printf("^^\n");
					} else if( ((String)camInitPanels[i].camTypeBox.getSelectedItem()).equals(CameraInitPanel.CAM_TYPE[1]) ) {//ip
						System.out.printf("Stop ");
						if(usbPlayer[i] == null) usbPlayer[i] = new USBPlayer(camAreas[i], camInitPanels[i]);
						usbPlayer[i].actionPerformed(arg0);
						System.out.printf("^^\n");
					}
				} else if( ((JButton)arg0.getSource()).getText().equals(CameraInitPanel.STOP_STR) ){
					if( ((String)camInitPanels[i].camTypeBox.getSelectedItem()).equals(CameraInitPanel.CAM_TYPE[0]) ) {//usb
						vPlayer[i].actionPerformed(arg0);
					} else if( ((String)camInitPanels[i].camTypeBox.getSelectedItem()).equals(CameraInitPanel.CAM_TYPE[1]) ) {//ip
						usbPlayer[i].actionPerformed(arg0);
					}
				}
			}
		}//actionPerformed
		
	}
}//CameraPanel class
