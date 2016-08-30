/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package gui;

import gui.VideoPanel.VideoActionListener;

import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTabbedPane;
import javax.swing.JTextArea;

import comm.TCP.TCPClient;
import comm.Test.UDPClientGUI;
import comm.UDP.UDPClient;


/**
 *
 * @author dennis
 */
public class TabPanel extends JTabbedPane {
	public static final int DEFAULT_HEIGHT = 20;   // panel height
	public static final int DEFAULT_DROP_HEIGHT = 80;
	public String defaultIP = "localhost";
	public String defaultPort = "4000";
	
	private TabPanel self = this;
	ArrayList<JLabel> tabLabels;
	CurrentTabT t;
	int x,y,width,height;
	VideoPanel videoPanel;
	UDPClientGUI clientGUI;
	TCPController TCPcontroller;
	
	VideoPanel pipPanel;
	CamSettingsPanel pipSettingsPanel;
	
	//For the tab control thread
	int curComponentNum = 0;
	int tabCounter;
	CamSettingsPanel curComponent;
	int tabChange = 0; //for camera port change
//---------------------------             CONSTRUCTORS             ---------------------------//
	public TabPanel(int x, int y, int width, float transparency, VideoPanel videoPanel, UDPClient clientGUI, VideoPanel pipPanel, TCPClient client) {
		this(x,y,width,DEFAULT_HEIGHT,transparency, videoPanel, clientGUI, pipPanel, client);
	}
	public TabPanel(int x, int y, int width, int height, float transparency, VideoPanel videoPanel, UDPClient clientGUI, VideoPanel pipPanel, TCPClient client) {
		tabLabels = new ArrayList<JLabel>();
		this.x = x;
		this.y = y;
		this.width = width;
		this.height= height;
		this.videoPanel = videoPanel;
		this.setTabLayoutPolicy( JTabbedPane.SCROLL_TAB_LAYOUT);
		this.setBounds(x, y, width, height);
		this.setPreferredSize(new Dimension(width, height));
		this.setOpaque(false);
		this.clientGUI = new UDPClientGUI(clientGUI);
		
		this.pipPanel = pipPanel;
		//videoPanel.addVideoTab(1);
		//self.addTab("Tab "+(self.getTabCount()-1), self.addCameraTab(1));
		//self.setSelectedIndex(self.getTabCount()-1);
		pipSettingsPanel = new CamSettingsPanel(0, pipPanel);
		//pipSettingsPanel.initPanels[0].playButton.removeActionListener(pipSettingsPanel.initPanels[0].initListener);
		TCPcontroller = new TCPController(client);
		
		tabLabels.add( new JLabel("Add 1"));
		tabLabels.add( new JLabel("Add 4"));
		this.addTab("Add 1", null);
		this.addTab("Add 4", null);
		this.addTab(" UDP ", this.clientGUI);
		//this.addTab("PiP", pipSettingsPanel);
		this.addTab("TCP", TCPcontroller);
		makePIPCameraTab();
		this.setSelectedIndex(3);
		//this.addTab("Tab "+(self.getTabCount()-1), self.addCameraTab(1));
		//this.setSelectedIndex(self.getTabCount()-1);
//		videoPanel.addVideoTab(1);
//		self.addTab("Tab "+(self.getTabCount()-1), self.addCameraTab(1));
//		self.setSelectedIndex(self.getTabCount()-1);
//		videoPanel.setTabCanvas(0);
		this.showDrop();
		t = new CurrentTabT();
		t.start();
		tabCounter = 1;//just keep a counter for tab number 
	}
//---------------------------               FUNCTIONS               ---------------------------//
	public void setDefaultIP(String defaultIP) {
		this.defaultIP = defaultIP;
	}
	public void setDefaultPort(String defaultPort) {
		this.defaultPort = defaultPort;
	}
	private void setDropSize(int dropHeight) {
		this.setBounds(x, y, width, height+dropHeight);
		this.setPreferredSize(new Dimension(width, height+dropHeight));
	}
	
	public CamSettingsPanel addCameraTab(int numCameras, VideoPanel panel) {
		return new CamSettingsPanel(numCameras, panel);
	}
	public CamSettingsPanel addCameraTab(int numCameras) {
		return new CamSettingsPanel(numCameras, videoPanel);
	}
	public void flipDrop() {
		if(self.getSize().height == height) showDrop();
		else hideDrop();
	}
	public void showDrop() {
		setDropSize(TabPanel.DEFAULT_DROP_HEIGHT);
	}
	public void hideDrop() {
		setDropSize(0);
	}
	
	public class CamSettingsPanel extends JPanel {
		CameraInitsPanel initPanels[];
		JPanel panels[];
		int num;

		public CamSettingsPanel(int i, VideoPanel VAL) {
			this.num = i;
			initPanels = new CameraInitsPanel[i];
			panels = new JPanel[i/2+1];
			this.setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
			for(int j = 0; j < i; j++) {
				if(j%2 == 0) {
					panels[j/2] = new JPanel();
					this.add(panels[j/2]);
				}
				initPanels[j] = new CameraInitsPanel(VAL);
				panels[j/2].add(initPanels[j]);
			}
			//this.setPreferredSize(new Dimension(self.width, 50));
		}
	}
	public class CameraInitsPanel extends JPanel {
		public static final String PLAY_BUTTON = "Play";
		public static final String PAUSE_BUTTON = "Pause";
		public static final String STOP_BUTTON = "Stop";
		
		public JButton playButton;
		public JButton stopButton;
		public JLabel labels[];
		public JTextArea inputs[];
		public JComboBox typeBox;
		public VideoActionListener videoListener;
		public InitActionListener initListener;
		
		public CameraInitsPanel(VideoPanel VAL) {
			playButton = new JButton(PLAY_BUTTON);
				videoListener = VAL.getVideoActionListener(this);
			playButton.addActionListener(initListener = new InitActionListener());
			stopButton = new JButton(STOP_BUTTON);
			stopButton.addActionListener(new StopHandleActionListener());
			labels = new JLabel[2];
			labels[0] = new JLabel("IP/Port");
			inputs = new JTextArea[2];
			inputs[0] = new JTextArea();
			inputs[0].setPreferredSize(new Dimension(100,14));
			inputs[1] = new JTextArea();
			inputs[1].setPreferredSize(new Dimension(100,14));
			typeBox = new JComboBox();
			typeBox.addItem("USB");
			typeBox.addItem("IP");
			typeBox.addActionListener(new ComboActionListener());
			
			this.setLayout(new BoxLayout(this, BoxLayout.X_AXIS));
			this.add(labels[0]);
			//this.add(Box.createRigidArea(new Dimension(15,0)));
			this.add(inputs[0]);
			this.add(Box.createRigidArea(new Dimension(15,0)));
			this.add(inputs[1]);
			//this.add(Box.createRigidArea(new Dimension(15,0)));
			this.add(playButton);
			//this.add(Box.createRigidArea(new Dimension(15,0)));
			this.add(stopButton);
			//this.add(Box.createRigidArea(new Dimension(15,0)));
			this.add(typeBox);
			
//			inputs[0].setText("192.168.80.121");
			//inputs[0].setText("192.168.254.1");
			//inputs[0].setText("192.168.10.1");
			//inputs[0].setText("rtsp://192.168.254.1:8554/test");
			//inputs[0].setText("rtsp://127.0.0.1:8554/test");
			//inputs[0].setText("localhost");
			inputs[0].setText(self.defaultIP);
			inputs[1].setText(""+(Integer.parseInt(defaultPort)+tabChange));
			tabChange += 1;
		}
		
		public class InitActionListener implements ActionListener {
			@Override
			public void actionPerformed(ActionEvent e) {
				videoListener.actionPerformed(e);//start/stop video
			}
		}
		public class StopHandleActionListener implements ActionListener {
			@Override
			public void actionPerformed(ActionEvent e) {
				videoListener.stopVideo();//start/stop video
			}
		}
		public class ComboActionListener implements ActionListener {
			@Override
			public void actionPerformed(ActionEvent e) {
				if(((JComboBox)e.getSource()).getSelectedItem().equals("USB")) {
					if(!videoListener.useUSB) videoListener.flipType();
				} else {
					if(videoListener.useUSB) videoListener.flipType();
				}
			}
		}
	}//CameraInitPanel class
	
	private class CurrentTabT extends Thread {
		
		public CurrentTabT() {
			curComponentNum = self.getSelectedIndex();
			if(curComponentNum == 0 || curComponentNum > 4)
				curComponent = (CamSettingsPanel) self.getSelectedComponent();
			else curComponent = null;
		}
		
		public void run() {
			while(true) {
				//add "1 camera" tab
				if(self.getSelectedIndex() == 0) {
					makeOneCameraTab();
				} else if(self.getSelectedIndex() == 1 ) {
					makeFourCameraTab();
				} else if(self.getSelectedIndex() == 2 ) {
				} else if(self.getSelectedIndex() == 3 ) {
				} else if(self.getSelectedIndex() == 4 ) {
					
				} else if(curComponentNum != self.getSelectedIndex()) {
					System.out.printf("switched:%d->%d\n", curComponentNum,self.getSelectedIndex());
					if(curComponentNum-5 >= 0) {
						videoPanel.stopVideos(curComponentNum-5);
						videoPanel.unbindNumTab(curComponentNum-5);
					} 
					if(self.getSelectedIndex()-5 >= 0) {
						videoPanel.setNumTab(self.getSelectedIndex()-5);
					}
					curComponentNum = self.getSelectedIndex();
					curComponent = (CamSettingsPanel) self.getSelectedComponent();
				}
				try { Thread.sleep(100); } 
				catch (InterruptedException e) { e.printStackTrace(); }
			}
		}//run
	}
	public void makePIPCameraTab() {
		pipPanel.addVideoTab(1);
		self.addTab("PiP", self.addCameraTab(1,pipPanel));
		curComponentNum = 0;
		self.setSelectedIndex(self.getTabCount()-1);
		System.out.printf("Make pip Tab\n");
		tabChange -= 1;
	}
	public void makeOneCameraTab() {
		videoPanel.addVideoTab(1);
		self.addTab("Tab "+tabCounter, self.addCameraTab(1));
		curComponentNum = 0;
		self.setSelectedIndex(self.getTabCount()-1);
		System.out.printf("Make 1 Tab%d\n",tabCounter);
		tabCounter += 1;
	}
	public void makeOneCameraTab(String IP, String port) {
		makeOneCameraTab();
		CamSettingsPanel temp = (CamSettingsPanel)this.getSelectedComponent();
		temp.initPanels[0].inputs[0].setText(IP);
		temp.initPanels[0].inputs[1].setText(port+"");
		tabChange -= 1;
	}
	public void makeFourCameraTab() {
		videoPanel.addVideoTab(4);
		self.addTab("T4b "+tabCounter, self.addCameraTab(4));
		curComponentNum = 1;
		self.setSelectedIndex(self.getTabCount()-1);
		System.out.printf("Make 4 Tab%d\n",tabCounter);
		tabCounter += 1;
	}
	public void changeTab() {
		curComponentNum = self.getSelectedIndex();
		curComponent = (CamSettingsPanel) self.getSelectedComponent();
	}
	public void setPipPanel(String IP, String port) {
		CamSettingsPanel temp = (CamSettingsPanel)this.getComponentAt(4);
		temp.initPanels[0].inputs[0].setText(IP);
		temp.initPanels[0].inputs[1].setText(port+"");
	}
	
	public static void main(String[] args){
		long[] d = new long[1];
		JFrame f = new JFrame();
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		//f.setLayout(null);
		TabPanel p = new TabPanel(0, 0,100,1f, new VideoPanel(0,0,0,0, 1f), new UDPClient("192.168.0.10", 5005, 5005), new VideoPanel(0,0,0,0, 1f)
		, new TCPClient("localhost", 5000));
		//f.getContentPane().add(p);
		//CameraInitPanel c = new CameraInitPanel();
		f.add(p);
		//f.setSize(800, 450);
		f.pack();
		f.setVisible(true);
	}
}
