package GUI;

import GUI.TabPanel.CameraInitsPanel;
import cameraTabs.CameraInitPanel2;
import cameraTabs.USBPlayer;
import cameraTabs.USBVideoPlayer;
import cameraTabs.RTSPPlayer;

import java.awt.BorderLayout;
import java.awt.Canvas;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.GridLayout;
import java.awt.Image;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.net.URISyntaxException;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.WindowConstants;

import org.gstreamer.State;
import org.gstreamer.swing.VideoComponent;

/**
 *
 * @author dennis
 */
public class VideoPanel extends GUIComponent {
	private VideoPanel self = this;
	private int IPcounter;
	private ArrayList<ArrayList<RTSPPlayer>> vPlayers;
	private ArrayList<ArrayList<USBVideoPlayer>> usbPlayers;
	private ArrayList<ArrayList<VideoComponent>> cam;
	//public CameraInitPanel2 camInitPanels;
	private String ip;
	private String port;
	private int width, height;
	public UpdateVideoFunction videoUpdateFunction;
	public boolean useUSB;
	
	private int tabCounter;
	private int tabVideoCounter;
	
	ArrayList<ArrayList<Image>> img;
	ArrayList<ArrayList<Integer>> i;
	ArrayList<ArrayList<Integer>> i0;
	ArrayList<ArrayList<Integer>> videoWidth;
	ArrayList<ArrayList<Integer>> videoHeight;
	Object object;
	
	public int videoNum;//get which tab currently on(for IP cameras)
	public int numTab; //current tab video is on
//---------------------------             CONSTRUCTORS             ---------------------------//
	public VideoPanel(int x, int y, int width, int height, float transparency) {
		super(x,y,width,height, transparency);
	}
	public VideoPanel(int x, int y, int width, int height, float transparency, String ip, String port) {
		super(x,y,width,height, transparency);
		this.ip = ip;
		this.port = port;
		this.width = width;
		this.height = height;
		//this.setBackground(Color.black);
		
//		cam = new Canvas();
//		cam.setPreferredSize(new Dimension(width, height));
//		cam.setBounds(x, y, width, height);
		
		//camInitPanels = new CameraInitPanel2(ip, port);
		vPlayers = new ArrayList();
		usbPlayers = new ArrayList();
		cam = new ArrayList<ArrayList<VideoComponent>>();
		img = new ArrayList<ArrayList<Image>>();
		i = new ArrayList<ArrayList<Integer>>();
		i0 = new ArrayList<ArrayList<Integer>>();
		videoWidth = new ArrayList<ArrayList<Integer>>();
		videoHeight = new ArrayList<ArrayList<Integer>>();
		
		//For PiP camera Settings
//		vPlayers.add(new ArrayList<RTSPPlayer>());
//		usbPlayers.add(new ArrayList<USBVideoPlayer>());
//		cam.add(new ArrayList<VideoComponent>());
//		img.add(new ArrayList<Image>());
//		i.add(new ArrayList<Integer>());
//		i0.add(new ArrayList<Integer>());
//		videoWidth.add(new ArrayList<Integer>());
//		videoHeight.add(new ArrayList<Integer>());
//		for(int j = 0; j < 1; j++) {
//			try {
//				cam.get(cam.size()-1).add(null);
//				img.get(img.size()-1).add(null);
//				i.get(i.size()-1).add(0);
//				i0.get(i.size()-1).add(0);
//				videoWidth.get(i.size()-1).add(0);
//				videoHeight.get(i.size()-1).add(0);
//			} catch(Exception e) {
//				e.printStackTrace();
//			}
//		}
		//For PiP camera Settings
//		vPlayers.add(new ArrayList<RTSPPlayer>());
//		usbPlayers.add(new ArrayList<USBVideoPlayer>());
//		cam.add(new ArrayList<VideoComponent>());
		//cam.get(0).add(new Canvas());
		videoUpdateFunction = new UpdateVideoFunction();
		tabCounter = 0;
		
	}
//---------------------------               FUNCTIONS               ---------------------------//
	public void setTabCanvas(int tabNum) {
		if(tabNum >= 0) {
	//		for(int i = 0; i < cam.get(tabNumPrev).size(); i++)//remove previous canvas
	//			this.remove(cam.get(tabNumPrev).get(i));
//			if(cam.get(tabNum).size() <= 1) this.setLayout(new FlowLayout());//add proper layout
//			else this.setLayout(new GridLayout(cam.get(tabNum).size()/2,cam.get(tabNum).size()/2));
			for(int i = 0; i < cam.get(tabNum).size(); i++) {//add current tab canvas
				if(cam.get(tabNum).get(i) != null) {
					this.add(cam.get(tabNum).get(i));
					//System.out.printf("added camera:%d %d %d %d",x1,y1, width1, height1);
				}
			}
			videoNum = tabNum;
			System.out.println("Set RTSP video "+tabNum);
		}
	}
	public void unbindNumTab(int tabNum) {
		if(tabNum >= 0) {
			for(int i = 0; i < cam.get(tabNum).size(); i++) {//add current tab canvas
				if(cam.get(tabNum).get(i) != null) {
					this.remove(cam.get(tabNum).get(i));
				}
			}
		}
	}
	public void addVideoTab(int numCanvas) {
		vPlayers.add(new ArrayList<RTSPPlayer>());
		usbPlayers.add(new ArrayList<USBVideoPlayer>());
		cam.add(new ArrayList<VideoComponent>());
		img.add(new ArrayList<Image>());
		i.add(new ArrayList<Integer>());
		i0.add(new ArrayList<Integer>());
		videoWidth.add(new ArrayList<Integer>());
		videoHeight.add(new ArrayList<Integer>());
		for(int x = 0; x < numCanvas; x++) {
			try {
				cam.get(cam.size()-1).add(null);
				img.get(img.size()-1).add(null);
				i.get(i.size()-1).add(0);
				i0.get(i.size()-1).add(0);
				videoWidth.get(i.size()-1).add(0);
				videoHeight.get(i.size()-1).add(0);
			} catch(Exception e) {
				e.printStackTrace();
			}
		}
		tabCounter++;
		tabVideoCounter = 0;
		System.out.println("add Video Tab");
	}
	public VideoActionListener getVideoActionListener(CameraInitsPanel initPanel) {
		System.out.printf("videoTab%d made\n",tabCounter-1);
		vPlayers.get(tabCounter-1).add(tabVideoCounter, null);
		usbPlayers.get(tabCounter-1).add(tabVideoCounter, null);
		return new VideoActionListener(tabCounter-1, tabVideoCounter++, initPanel);
	}
	
	@Override
	public void paintBuffer(Graphics g) {
		super.paintComponent(g);
		if( !this.useUSB ) {
//			for(int i = 0; i < cam.get(videoNum).size();i++) {
//				if(cam.get(videoNum).get(i) != null) {
//					this.paintComponent(cam.get(videoNum).get(i).getGraphics());
//					System.out.println("Paint RTSP");
//				}
//			}
			super.paintChildren(g);
		} else if( this.useUSB ) {
				for(int x = 0; x < img.get(numTab).size(); x++) {
					if(img.get(numTab).get(x) != null)
					g.drawImage(img.get(numTab).get(x), i.get(numTab).get(x), i0.get(numTab).get(x), videoWidth.get(numTab).get(x), videoHeight.get(numTab).get(x), null);
				}
		}
		//g.dispose();
	}
	public void stopVideos(int tabNum) {
		for(int x = 0; x < usbPlayers.get(tabNum).size(); x++) {
			if(usbPlayers.get(tabNum).get(x) != null)
				usbPlayers.get(tabNum).get(x).closeConnection();
		}
		for(int x = 0; x < vPlayers.get(tabNum).size(); x++) {
			if(vPlayers.get(tabNum).get(x) != null)
				vPlayers.get(tabNum).get(x).setStop();
		}
	}
//	public void clearScreens() {
//		for(int iter = 0; iter < img.length; iter++) {
//			img[iter] = null;
//			i[iter] = 0;
//			i0[iter] = 0;
//			videoWidth[iter] = 0;
//			videoHeight[iter] = 0;
//			object[iter] = null;
//		}
//	}

	public void getFrame(Image img, int i, int i0, int videoWidth, int videoHeight, int tabNum, int videoNum) {
		int iter = 0;
		if(i == width/2) iter++;
		if(i0 == height/2) iter += 2;
		
//		this.img[iter] = img;
//		this.i[iter] = i;
//		this.i0[iter] = i0;
//		this.videoWidth[iter] = videoWidth;
//		this.videoHeight[iter] = videoHeight;
//		this.object[iter] = object;
		this.img.get(tabNum).set(videoNum, img);
		this.i.get(tabNum).set(videoNum, i);
		this.i0.get(tabNum).set(videoNum, i0);
		this.videoWidth.get(tabNum).set(videoNum, videoWidth);
		this.videoHeight.get(tabNum).set(videoNum, videoHeight);
	}
	
	public class VideoActionListener implements ActionListener {
		int tabNum, numVideo;
		boolean startVideo = true;
		boolean useUSB = true;
		CameraInitsPanel initPanel;
		//String ip, port;
		
		public VideoActionListener(int tabNum, int numVideo, CameraInitsPanel initPanel) {
			this.tabNum = tabNum;
			this.numVideo = numVideo;
			this.initPanel = initPanel;
//			this.ip = ip;
//			this.port = port;
			self.useUSB = true;
		}
		public boolean flipSwitch() {
			return startVideo = !startVideo;
		}
		public boolean flipType() {
			stopVideo();//stop video before flipping it
			self.useUSB = false;
			return useUSB = !useUSB;
		}
		
		@Override
		public void actionPerformed(ActionEvent e) {
			if(startVideo) startVideo();
			else pauseVideo();
		}
		public void startVideo() {
			//camInitPanels = new CameraInitPanel2(initPanel.inputs[0].getText(),initPanel.inputs[1].getText());
			if( !useUSB ) {//IP
				if(vPlayers.get(tabNum).get(numVideo) == null) {
					try {
						//vPlayers.get(tabNum).add(numVideo, new RTSPPlayer(new String[]{}, "rtsp://127.0.0.1:8554/test"));
						vPlayers.get(tabNum).add(numVideo, new RTSPPlayer(initPanel.inputs[1].getText().split(" "), initPanel.inputs[0].getText()));
						//cam.get(tabNum).set(numVideo, vPlayers.get(tabNum).get(numVideo).getVideoComponent());
						vPlayers.get(tabNum).get(numVideo).videoComponent = new VideoComponent();
						cam.get(tabNum).set(numVideo, vPlayers.get(tabNum).get(numVideo).videoComponent);
						vPlayers.get(tabNum).get(numVideo).playbin.setVideoSink(vPlayers.get(tabNum).get(numVideo).videoComponent.getElement());
						
						int x1 = (numVideo%2)*(width/2);//either 0 or width/2 (go right?)
						int y1 = (numVideo/2)*(height/2);//either 0 or height/2 (go down?)
						int width1 = width;
							if(cam.get(tabNum).size() > 1) width1 /= 2;
						int height1 = height;
							if(cam.get(tabNum).size() > 1) height1 /= 2;
						
						cam.get(tabNum).get(numVideo).setPreferredSize(new Dimension(width1, height1));
						cam.get(tabNum).get(numVideo).setBounds(x1,y1, width1, height1);
						cam.get(tabNum).get(numVideo).setSize(width1, height1);
						cam.get(tabNum).get(numVideo).setOpaque(false);
						System.out.printf("Made vPlayer:%d %d\n", tabNum, numVideo);
					} catch (URISyntaxException e) {e.printStackTrace(); }
				}
				self.unbindNumTab(tabNum);
				self.setTabCanvas(tabNum);
				//vPlayers.get(tabNum).get(numVideo).videoComponent = cam.get(tabNum).get(numVideo);
//					JFrame frame = new JFrame("VideoPlayer");
//			    frame.getContentPane().add(vPlayers.get(tabNum).get(numVideo).videoComponent, BorderLayout.CENTER);
//			    frame.setPreferredSize(new Dimension(640, 480));
//			    frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
//			    frame.pack();
//			    frame.setVisible(true);
		    vPlayers.get(tabNum).get(numVideo).playbin.setState(State.PLAYING);
				//vPlayers.get(tabNum).get(numVideo).startMain();
				//vPlayers.get(tabNum).get(numVideo).setVideoSink();
				//vPlayers.get(tabNum).get(numVideo).setPlay();
		    System.out.println("Make RTSP");
//				self.setTabCanvas(tabNum);
			} else {//USB
				if(usbPlayers.get(tabNum).get(numVideo) == null) {
					int x1 = (numVideo%2)*(width/2);//either 0 or width/2 (go right?)
					int y1 = (numVideo/2)*(height/2);//either 0 or height/2 (go down?)
					int width1 = width;
						if(usbPlayers.get(tabNum).size() > 1) width1 /= 2;
					int height1 = height;
						if(usbPlayers.get(tabNum).size() > 1) height1 /= 2;
					usbPlayers.get(tabNum).add(numVideo, new USBVideoPlayer(self, x1,y1,width1,height1, initPanel,tabNum,numVideo));
				}
				usbPlayers.get(tabNum).get(numVideo).handleConnectionRequest();
			}
		}//startVideo
		public void pauseVideo() {
			if(!useUSB) {
				if(vPlayers.get(tabNum).get(numVideo) != null) 
					vPlayers.get(tabNum).get(numVideo).setPause();
			} else {
				if(usbPlayers.get(tabNum).get(numVideo) != null) { 
					usbPlayers.get(tabNum).get(numVideo).closeConnection();
				}
			}
		}
		public void stopVideo() {
			if( !useUSB ) {//IP
				if(vPlayers.get(tabNum).get(numVideo) != null)
					self.remove(vPlayers.get(tabNum).get(numVideo).getVideoComponent());
					vPlayers.get(tabNum).get(numVideo).setStop();
					vPlayers.get(tabNum).set(numVideo,null);
			} else {//USB
				if(usbPlayers.get(tabNum).get(numVideo) != null) { 
					usbPlayers.get(tabNum).get(numVideo).closeConnection();
				}
			}
		}//closeVideo
	}//VideoActionListener class
	
	public class UpdateVideoFunction extends UpdateFunction {
		@Override
		public boolean checkValues() { return true; }
		@Override
		public void doUpdate() { self.repaint(); }
	}

	public void setNumTab(int selectedIndex) {
		numTab = selectedIndex;
	}
}
