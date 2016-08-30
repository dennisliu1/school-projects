package cameraTabs;

/**
 * Camera Client widget for MarsRover Project @ York
 * Author: Bart Verzijlenberg
 * Date: Mar 12, 2009.
 */

import gui.VideoPanel;

import java.awt.Canvas;
import java.awt.Dimension;
import java.awt.Image;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ConnectException;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.*;


public class USBSplitter {
	public static final int _DEFAULT_WIDTH = 640;
	public static final int _DEFAULT_HEIGHT = 480;
	public static final int _DEFAULT_FPS = 30;
	public static final int _DEFAULT_ASPECT_RATIO_A = 4;
	public static final int _DEFAULT_ASPECT_RATIO_B = 3;
	
	public byte[] saved_image = new byte[960 * 720 * 3];
	public boolean saveImage = false;
	public String hostname = "192.168.1.1";
	public boolean flip_h = false;
	public boolean flip_v = false;
	public boolean rotation45 = false;
	public double offset_x;
	public double offset_y;
	public double x_fov;
	public double y_fov;
	public boolean telescopeFast = false;
	public boolean forcedSize = false;
	public int forced_width = (int) (_DEFAULT_WIDTH * 0.75);
	public int forced_height = (int) (_DEFAULT_HEIGHT * 0.75);

	// Connection members
	private Socket socketSource = null;
	private OutputStream outSource = null;
	private InputStream inSource = null;
	
	private boolean isDest[];
	private ServerSocket serverSocketDest[];
	private Socket socketDest[];
	private OutputStream outDest[];
	private InputStream inDest[];

	/**
	 * READ_ONLY!!!!!!
	 */
	private byte[] jpg_buf = new byte[960 * 720 * 3];

	private int deltaX;
	private int deltaY;
	private int width = _DEFAULT_WIDTH;
	private int height = _DEFAULT_HEIGHT;
	private int fps = _DEFAULT_FPS;
	private int req_width = width;
	private int req_height = height;
	private int focus = 1;
	private int wbSetting;
	private int exposureSetting;
	private int init;

	private char wbModeSetting = '1';
	private char exposureModeSetting = '3';

	private boolean setWhiteBalance = false;
	private boolean setWbMode = true;
	private boolean sendPanTilt = false;
	private boolean sendSettings = false;
	private boolean sendDoFocus = false;
	private boolean sendFocusVal = true;
	private boolean resetPanTilt = false;
	private boolean resetFocus = false;
	private boolean setExposure = false;
	private boolean setExposureMode = true;

	// Configuration settings
	private int startPort = 30100;

	//private boolean telescopeMode = false;
	private grabFramesTask grabThread;
//-------------------------------------	Custom stuff	------------------------------------//
	JTabbedPane tabPane;
	JButton play, stop;
	JTextField addressTA, portTA;
	Thread USBCameraThread;
	
	CameraInitPanel sourcePanel, initPanels[];
	SourceActionListener sourceActionListener;
	DestActionListener destActionListener[];
	//int sleepInterval = 100;
//-------------------------------------------------------------------------//
	//constructors
	public USBSplitter(CameraInitPanel sourcePanel, CameraInitPanel[] initPanels) {
		this.play = sourcePanel.startCamB;
		this.stop = sourcePanel.stopCamB;
		this.addressTA = sourcePanel.fieldInfos[0];
		this.portTA = sourcePanel.fieldInfos[1];
		this.hostname = this.addressTA.getText();
		this.startPort = Integer.parseInt(this.portTA.getText());
		
		this.sourcePanel = sourcePanel;
		this.initPanels = initPanels;
		
		serverSocketDest = new ServerSocket[initPanels.length];
		socketDest = new Socket[initPanels.length];
		outDest = new OutputStream[initPanels.length];
		inDest = new InputStream[initPanels.length];
		
		isDest = new boolean[initPanels.length];
		for(int i = 0; i < initPanels.length; i++)
			isDest[i] = false;
		sourceActionListener = new SourceActionListener();
		destActionListener = new DestActionListener[initPanels.length];
		for(int i = 0; i < destActionListener.length; i++)
			destActionListener[i] = new DestActionListener();
	}//constructor
	
//	public class USBCamActionListener implements ActionListener {
//		@Override
//		public void actionPerformed(ActionEvent ae) {
//			if(ae.getSource().equals(play)) {
//				System.out.println("Play USB!");
//				handleConnectionRequest();
//			} else if(ae.getSource().equals(stop)) {
//				// Disconnect if we are connected to a stream
//				System.out.println("Stop USB!");
//				closeConnection();
//			}
//		}
//	}
//----------------------------------------------------------------------------------------------/
	
	//actionlistener for source initPanel
	public class SourceActionListener implements ActionListener {
		public void actionPerformed(ActionEvent ae) {
			if(ae.getSource().equals(play)) {
				System.out.println("Play USB!");
				handleConnectionRequest();
			} else if(ae.getSource().equals(stop)) {
				// Disconnect if we are connected to a stream
				System.out.println("Stop USB!");
				closeConnection();
			}
		}
	}
	
	//Destination initPanel
	public class DestActionListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent ae) {
			if(((JButton)ae.getSource()).getText().equals("start")) {
				System.out.println("Start Split USB!");
				
				StartThread t = new StartThread();
				t.setAE(ae);
				t.start();
			} else if(((JButton)ae.getSource()).getText().equals("stop")) {
				// Disconnect if we are connected to a stream
				System.out.println("Stop Split USB!");
				
			}
		}
	}
	public class StartThread extends Thread {
		ActionEvent ae;
		public void setAE(ActionEvent ae) {
			this.ae = ae;
		}
		public void run() {
			setDestinationRequest(ae);
		}
	}
	
	public void handleConnectionRequest() {
		try {
			socketSource = new Socket(hostname, startPort);
			System.out.println("Connected");
			outSource = socketSource.getOutputStream();
			inSource = socketSource.getInputStream();
			
			System.out.println("Connected2");
			
			// Start a new frame grabber
			// getDisplay().asyncExec(new grabFramesTask());
			grabThread = new grabFramesTask();
			USBCameraThread = new Thread(grabThread);
			USBCameraThread.start();

		} catch (ConnectException e) {
			if (e.getMessage().equals("Connection refused"))
				System.out.println("Connection Refused");
		} catch (UnknownHostException e) {} 
		catch (IOException e) { }
	}
	public void setDestinationRequest(ActionEvent ae) {
		try {
			int i = 0;
			for(; i < initPanels.length; i++) //find source#
				if(ae.getSource().equals(initPanels[i].startCamB)) break;
			System.out.printf("split to: %d\n",i);
			if(i < initPanels.length) {//found source#
				serverSocketDest[i] = new ServerSocket(Integer.parseInt(initPanels[i].fieldInfos[1].getText()));
				System.out.printf("waiting: %d\n",i);
				socketDest[i] = serverSocketDest[i].accept();
				outDest[i] = socketDest[i].getOutputStream();
				inDest[i] = socketDest[i].getInputStream();
				isDest[i] = true;
				System.out.printf("start splitting: %d\n",i);
			}
		} catch(Exception e) {}
	}
	public void closeDestinationRequest(ActionEvent ae) {
		int i = 0;
		for(; i < initPanels.length; i++) //find source#
				if(ae.getSource().equals(initPanels[i].startCamB)) break;
		if(serverSocketDest[i] != null) {
			try {
				isDest[i] = false;
				inDest[i].close();
				outDest[i].close();
				serverSocketDest[i].close();
				System.out.printf("closed split: %d\n",i);
			} catch (IOException ex) {}
		}
	}
	
	public class grabFramesTask implements Runnable {

		byte buf[] = new byte[10];
		private int byte_count;
		public boolean alive = true;

		private void handleAcknowledgement() throws IOException {

			if (sendPanTilt) {
				sendPanTilt = false;
				byte[] buf2 = new byte[23];
				charToByte(buf2, ("APT" + (deltaX) + " " + deltaY));
				outSource.write(buf2, 0, 23);

				deltaX = deltaY = 0;
			} else if (sendSettings) { // Change the resolution and fps
				sendSettings = false;

				if (width < 100 || height < 100) {
					width = _DEFAULT_WIDTH;
					height = _DEFAULT_HEIGHT;
					fps = _DEFAULT_FPS;
				}
				if (fps == 0)
					fps = _DEFAULT_FPS;

				byte[] buf2 = new byte[20];
				charToByte(buf2, ("ack" + req_width + " " + req_height + " " + (2 * fps)));
				System.out.println(("ack" + req_width + " " + req_height + " " + (2 * fps)));
				outSource.write(buf2, 0, 13);

			} else if (sendFocusVal) { // Reset Focus
				sendFocusVal = false;
				byte t[] = { 'F', 'O', 'C' };
				outSource.write(t, 0, 3);
				outSource.write(intToByteArray(focus), 0, 4);

			} else if (resetPanTilt) { // Reset the pan/tilt position
				resetPanTilt = false;
				byte t[] = { 'R', 'C', 'K' };
				outSource.write(t, 0, 3);

			} else if (sendDoFocus) { // Autofocus
				sendDoFocus = false;
				byte t[] = { 'F', 'C', 'S' };
				outSource.write(t, 0, 3);

			} else if (resetFocus) { // Reset Focus
				resetFocus = false;
				byte t[] = { 'F', 'C', 'R' };
				outSource.write(t, 0, 3);

			} else if (setWhiteBalance) { // Reset Focus
				setWhiteBalance = false;
				byte t[] = { 'W', 'B', 'V' };
				outSource.write(t, 0, 3);
				outSource.write(intToByteArray(wbSetting), 0, 4);

			} else if (setWbMode) { // Reset Focus
				setWbMode = false;
				byte t[] = { 'W', 'B', 'A', (byte) wbModeSetting };
				outSource.write(t, 0, 4);

			} else if (setExposure) { // Reset Focus
				setExposure = false;
				byte t[] = { 'E', 'X', 'V' };
				outSource.write(t, 0, 3);
				outSource.write(intToByteArray(exposureSetting), 0, 4);

			} else if (setExposureMode) { // Reset Focus
				setExposureMode = false;
				byte t[] = { 'E', 'X', 'M', (byte) exposureModeSetting };
				outSource.write(t, 0, 4);

			} else if (init == 2) {
				init = 1;
				byte t[] = { 'V', 'F', 'M', (byte) (flip_v ? '1' : '0') };
				outSource.write(t, 0, 4);

			} else if (init == 1) {
				init = 0;
				byte t[] = { 'H', 'F', 'M', (byte) (flip_h ? '1' : '0') };
				outSource.write(t, 0, 4);

			} else {
				byte t[] = { 'A', 'C', 'K' };
				outSource.write(t, 0, 3);

			}
		}

		@Override
		public void run() {
			ImageIcon imgIcon;
			Image img;
			int videoWidth, videoHeight;
			int ratioA = _DEFAULT_ASPECT_RATIO_A,ratioB = _DEFAULT_ASPECT_RATIO_B, ratioNum;
			int i = 0;
			
			while (alive) {
				try {
					// ===================================
					// Handle communication.

					// Get Focus Setting
					final byte c[] = new byte[4];
					readSocket(inSource, 4, c);

					// Get the size of the incoming frame
					readSocket(inSource, 10, buf);
					int length = Integer.parseInt(new String(buf).split(" ")[0]);
					byte_count += length;

					// Read the frame
					readSocket(inSource, length, jpg_buf);

					// Save an image for outside use.
					if(saveImage){
						System.arraycopy(jpg_buf, 0, saved_image, 0, length);
						saveImage = false;
					}
					
					// Send an acknowledgement
					handleAcknowledgement();

					// ===================================
//--------------------------------		NEW CODE	-----------------------------------------------------------------//
					i = 0;
					for(; i < isDest.length; i++) {
						if(isDest[i]) {
							if(!serverSocketDest[i].isClosed())
								outDest[i].write(c, 0, c.length);
							if(!serverSocketDest[i].isClosed())
								outDest[i].write(buf, 0, buf.length);
							if(!serverSocketDest[i].isClosed())
								outDest[i].write(jpg_buf, 0, length);
							
						}
					}
				} catch (NumberFormatException e) {
					System.out.println("Number oops: " + e.getMessage());

				} catch (IOException e) {
					System.out.println("Thread Oops. : " + e.getMessage());
					//e.printStackTrace();
					//if(serverSocketDest[i].isClosed()) {
						try {
							inDest[i].close();
							outDest[i].close();
							serverSocketDest[i].close();
							System.out.printf("closed split: %d\n",i);
							if(isDest[i]) {
								isDest[i] = false;
								serverSocketDest[i] = new ServerSocket(Integer.parseInt(initPanels[i].fieldInfos[1].getText()));
								System.out.printf("waiting: %d\n",i);
								socketDest[i] = serverSocketDest[i].accept();
								outDest[i] = socketDest[i].getOutputStream();
								inDest[i] = socketDest[i].getInputStream();
								isDest[i] = true;
								System.out.printf("start splitting: %d\n",i);
							}
						} catch (IOException ex) {}
					//}
					//return;
				} catch (Exception e) {
					e.printStackTrace();
				}
			}//whiletrue
		}//run
	}
	
	//close the connection to the USB Camera.
	public void closeConnection() {

		System.out.println("Closing Connection");

		if (grabThread != null) {
			USBCameraThread.stop();
			grabThread.alive = false;
			grabThread = null;
		}

		if (socketSource != null) {
			try {
				outSource.close();
				inSource.close();
				socketSource.close();
				System.out.println("Closed Connection");

			} catch (IOException e) { }
		}
	}

	//reads from the socket
	private void readSocket(InputStream ins, int length, byte[] buf) throws IOException {
		int res;
		res = 0;
		while (length != res) {
			if (ins.available() > 0) {
				int r = ins.read(buf, res, length - res);
				if (r > 0)
					res += r;

			}
		}
	}

	//String(char[]) to byte[]
	private void charToByte(byte[] buf2, String s) {
		char[] c = s.toCharArray();
		for (int l = 0; l < buf2.length; l++)
			buf2[l] = (byte) (l < c.length ? c[l] : ' ');
	}
	
//------------------------	 MISC + SWT STUFF	--------------------------------------------//
	
	public void dispose() {
		System.out.println("Disposing of CameraWidget");

		//super.dispose();

		// make sure we close cleanly
		closeConnection();

	}
	
	//converts Integer to byte[4].
	private byte[] intToByteArray(final int integer) {
		int byteNum = 4;
		byte[] byteArray = new byte[4];

		for (int n = 0; n < byteNum; n++)
			byteArray[n] = (byte) (integer >>> (n * 8));

		return (byteArray);
	}
}