package cameraTabs;

/**
 * Camera Client widget for MarsRover Project @ York
 * Author: Bart Verzijlenberg
 * Date: Mar 12, 2009.
 */

import java.awt.Canvas;
import java.awt.Image;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ConnectException;
import java.net.Socket;
import java.net.UnknownHostException;

import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JTabbedPane;
import javax.swing.JTextField;


public class USBPlayer {
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
	public int forced_width = (int) (640 * 0.75);
	public int forced_height = (int) (480 * 0.75);

	// Connection members
	private Socket socket = null;
	private OutputStream out = null;
	private InputStream in = null;

	/**
	 * READ_ONLY!!!!!!
	 */
	private byte[] jpg_buf = new byte[960 * 720 * 3];

	private int deltaX;
	private int deltaY;
	private int width = 640;
	private int height = 480;
	private int fps = 30;
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
	CameraInitPanel2 initPanel;
	JButton play, stop;
	JTextField addressTA, portTA;
	Canvas cameraCanvas;
	Thread USBCameraThread;
	
	//int sleepInterval = 100;
	
//-------------------------------------------------------------------------//
	//constructors
	public USBPlayer(Canvas canvas, CameraInitPanel2 initPanel) {
		this.initPanel = initPanel;
		this.play = initPanel.startCamB;
		this.stop = initPanel.stopCamB;
		this.cameraCanvas = canvas;
		this.addressTA = initPanel.fieldInfos[0];
		this.portTA = initPanel.fieldInfos[1];
		this.hostname = this.addressTA.getText();
		this.startPort = Integer.parseInt(this.portTA.getText());
//		this.play.addActionListener(new USBCamActionListener());
//		this.stop.addActionListener(new USBCamActionListener());
	}//constructor
	
	public class USBCamActionListener implements ActionListener {
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
//----------------------------------------------------------------------------------------------/
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
	
	
	public void handleConnectionRequest() {
		try {
			socket = new Socket(hostname, startPort);
			System.out.println("Connected");
			out = socket.getOutputStream();
			in = socket.getInputStream();

			System.out.println("Connected2");

			// Start a new frame grabber
			// getDisplay().asyncExec(new grabFramesTask());
			grabThread = new grabFramesTask();
			USBCameraThread = new Thread(grabThread);
			USBCameraThread.start();

		} catch (ConnectException e) {
			if (e.getMessage().equals("Connection refused"))
				System.out.println("Connection Refused");
			else
				e.printStackTrace();

		} catch (UnknownHostException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
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
				charToByte(buf2, new String("APT" + (deltaX) + " " + deltaY));
				out.write(buf2, 0, 23);

				deltaX = deltaY = 0;
			} else if (sendSettings) { // Change the resolution and fps
				sendSettings = false;

				if (width < 100 || height < 100) {
					width = 640;
					height = 480;
					fps = 30;
				}
				if (fps == 0)
					fps = 30;

				byte[] buf2 = new byte[20];
				charToByte(buf2, new String("ack" + req_width + " " + req_height + " " + (2 * fps)));
				System.out.println(new String("ack" + req_width + " " + req_height + " " + (2 * fps)));
				out.write(buf2, 0, 13);

			} else if (sendFocusVal) { // Reset Focus
				sendFocusVal = false;
				byte t[] = { 'F', 'O', 'C' };
				out.write(t, 0, 3);
				out.write(intToByteArray(focus), 0, 4);

			} else if (resetPanTilt) { // Reset the pan/tilt position
				resetPanTilt = false;
				byte t[] = { 'R', 'C', 'K' };
				out.write(t, 0, 3);

			} else if (sendDoFocus) { // Autofocus
				sendDoFocus = false;
				byte t[] = { 'F', 'C', 'S' };
				out.write(t, 0, 3);

			} else if (resetFocus) { // Reset Focus
				resetFocus = false;
				byte t[] = { 'F', 'C', 'R' };
				out.write(t, 0, 3);

			} else if (setWhiteBalance) { // Reset Focus
				setWhiteBalance = false;
				byte t[] = { 'W', 'B', 'V' };
				out.write(t, 0, 3);
				out.write(intToByteArray(wbSetting), 0, 4);

			} else if (setWbMode) { // Reset Focus
				setWbMode = false;
				byte t[] = { 'W', 'B', 'A', (byte) wbModeSetting };
				out.write(t, 0, 4);

			} else if (setExposure) { // Reset Focus
				setExposure = false;
				byte t[] = { 'E', 'X', 'V' };
				out.write(t, 0, 3);
				out.write(intToByteArray(exposureSetting), 0, 4);

			} else if (setExposureMode) { // Reset Focus
				setExposureMode = false;
				byte t[] = { 'E', 'X', 'M', (byte) exposureModeSetting };
				out.write(t, 0, 4);

			} else if (init == 2) {
				init = 1;
				byte t[] = { 'V', 'F', 'M', (byte) (flip_v ? '1' : '0') };
				out.write(t, 0, 4);

			} else if (init == 1) {
				init = 0;
				byte t[] = { 'H', 'F', 'M', (byte) (flip_h ? '1' : '0') };
				out.write(t, 0, 4);

			} else {
				byte t[] = { 'A', 'C', 'K' };
				out.write(t, 0, 3);

			}
		}

		public void run() {
			ImageIcon imgIcon;
			Image img;
			int videoWidth, videoHeight;
			int ratioA = _DEFAULT_ASPECT_RATIO_A,ratioB = _DEFAULT_ASPECT_RATIO_B, ratioNum;
			
			while (alive) {
				try {
					
					// ===================================
					// Handle communication.

					// Get Focus Setting
					final byte c[] = new byte[4];
					readSocket(in, 4, c);

					// Get the size of the incoming frame
					readSocket(in, 10, buf);
					int length = Integer.parseInt(new String(buf).split(" ")[0]);
					byte_count += length;

					// Read the frame
					readSocket(in, length, jpg_buf);

					// Save an image for outside use.
					if(saveImage) {
						System.arraycopy(jpg_buf, 0, saved_image, 0, length);
						saveImage = false;
					}
					
					// Send an acknowledgement
					handleAcknowledgement();

					// ===================================
//--------------------------------		NEW CODE	-----------------------------------------------------------------//
					imgIcon = new ImageIcon(jpg_buf);
			        img = imgIcon.getImage();
			        
			      //  System.out.println(imgIcon.toString());
			        //System.out.println(img.getHeight(null));
			        //img = img.getScaledInstance(img.getWidth(null), img.getHeight(null), Image.SCALE_DEFAULT);
			        //img = img.getScaledInstance(cameraCanvas.getWidth(), cameraCanvas.getHeight(), Image.SCALE_DEFAULT);
			        videoWidth = cameraCanvas.getSize().width;
							videoHeight = cameraCanvas.getSize().height;
							ratioNum = Math.min((videoWidth / ratioA), (videoHeight / ratioB));
							videoWidth = ratioA * ratioNum;
							videoHeight = ratioB * ratioNum;
							cameraCanvas.getGraphics().drawImage(img, 0,0, videoWidth,videoHeight, null);
			        
				} catch (NumberFormatException e) {
					System.out.println("Number oops: " + e.getMessage());

				} catch (IOException e) {
					System.out.println("Thread Oops. : " + e.getMessage());
					return;
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

		if (socket != null) {
			try {
				out.close();
				in.close();
				socket.close();
				System.out.println("Closed Connection");

			} catch (IOException e) {
				e.printStackTrace();
			}
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