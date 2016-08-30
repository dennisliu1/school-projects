package york.rover.camera;


import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ConnectException;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Timer;
import java.util.TimerTask;

import org.eclipse.swt.SWT;
import org.eclipse.swt.SWTException;
import org.eclipse.swt.events.MouseEvent;
import org.eclipse.swt.events.MouseListener;
import org.eclipse.swt.events.MouseMoveListener;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Canvas;
import org.eclipse.swt.widgets.Combo;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.MessageBox;
import org.eclipse.swt.widgets.TabFolder;
import org.eclipse.swt.widgets.TabItem;
import org.eclipse.swt.widgets.Text;

public class StereoWidget extends Composite {

	TabFolder folder;

	// Parts for settings page:
	TabItem settings;
	private Text firstPort;
	private Text numCamera;
	private Text host;

	// Configuration settings
	private String hostname = "marsrover.cse.yorku.ca";
	private int startPort = 30200;
	private int numberOfCameras = 1;

	String currentCamera = "";
	private TabItem currentTab;

	private Socket socket = null;
	private OutputStream out = null;
	private InputStream in = null;
	private Timer timer;
	
	private byte[] jpg_buf = new byte[640*480*3];
	private byte[] jpg_buf2 = new byte[640*480*3];
	
	private int xDown;
	private int yDown;
	private int deltaX;
	private int deltaY;

	int width = 640, height = 480, fps = 15;
	
	private int req_width = width;
	private int req_height = height;
	
	boolean sendPanTilt;
	boolean sendSettings;

	boolean setFocus = false;
	int focusSetting = 0;
	
	private boolean setExposure = false;
	private int exposureSetting;

	private int wbSetting;
	private boolean setWhiteBalance = false;
	
	private char wbModeSetting;
	private boolean setWbMode = false;
	
	private char exposureModeSetting;
	private boolean setExposureMode = false;
	
	private char expAutoModeSetting;
	private boolean setExpAutoMode = false;
	
	boolean down = false;

	private Canvas canv;

	private Canvas canv2;

	private Text focus;

	private Text exposureValue;

	private Text whiteBalanceValue;

	private Text wbValue;

	private Text exposureModeValue;

	private Text expAutoValue;
	
	
	public void setCamTabName(String arg, int i){
		folder.getItem(i).setText(arg);
	}
	
	@Override
	public void dispose() {
		super.dispose();

		// make sure we close cleanly
		closeConnection();
		
	}

	public StereoWidget(Composite parent, int style, int start, int numCams) {
		super(parent, style);

		startPort = start;
		numberOfCameras = numCams;

		setLayout(new GridLayout(1, false));

		init();
	}
	
	public StereoWidget(Composite parent, int style) {
		super(parent, style);

		setLayout(new GridLayout(1, false));

		init();

	}

	private void init() {
		// Add a tabFolder
		folder = new TabFolder(this, SWT.BORDER);
		GridData gd = new GridData(SWT.FILL, SWT.FILL, true, true);
		gd.horizontalAlignment = SWT.CENTER;
		gd.verticalAlignment = SWT.CENTER;
		folder.setLayoutData(gd);

		settings = new TabItem(folder, SWT.NONE);
		settings.setText("Settings");
		Composite cSettings = createSettingsPage();
		settings.setControl(cSettings);

		folder.addSelectionListener(new SelectionListener() {
			
			public void widgetSelected(SelectionEvent arg0) {
				handleConnectionRequest((TabItem) arg0.item);
				folder.pack();

			}

			public void widgetDefaultSelected(SelectionEvent arg0) {

			}
		});
		
		initTabs();		
		folder.pack();
	}

	private void closeConnection() {

		if (socket != null) {

			try {
				timer.cancel();

				out.close();
				in.close();
				socket.close();
				printMsg("Closed Connection");

			} catch (IOException e) {
				e.printStackTrace();
			}

		}
	}

	private Composite createSettingsPage() {
		Composite cSettings = new Composite(folder, SWT.NONE);
		cSettings.setLayout(new GridLayout(2, false));

		// =========================================
		Label l = new Label(cSettings, SWT.NONE);
		l.setText("Server Hostname");
		l.setLayoutData(new GridData());

		host = new Text(cSettings, SWT.SINGLE | SWT.BORDER);
		host.setText(hostname);
		GridData gd = new GridData();
		gd.widthHint = 200;
		host.setLayoutData(gd);

		// =========================================
		l = new Label(cSettings, SWT.NONE);
		l.setText("Number of Cameras");
		l.setLayoutData(new GridData());

		numCamera = new Text(cSettings, SWT.SINGLE | SWT.BORDER);
		numCamera.setText(Integer.toString(numberOfCameras));
		gd = new GridData();
		gd.widthHint = 200;
		numCamera.setLayoutData(gd);

		// =========================================
		l = new Label(cSettings, SWT.NONE);
		l.setText("First Port");
		l.setLayoutData(new GridData());

		firstPort = new Text(cSettings, SWT.SINGLE | SWT.BORDER);
		firstPort.setText(Integer.toString(startPort));
		gd = new GridData();
		gd.widthHint = 200;
		firstPort.setLayoutData(gd);

		new Label(cSettings, SWT.NONE).setLayoutData(new GridData());

		Button b = new Button(cSettings, SWT.PUSH);
		b.setText("Apply");
		b.setLayoutData(new GridData());
		b.addSelectionListener(new SelectionListener() {

			public void widgetDefaultSelected(SelectionEvent arg0) {

			}

			public void widgetSelected(SelectionEvent arg0) {
				try {
					hostname = host.getText();
					startPort = Integer.parseInt(firstPort.getText());
					numberOfCameras = Integer.parseInt(numCamera.getText());
				} catch (NumberFormatException e) {

					MessageBox m = new MessageBox(folder.getShell());
					m.setText("Number Format Error");
					m.setMessage("Oops. Should have numbers only for startPort and numberOfCameras)");
					m.open();

				}

				initTabs();

			}
		});
		
		return cSettings;
	}

	private void initTabs() {

		for (int i = folder.getItemCount() - 1; i < numberOfCameras; i++) {
			
			TabItem camTab = new TabItem(folder, SWT.NONE);
			Composite comp = new Composite(folder, SWT.NONE);
			comp.setLayout(new GridLayout(8, false));
			
			
			// Add frame size selector
			Combo combo = new Combo (comp, SWT.READ_ONLY);
			combo.setItems (new String [] {"160x120", "320x240","640x480","800x600", "960x720"});
			combo.setSize (200, 200);
			combo.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
			combo.select(2);
			combo.addSelectionListener(new SelectionListener(){
				
				public void widgetDefaultSelected(SelectionEvent arg0) {}

				public void widgetSelected(SelectionEvent arg0) {
					int sel = ((Combo)arg0.widget).getSelectionIndex();
					String s = ((Combo)arg0.widget).getItem(sel);
					
					req_width = Integer.parseInt( s.split("x")[0] );
					req_height = Integer.parseInt( s.split("x")[1] );
						
					Canvas c = (Canvas)folder.getSelection()[0].getControl().getData();
					GridData g= (GridData)c.getLayoutData();
					g.minimumHeight = height;
					g.minimumWidth = width;
					folder.getShell().pack();
					
					sendSettings = true;
										
				}
			});
			
			// Add FPS selector
			Combo combo2 = new Combo (comp, SWT.READ_ONLY);
			combo2.setItems (new String [] {"5", "10", "15"});
			combo2.setSize (200, 200);
			combo2.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
			combo2.select(2);
			combo2.addSelectionListener(new SelectionListener(){
				public void widgetDefaultSelected(SelectionEvent arg0) {}

				public void widgetSelected(SelectionEvent arg0) {
					int sel = ((Combo)arg0.widget).getSelectionIndex();
					
					if( fps != sel*5 + 5){
						fps = sel*5 + 5;
						sendSettings = true;
					}
				}
			});
			
			
			
			
			Button doFocusButton = new Button(comp, SWT.PUSH);
			doFocusButton.setText("Set Focus");
			doFocusButton.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
			doFocusButton.addSelectionListener(new SelectionListener(){
				@Override
				public void widgetDefaultSelected(SelectionEvent arg0) {
					// TODO Auto-generated method stub
				}

				@Override
				public void widgetSelected(SelectionEvent arg0) {
					focusSetting = Integer.parseInt(focus.getText());
					setFocus = true;
					
				}
			});
			focus = new Text(comp, SWT.SINGLE | SWT.BORDER);
			focus.setText("2");
			GridData gd = new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1);
			gd.widthHint = 100;
			focus.setLayoutData(gd);
			
			
			//---------------------------
			Button setExposureButton = new Button(comp, SWT.PUSH);
			setExposureButton.setText("Set Exposure Value");
			setExposureButton.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
			setExposureButton.addSelectionListener(new SelectionListener(){
				
				@Override
				public void widgetDefaultSelected(SelectionEvent arg0) {
					// TODO Auto-generated method stub
				}

				@Override
				public void widgetSelected(SelectionEvent arg0) {
					exposureSetting = Integer.parseInt(exposureValue.getText());
					setExposure = true;
					
				}
			});
			exposureValue = new Text(comp, SWT.SINGLE | SWT.BORDER);
			exposureValue.setText("166");
			gd = new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1);
			gd.widthHint = 100;
			exposureValue.setLayoutData(gd);
			//-----------------------------
			
			//---------------------------
			Button whiteBalanceButton = new Button(comp, SWT.PUSH);
			whiteBalanceButton.setText("Set whiteBalance Value");
			whiteBalanceButton.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
			whiteBalanceButton.addSelectionListener(new SelectionListener(){

				@Override
				public void widgetDefaultSelected(SelectionEvent arg0) {
					// TODO Auto-generated method stub
				}

				@Override
				public void widgetSelected(SelectionEvent arg0) {
					wbSetting = Integer.parseInt(whiteBalanceValue.getText());
					setWhiteBalance = true;
					
				}
			});
			whiteBalanceValue = new Text(comp, SWT.SINGLE | SWT.BORDER);
			whiteBalanceValue.setText("4000");
			gd = new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1);
			gd.widthHint = 100;
			whiteBalanceValue.setLayoutData(gd);
			//-----------------------------
			
			
			//---------------------------
			Button exposureModeButton = new Button(comp, SWT.PUSH);
			exposureModeButton.setText("Set Exposure mode 1,3");
			exposureModeButton.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
			exposureModeButton.addSelectionListener(new SelectionListener(){

				@Override
				public void widgetDefaultSelected(SelectionEvent arg0) {
					// TODO Auto-generated method stub
				}

				@Override
				public void widgetSelected(SelectionEvent arg0) {
					exposureModeSetting = exposureModeValue.getText().charAt(0);
					setExposureMode = true;
					
				}
			});
			
			exposureModeValue = new Text(comp, SWT.SINGLE | SWT.BORDER);
			exposureModeValue.setText("0");
			gd = new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1);
			gd.widthHint = 100;
			exposureModeValue.setLayoutData(gd);
			//-----------------------------
						
			//---------------------------
			Button setExposureAutoModeButton = new Button(comp, SWT.PUSH);
			setExposureAutoModeButton.setText("Set Exposure Auto mode 0,1");
			setExposureAutoModeButton.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
			setExposureAutoModeButton.addSelectionListener(new SelectionListener(){

				@Override
				public void widgetDefaultSelected(SelectionEvent arg0) {
					// TODO Auto-generated method stub
				}

				@Override
				public void widgetSelected(SelectionEvent arg0) {
					expAutoModeSetting = expAutoValue.getText().charAt(0);
					setExpAutoMode = true;
					
				}
			});
			expAutoValue = new Text(comp, SWT.SINGLE | SWT.BORDER);
			expAutoValue.setText("0");
			gd = new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1);
			gd.widthHint = 100;
			expAutoValue.setLayoutData(gd);
			//-----------------------------

			//---------------------------
			Button whiteBalanceModeButton = new Button(comp, SWT.PUSH);
			whiteBalanceModeButton.setText("Set White Balance mode 0,1");
			whiteBalanceModeButton.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
			whiteBalanceModeButton.addSelectionListener(new SelectionListener(){

				@Override
				public void widgetDefaultSelected(SelectionEvent arg0) {
					// TODO Auto-generated method stub
				}

				@Override
				public void widgetSelected(SelectionEvent arg0) {
					wbModeSetting = wbValue.getText().charAt(0);
					setWbMode = true;
					
				}
			});
			wbValue = new Text(comp, SWT.SINGLE | SWT.BORDER);
			wbValue.setText("0");
			gd = new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1);
			gd.widthHint = 100;
			wbValue.setLayoutData(gd);
			
			//-----------------------------
			
			// Add the canvas
			canv = new Canvas(comp, SWT.BORDER);
			GridData d = new GridData(SWT.FILL, SWT.FILL, true, true, 4, 1);
			canv.setLayoutData(d);
			
			canv.addMouseMoveListener(new MouseMoveListener(){
				public void mouseMove(MouseEvent arg0) {

					deltaX = arg0.x - xDown;
					deltaY = arg0.y - yDown;
				
					if(down)
						sendPanTilt = true;
					
				}});
				
			canv.addMouseListener(new MouseListener(){
					public void mouseDoubleClick(MouseEvent arg0) {}
					public void mouseUp(MouseEvent arg0) {
						down = false;
					}
					
					public void mouseDown(MouseEvent arg0) {
						down = true;
						xDown = arg0.x;
						yDown = arg0.y;
					
					}		
				});
			
			// Add the canvas
			canv2 = new Canvas(comp, SWT.BORDER);
			d = new GridData(SWT.FILL, SWT.FILL, true, true, 4, 1);
			canv2.setLayoutData(d);
			
			canv2.addMouseMoveListener(new MouseMoveListener(){
				public void mouseMove(MouseEvent arg0) {

					deltaX = arg0.x - xDown;
					deltaY = arg0.y - yDown;
				
					if(down)
						sendPanTilt = true;
					
				}});
				
			canv2.addMouseListener(new MouseListener(){
					public void mouseDoubleClick(MouseEvent arg0) {}
					public void mouseUp(MouseEvent arg0) {
						down = false;
					}
					
					public void mouseDown(MouseEvent arg0) {
						down = true;
						xDown = arg0.x;
						yDown = arg0.y;
					
					}		
				});
			
			camTab.setText("Camera " + (i + 1));
			camTab.setData(new Integer(i));
			camTab.setControl(comp);
			
			comp.setData(canv);

			

		}
	}

	private byte[] intToByteArray (final int integer) {
		int byteNum = (40 - Integer.numberOfLeadingZeros (integer < 0 ? ~integer : integer)) / 8;
		byte[] byteArray = new byte[4];
		
		for (int n = 0; n < byteNum; n++)
			byteArray[n] = (byte) (integer >>> (n * 8));
		
		return (byteArray);
	}
	
	public class grabFramesTask extends TimerTask {

		private Image i, i2;
		byte buf[] = new byte[10];
		
		@Override
		public void run() {

			try {
				readSocket(10, buf);
				int length = Integer.parseInt(new String(buf).split(" ")[0]);
				readSocket(length, jpg_buf);
				InputStream in = new ByteArrayInputStream(jpg_buf);
				

				readSocket(10, buf);
				int length2 = Integer.parseInt(new String(buf).split(" ")[0]);
				readSocket(length2, jpg_buf2);
				InputStream in2 = new ByteArrayInputStream(jpg_buf2);

				
				try {
					i = new Image(currentTab.getDisplay(), in);
					height = i.getImageData().height;
					width = i.getImageData().width;
					
					i2 = new Image(currentTab.getDisplay(), in2);
					height = i2.getImageData().height;
					width = i2.getImageData().width;
					
					currentTab.getDisplay().syncExec(new Runnable() {
						public void run() {
							
							Canvas c = canv;
							
							if( c.getBackgroundImage() != null)
								c.getBackgroundImage().dispose();
							
							c.setBackgroundImage(i);
							GridData g= (GridData)c.getLayoutData();		
														
							if(g.minimumHeight != height)
							{
								g.minimumHeight = height;
								g.minimumWidth = width;
								c.setLayoutData(g);
								
								folder.getShell().pack();
								System.out.println("Resizing");
							}
							
							c = canv2;
							
							if( c.getBackgroundImage() != null)
								c.getBackgroundImage().dispose();
							
							c.setBackgroundImage(i2);
							g = (GridData)c.getLayoutData();		
														
							if(g.minimumHeight != height)
							{
								g.minimumHeight = height;
								g.minimumWidth = width;
								c.setLayoutData(g);
								
								folder.getShell().pack();
								System.out.println("Resizing");
							}
						}
					});
					
				} catch (SWTException e) {
					if( e.getMessage().equals("Invalid image")){
						System.out.println(e.getMessage());
					}else
						throw e;

				}
				
				//TODO Yes, its kinda screwy, but it works for now.
				if(sendPanTilt){
					sendPanTilt = false;
					xDown += deltaX;
					yDown += deltaY;
					
					byte[] buf2 = new byte[23];
					charToByte(buf2, new String("APT" + (-deltaX)/3 + " " + deltaY/3));
					out.write(buf2, 0, 23);
					
				} else if( sendSettings){			// Change the resolution and fps
					sendSettings = false;
					if( width < 100 || height < 100){
						width = 640;
						height = 480;
						fps = 15;
					}
					
					byte[] buf2 = new byte[20];
					charToByte(buf2, new String("ack" + req_width + " " + req_height + " " + (2*fps)));
					System.out.println(new String("ack" + req_width + " " + req_height + " " + (2*fps)));
					out.write(buf2, 0, 13);
										
				} else if( setFocus){	// Reset Focus
					setFocus = false;
					byte t[] = { 'F', 'O', 'C' };
					out.write(t, 0, 3);
					out.write(intToByteArray(focusSetting), 0, 4);
					
				} else if( setExposure){	// Reset Focus
					setExposure = false;
					byte t[] = { 'E', 'X', 'V' };
					out.write(t, 0, 3);
					out.write(intToByteArray(exposureSetting), 0, 4);
				
				} else if( setWhiteBalance){	// Reset Focus
					setWhiteBalance = false;
					byte t[] = { 'W', 'B', 'V' };
					out.write(t, 0, 3);
					out.write(intToByteArray(wbSetting), 0, 4);
						
				} else if( setWbMode){	// Reset Focus
					setWbMode = false;
					byte t[] = { 'W', 'B', 'A', (byte) wbModeSetting };
					out.write(t, 0, 4);
					
				} else if( setExposureMode){	// Reset Focus
					setExposureMode = false;
					byte t[] = { 'E', 'X', 'M', (byte) exposureModeSetting };
					out.write(t, 0, 4);
					
				} else if( setExpAutoMode){	// Reset Focus
					setExpAutoMode = false;
					byte t[] = { 'E', 'X', 'A', (byte) expAutoModeSetting };
					out.write(t, 0, 4);
				
				} else{
					byte t[] = { 'A', 'C', 'K' };
					out.write(t, 0, 3);
					
				}
				
			} catch (SWTException e) {
				if( e.getMessage().equals("Widget is disposed")){
					System.err.println("Caught Widget Disposed. Exiting");
					
				}else
					throw e;
				
				
			} catch (NumberFormatException e){
				printMsg("Number oops: " + e.getMessage());
			} catch (IOException e) {
				printMsg("Thread Oops. : " + e.getMessage());
				if( e.getMessage().equals("Connection reset"))
					timer.cancel();
				
			}
		}

		private void charToByte(byte[] buf2, String s) {
			char[] c = s.toCharArray();
			for( int l = 0; l < buf2.length; l++)
				buf2[l] = (byte) (l < c.length ? c[l] : ' ');
		}
	}

	private void readSocket(int length, byte[] buf) throws IOException {
		int res;
		res = 0;
		while (length != res) {
			if (in.available() > 0) {
				int r = in.read(buf, res, length - res);
				if (r > 0)
					res += r;

			}
		}
	}

	private void handleConnectionRequest(TabItem selected) {
		printMsg(selected.getText());

		// Don't disconnect if this is the current camera
		if (currentCamera.equals(selected.getText()))
			return;

		// Disconnect if we are connected to a stream
		closeConnection();

		currentCamera = selected.getText();

		// Allow the widget to shrink.
		if(currentTab != null){
			Canvas c = (Canvas)currentTab.getControl().getData();
			GridData g= (GridData)c.getLayoutData();		
			g.minimumHeight = 0;
			g.minimumWidth = 0;
			c.setLayoutData(g);
			
		}
		
		// Just the settings page. Don't connect.
		if (currentCamera.equals("Settings")){
			folder.getShell().pack();
			return;
		}
		
		Integer port = (Integer) selected.getData();
		try {
			currentTab = selected;

			socket = new Socket(hostname, startPort + port);
			out = socket.getOutputStream();
			in = socket.getInputStream();

			printMsg("Connected");

			timer = new Timer();
			timer.scheduleAtFixedRate(new grabFramesTask(), 0, 50);

		} catch (ConnectException e) {
			if (e.getMessage().equals("Connection refused"))
				printMsg("Connection Refused");
			else
				e.printStackTrace();

		} catch (UnknownHostException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		Canvas c = canv;
		GridData g= (GridData)c.getLayoutData();		
		g.minimumHeight = height;
		g.minimumWidth = width;
		c.setLayoutData(g);
		
		c = canv2;
		g= (GridData)c.getLayoutData();		
		g.minimumHeight = height;
		g.minimumWidth = width;
		c.setLayoutData(g);
		
		folder.getShell().pack();
	}

	public void setTab(String camera) {
		int item = Integer.parseInt(camera);
		folder.setSelection(item);
		handleConnectionRequest(folder.getItem(item));
	}

	public void printMsg(final String s) {
		System.out.println(s);
//		this.getDisplay().asyncExec(new Runnable(){@Override
//			public void run() {
//			if(outputMethod != null)
//				
//				try {
//					outputMethod.invoke(viewClass, s);
//				} catch (IllegalArgumentException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				} catch (IllegalAccessException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				} catch (InvocationTargetException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
//
//		}});
	}
	
//	public void setErrorOut(Method meth, View view) {
//		outputMethod = meth;
//		viewClass = view;
//	}
}
