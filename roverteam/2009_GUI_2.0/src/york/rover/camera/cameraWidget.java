package york.rover.camera;

/**
 * Camera Client widget for MarsRover Project @ York
 * Author: Bart Verzijlenberg
 * Date: Mar 12, 2009.
 */

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.ConnectException;
import java.net.Socket;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;

import org.eclipse.swt.SWT;
import org.eclipse.swt.SWTException;
import org.eclipse.swt.events.KeyEvent;
import org.eclipse.swt.events.KeyListener;
import org.eclipse.swt.events.MouseEvent;
import org.eclipse.swt.events.MouseListener;
import org.eclipse.swt.events.MouseWheelListener;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.graphics.GC;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.graphics.ImageData;
import org.eclipse.swt.graphics.Point;
import org.eclipse.swt.graphics.Rectangle;
import org.eclipse.swt.graphics.Transform;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.opengl.GLCanvas;
import org.eclipse.swt.opengl.GLData;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Canvas;
import org.eclipse.swt.widgets.Combo;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.MessageBox;
import org.eclipse.swt.widgets.Slider;
import org.eclipse.swt.widgets.TabFolder;
import org.eclipse.swt.widgets.TabItem;
import org.eclipse.swt.widgets.Text;
import org.lwjgl.LWJGLException;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GLContext;
import org.lwjgl.util.glu.GLU;

import view.View;

public class cameraWidget extends Composite {

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
				printMsg(new String("ack" + req_width + " " + req_height + " " + (2 * fps)));
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

		private void renderWithGL(final ImageData im) {

			byte[] b = unpackedPixels.array();

			// Of course, one is RGB the other BGR.....
			for (int i = 0; i < im.data.length; i += 3) {
				b[i] = im.data[i + 2];
				b[i + 1] = im.data[i + 1];
				b[i + 2] = im.data[i + 0];

			}

			((GLCanvas) canvas).setCurrent();
			try {
				GLContext.useContext((GLCanvas) canvas);
			} catch (LWJGLException e) {
				e.printStackTrace();
			}

			GL11.glClear(GL11.GL_COLOR_BUFFER_BIT | GL11.GL_DEPTH_BUFFER_BIT);
			GL11.glLoadIdentity();
			GL11.glEnable(GL11.GL_TEXTURE_2D);
			GL11.glBindTexture(GL11.GL_TEXTURE_2D, textid.get(0));
			GL11.glTexImage2D(GL11.GL_TEXTURE_2D, 0, GL11.GL_RGB, im.width, im.height, 0, GL11.GL_RGB, GL11.GL_UNSIGNED_BYTE, unpackedPixels);

			// Draw
			GL11.glTranslatef(-0.0f, 0.0f, zoom_dist);
			GL11.glColor4f(1.0f, 1.0f, 1.0f, 0.0f);

			GL11.glPushMatrix();
			if (rotation45)
				GL11.glRotatef(-45, 0f, 0f, 1f);

			GL11.glBegin(GL11.GL_QUADS);
			GL11.glTexCoord2f(flip_h ? 1 : 0, flip_v ? 0 : 1);
			GL11.glVertex2d(-4, -3);
			GL11.glTexCoord2f(flip_h ? 1 : 0, flip_v ? 1 : 0);
			GL11.glVertex2d(-4, 3);
			GL11.glTexCoord2f(flip_h ? 0 : 1, flip_v ? 1 : 0);
			GL11.glVertex2d(4, 3);
			GL11.glTexCoord2f(flip_h ? 0 : 1, flip_v ? 0 : 1);
			GL11.glVertex2d(4, -3);
			GL11.glEnd();

			GL11.glPopMatrix();

			if (telescopeMode) {
				GL11.glDisable(GL11.GL_TEXTURE_2D);
				GL11.glColor3f(1.0f, 0.0f, 0.0f);

				GL11.glBegin(GL11.GL_LINES);
				GL11.glVertex2d(-4, 3 * offset_y);
				GL11.glVertex2d(4, 3 * offset_y);
				GL11.glEnd();

				GL11.glBegin(GL11.GL_LINES);
				GL11.glVertex2d(4 * offset_x, 3 * offset_y);
				GL11.glVertex2d(-2, -2);
				GL11.glEnd();

				GL11.glBegin(GL11.GL_LINES);
				GL11.glVertex2d(4 * offset_x, -3);
				GL11.glVertex2d(4 * offset_x, 3);
				GL11.glEnd();

			}

			((GLCanvas) canvas).swapBuffers();
		}

		private void renderWithSWT(final ImageData im) {
			if (!rotation45 || im == null) {
				if (forcedSize)
					t = new Transform(display, (float) ((forced_width / (double) im.width) * (flip_h ? (-1) : 1)), 0, 0, (float) ((forced_height / (double) im.height) * (flip_v ? (-1)
							: 1)), flip_h ? width : 0, flip_v ? height : 0);
				else
					t = new Transform(display, (flip_h ? (-1) : 1), 0, 0, (flip_v ? (-1) : 1), flip_h ? width : 0, flip_v ? height : 0);

			} else {

				// TODO. Totally Screwy. Works for the specific case we need,
				// but not in general.
				double rotation = -Math.PI / 4;
				float cs = (float) Math.cos(rotation);
				float s = (float) Math.sin(rotation);

				double w = (width / 2) * (flip_h ? (-cs) : cs) + (height / 2) * (flip_h ? (s) : -s);
				double h = (width / 2) * (flip_h ? (-s) : s) + (height / 2) * (flip_h ? (-cs) : cs);
				t = new Transform(folder.getDisplay(), flip_h ? (-cs) : cs, flip_h ? (s) : -s, flip_h ? (-s) : s, flip_h ? (-cs) : cs, flip_h ? (float) (-w)
						: 0, (float) (height + 2 * h));
			}

			GC gc = new GC(canvas);
			gc.setTransform(t);

			Image a = new Image(display, im);
			gc.drawImage(a, 0, 0);

			a.dispose();
			gc.dispose();
		}

		public void run() {

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
					if(saveImage){
						System.arraycopy(jpg_buf, 0, saved_image, 0, length);
						saveImage = false;
					}
					
					// Send an acknowledgement
					handleAcknowledgement();

					// ===================================
					// Decode image.
					InputStream in = new ByteArrayInputStream(jpg_buf);
					final ImageData im = new ImageData(in);

					// Scale Image
					if (forcedSize) {
						height = forced_height;
						width = forced_width;
					} else {
						height = im.height;
						width = im.width;
					}

					frame_count++;

					display.syncExec(new Runnable() {
						@Override
						public void run() {

							if (!canvas.isDisposed()) {

								// ===========================
								// Render Image
								if (renderModeOpenGL)
									renderWithGL(im);
								else
									renderWithSWT(im);

								// ===========================
								// Change layout if needed
								redoLayout();

								// ============================
								// Make sure we don't overwrite the focus value
								// if we are setting a new one
								if (!sendFocusVal) {
									focus = 0x000000FF & ((int) c[0]);

									Composite comp = (Composite) currentTab.getControl();
									Slider s = (Slider) comp.getChildren()[6];
									s.setSelection(focus);

								}

								// ============================
								// Record FPS and Mbps
								double diff = System.currentTimeMillis() - lastTime;
								if (diff > 1000) {

									Composite comp = (Composite) currentTab.getControl();
									Label s = (Label) comp.getChildren()[4];

									String t = Double.toString((frame_count / diff * 1000));
									if (t.length() > 5)
										t = t.substring(0, 5);

									String t2 = Double.toString((byte_count * 8.0 / 1000000.0 / diff * 1000));
									if (t2.length() > 5)
										t2 = t2.substring(0, 5);

									s.setText(t + "/" + t2);

									lastTime = System.currentTimeMillis();
									frame_count = 0;
									byte_count = 0;
								}

							}
						}
					});
				} catch (SWTException e) {
					if (e.getMessage().equals("Widget is disposed")) {
						System.err.println("Caught Widget Disposed. Exiting");
						closeConnection();
						System.exit(1);
					} else if (e.getMessage().equals("Invalid image")) {
						printMsg(e.getMessage());

					} else if (e.getMessage().equals("Unsupported or unrecognized format")) {
						System.err.println("Bad Image Format");

					} else
						throw e;

				} catch (NumberFormatException e) {
					printMsg("Number oops: " + e.getMessage());

				} catch (IOException e) {
					printMsg("Thread Oops. : " + e.getMessage());
					return;
				} catch (Exception e) {
					e.printStackTrace();

				}
			}
		}
	}

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

	private Object telescopeConnectionSync = new Object();
	private TabFolder folder;
	private Display display;
	private String currentCamera = "";
	private TabItem currentTab;

	// Connection members
	private Socket socket = null;
	private OutputStream out = null;
	private InputStream in = null;
	private ByteBuffer unpackedPixels = ByteBuffer.allocate(960 * 720 * 3);

	/**
	 * READ_ONLY!!!!!!
	 */
	private byte[] jpg_buf = new byte[960 * 720 * 3];
	private Socket telescope_socket = null;
	private OutputStream telescope_out = null;
	private InputStream telescope_in = null;

	private IntBuffer textid;

	private String[] names = { "bird-front", "bird-left", "bird-right", "bird-back", "base-left", "base-right", "stereo-left", "stereo-right", "arm-top", "arm-end",
			"arm-misc" };

	private int deltaX;
	private int deltaY;
	private int width = 640;
	private int height = 480;
	private int fps = 30;
	private int req_width = width;
	private int req_height = height;
	private int focus = 1;
	private int maxFocusSetting = 255;
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

	private Transform t;

	private long lastTime = 0;
	private int frame_count = 0;

	// Configuration settings
	private int startPort = 30100;
	private int numberOfCameras = names.length;

	private boolean telescopeMode = false;
	private float zoom_dist = -7.3f;
	private grabFramesTask grabThread;

	private boolean renderModeOpenGL = false;
	private Button renderMode;

	private cameraWidget widget = this;

	private Composite canvas;
	private Method outputMethod = null;
	private View viewClass;

	public cameraWidget(Composite parent, int style) {
		super(parent, style);
		display = getDisplay();
		setLayout(new GridLayout(1, false));
		init();

	}

	public cameraWidget(Composite parent, int style, int start, int numCams) {
		super(parent, style);
		display = getDisplay();
		startPort = start;
		numberOfCameras = numCams;

		setLayout(new GridLayout(1, false));
		init();
	}

	public cameraWidget(Composite parent, int style, int start, int numCams, boolean telescopeMode) {
		super(parent, style);
		display = getDisplay();
		startPort = start;
		numberOfCameras = numCams;
		this.telescopeMode = telescopeMode;
		setLayout(new GridLayout(1, false));
		init();
	}

	private void addNormalMoveCode(Composite canv) {
		canv.addKeyListener(new KeyListener() {

			@Override
			public void keyPressed(KeyEvent arg0) {

				if (arg0.keyCode == SWT.ARROW_LEFT) {
					deltaX = 5;
					sendPanTilt = true;
				} else if (arg0.keyCode == SWT.ARROW_RIGHT) {
					deltaX = -5;
					sendPanTilt = true;
				} else if (arg0.keyCode == SWT.ARROW_UP) {
					deltaY = -5;
					sendPanTilt = true;
				} else if (arg0.keyCode == SWT.ARROW_DOWN) {
					deltaY = 5;
					sendPanTilt = true;
				}

			}

			@Override
			public void keyReleased(KeyEvent arg0) {
				// TODO Auto-generated method stub

			}
		});
	}

	private void addSliders(Composite comp) {
		// -------------------------------------------
		// Add Focus Slider
		Label l = new Label(comp, SWT.NONE);
		l.setText("Focus Setting");
		l.setLayoutData(new GridData());

		final Slider slider = new Slider(comp, SWT.HORIZONTAL);
		slider.setIncrement(1);
		slider.setMaximum(maxFocusSetting);
		slider.setMinimum(0);
		slider.setLayoutData(new GridData(SWT.FILL, SWT.LEFT, true, false, 4, 1));
		slider.addListener(SWT.Selection, new Listener() {
			public void handleEvent(Event arg0) {
				focus = slider.getSelection();
				sendFocusVal = true;
			}
		});

		if (telescopeMode) {
			// -------------------------------------------
			// Add Focus Slider
			l = new Label(comp, SWT.NONE);
			l.setText("Focus Setting");
			l.setLayoutData(new GridData());

			final Slider slider2 = new Slider(comp, SWT.HORIZONTAL);
			slider2.setIncrement(1);
			slider2.setMaximum(3900);
			slider2.setMinimum(0);
			slider2.setLayoutData(new GridData(SWT.FILL, SWT.LEFT, true, false, 4, 1));
			slider2.addListener(SWT.Selection, new Listener() {
				public void handleEvent(Event arg0) {

					try {
						synchronized (telescopeConnectionSync) {
							if (connectTelescope()) {
								int setting = slider2.getSelection();

								byte t[] = { 'T', 'S', 'F', 'C' };
								telescope_out.write(t, 0, 4);
								telescope_out.write(intToByteArray(setting), 0, 4);
								System.out.println("Focus to " + setting);
								disconnectTelecope();
							}
						}
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}

				}
			});
		}

		// -------------------------------------------
		// Add Contrast Slider
		final Button exposureButton = new Button(comp, SWT.CHECK);
		exposureButton.setText("Man. Exp.");
		exposureButton.setLayoutData(new GridData());
		exposureButton.addSelectionListener(new SelectionListener() {

			public void widgetDefaultSelected(SelectionEvent arg0) {
			}

			public void widgetSelected(SelectionEvent arg0) {
				exposureModeSetting = exposureButton.getSelection() ? '1' : '3';
				setExposureMode = true;

			}
		});

		final Slider exposureSlider = new Slider(comp, SWT.HORIZONTAL);
		exposureSlider.setIncrement(1);
		exposureSlider.setMaximum(10000);
		exposureSlider.setMinimum(0);
		exposureSlider.setLayoutData(new GridData(SWT.FILL, SWT.LEFT, true, false, 4, 1));
		exposureSlider.addListener(SWT.Selection, new Listener() {
			public void handleEvent(Event arg0) {
				exposureSetting = exposureSlider.getSelection();
				setExposure = true;

			}
		});
	}

	private void addStandardControls(Composite comp) {
		// -------------------------------------------
		// Add frame size selector
		Combo combo = new Combo(comp, SWT.READ_ONLY);
		combo.setItems(new String[] { "160x120", "320x240", "640x480", "800x600", "960x720" });
		combo.setSize(200, 200);
		combo.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
		combo.addSelectionListener(new SelectionListener() {

			public void widgetDefaultSelected(SelectionEvent arg0) {
			}

			public void widgetSelected(SelectionEvent arg0) {
				int sel = ((Combo) arg0.widget).getSelectionIndex();
				String s = ((Combo) arg0.widget).getItem(sel);

				req_width = Integer.parseInt(s.split("x")[0]);
				req_height = Integer.parseInt(s.split("x")[1]);

				sendSettings = true;

			}
		});

		// -------------------------------------------
		// Add FPS selector
		Combo combo2 = new Combo(comp, SWT.READ_ONLY);
		combo2.setItems(new String[] { "5", "10", "15", "20", "25", "30" });
		combo2.setSize(200, 200);
		combo2.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
		combo2.addSelectionListener(new SelectionListener() {
			public void widgetDefaultSelected(SelectionEvent arg0) {
			}

			public void widgetSelected(SelectionEvent arg0) {
				int sel = ((Combo) arg0.widget).getSelectionIndex();

				if (fps != (sel + 1) * 5) {
					fps = (sel + 1) * 5;
					sendSettings = true;
				}
			}
		});

		// -------------------------------------------
		// Add Pan and tit reset
		Button resetPanTiltButton = new Button(comp, SWT.PUSH);
		resetPanTiltButton.setText("Reset PanTilt");
		resetPanTiltButton.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
		resetPanTiltButton.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetDefaultSelected(SelectionEvent arg0) {
			}

			@Override
			public void widgetSelected(SelectionEvent arg0) {
				resetPanTilt = true;
			}
		});

		// -------------------------------------------
		// Add Auto-Focus button
		Button doFocusButton = new Button(comp, SWT.PUSH);
		doFocusButton.setText("Do auto-focus");
		doFocusButton.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
		doFocusButton.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetDefaultSelected(SelectionEvent arg0) {
			}

			@Override
			public void widgetSelected(SelectionEvent arg0) {
				sendDoFocus = true;
			}
		});

		// // -------------------------------------------
		// // Add Set Infinite Focus
		// Button infFocusButton = new Button(comp, SWT.PUSH);
		// infFocusButton.setText("Infinite Focus");
		// infFocusButton.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false,
		// false, 1, 1));
		// infFocusButton.addSelectionListener(new SelectionListener() {
		//
		// @Override
		// public void widgetDefaultSelected(SelectionEvent arg0) {
		// }
		//
		// @Override
		// public void widgetSelected(SelectionEvent arg0) {
		// resetFocus = true;
		// }
		// });

		Label lbl_fps = new Label(comp, SWT.NONE);
		lbl_fps.setText("? FPS");
		GridData g = new GridData(SWT.FILL, SWT.CENTER, true, false, 1, 1);
		lbl_fps.setLayoutData(g);
	}

	private void addTelescopeControls(Composite comp) {
		Button stopButton = new Button(comp, SWT.PUSH);
		stopButton.setBackground(new Color(stopButton.getDisplay(), 255, 0, 0));
		stopButton.setText("STOP TELESCOPE");
		stopButton.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
		stopButton.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetDefaultSelected(SelectionEvent arg0) {
			}

			@Override
			public void widgetSelected(SelectionEvent arg0) {
				try {
					synchronized (telescopeConnectionSync) {
						if (connectTelescope()) {
							System.out.println("Sending Stop Command");
							byte t[] = { 'S', 'T', 'O', 'P' };
							telescope_out.write(t, 0, 4);
							disconnectTelecope();
						}
					}
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		});

		Button setZeroPosition = new Button(comp, SWT.PUSH);
		setZeroPosition.setText("Set Zero Position");
		setZeroPosition.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
		setZeroPosition.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetDefaultSelected(SelectionEvent arg0) {
			}

			@Override
			public void widgetSelected(SelectionEvent arg0) {
				try {
					synchronized (telescopeConnectionSync) {
						if (connectTelescope()) {

							System.out.println("Resetting Zero Position");
							byte t[] = { 'Z', 'E', 'R', 'O' };
							telescope_out.write(t, 0, 4);

							disconnectTelecope();
						}
					}
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		});

		Button gotoZeroPosition = new Button(comp, SWT.PUSH);
		gotoZeroPosition.setText("Goto Zero Position");
		gotoZeroPosition.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
		gotoZeroPosition.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetDefaultSelected(SelectionEvent arg0) {
			}

			@Override
			public void widgetSelected(SelectionEvent arg0) {
				try {
					synchronized (telescopeConnectionSync) {
						if (connectTelescope()) {

							System.out.println("Return to 0 Position");
							byte t[] = { 'R', 'E', 'T', '0' };
							telescope_out.write(t, 0, 4);

							disconnectTelecope();
						}
					}
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		});

		Button getPosition = new Button(comp, SWT.PUSH);
		getPosition.setText("Get Position");
		getPosition.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 1, 1));
		getPosition.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetDefaultSelected(SelectionEvent arg0) {
			}

			@Override
			public void widgetSelected(SelectionEvent arg0) {
				try {
					synchronized (telescopeConnectionSync) {
						if (connectTelescope()) {
							byte t[] = { 'G', 'P', 'O', 'S' };
							telescope_out.write(t, 0, 4);

							byte c[] = new byte[4];
							byte c2[] = new byte[4];

							readSocket(telescope_in, 4, c);
							readSocket(telescope_in, 4, c2);

							int a = byteArrayToInt(c);
							int b = byteArrayToInt(c2);

							System.out.println("Getting Position " + a + b);

							disconnectTelecope();
						}
					}

				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		});

	}

	public static int byteArrayToInt(byte[] b) {
		int value = 0;
		for (int i = 0; i < 4; i++) {
			int shift = (i) * 8;
			value += (b[i] & 0x000000FF) << shift;
		}
		return value;
	}

	private void addTelescopeMoveCode(Composite canv) {
		canv.addMouseListener(new MouseListener() {

			public void mouseDoubleClick(MouseEvent arg0) {
			}

			public void mouseDown(MouseEvent arg0) {
				if (arg0.button != 1)
					return;
				Point s = ((Composite) arg0.widget).getSize();

				int distfromCenter_x = (int) (arg0.x - (s.x / 2 + offset_x * width / 2.0));
				int distfromCenter_y = (int) (arg0.y - (s.y / 2 + -offset_y * height / 2.0));

				System.out.println(s.x + " " + s.y);

				// Need the angle to move by, based on where we clicked
				// Screen is WIDTH wide, which corresponds to a THETA angular
				// FOV
				// Each pixel is then a THETA/WIDTH anglular step

				double azimuth = distfromCenter_x * (x_fov / (double) width);
				double altitude = distfromCenter_y * (y_fov / (double) height);

				int a = (int) (azimuth / 360.0 * 4294967296.0); // 2^24
				int b = (int) (altitude / 360.0 * 4294967296.0);

				System.out.println("Moving" + azimuth + " " + altitude + " " + a + " " + b);

				try {
					synchronized (telescopeConnectionSync) {
						if (connectTelescope()) {

							if (telescopeFast) {
								byte t[] = { 'F', 'O', 'V', 'R' };
								telescope_out.write(t, 0, 4);

							} else {
								byte t[] = { 'M', 'O', 'V', 'R' };
								telescope_out.write(t, 0, 4);

							}

							telescope_out.write(intToByteArray(a), 0, 4);
							telescope_out.write(intToByteArray(b), 0, 4);

							disconnectTelecope();
						}
					}
				} catch (IOException e) {
					e.printStackTrace();

				}
			}

			public void mouseUp(MouseEvent arg0) {
			}
		});
	}

	private void charToByte(byte[] buf2, String s) {
		char[] c = s.toCharArray();
		for (int l = 0; l < buf2.length; l++)
			buf2[l] = (byte) (l < c.length ? c[l] : ' ');
	}

	private void closeConnection() {

		System.out.println("Closing Connection");

		if (grabThread != null) {
			grabThread.alive = false;
			grabThread = null;
		}

		if (socket != null) {
			try {
				out.close();
				in.close();
				socket.close();
				printMsg("Closed Connection");

			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		if (telescopeMode)
			disconnectTelecope();
	}

	private boolean connectTelescope() {
		try {
			System.out.println("Connecting to Telescope");
			telescope_socket = new Socket(hostname, 30030);
			telescope_out = telescope_socket.getOutputStream();
			telescope_in = telescope_socket.getInputStream();

		} catch (ConnectException e) {
			if (e.getMessage().equals("Connection refused"))
				printMsg("Telescope Connection Refused");
			else
				e.printStackTrace();
			return false;

		} catch (UnknownHostException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		return true;
	}

	private Composite createSettingsPage() {
		Composite cSettings = new Composite(folder, SWT.NONE);
		cSettings.setLayout(new GridLayout(4, false));

		// =========================================
		Label l = new Label(cSettings, SWT.NONE);
		l.setText("Server");
		l.setLayoutData(new GridData());

		final Text host = new Text(cSettings, SWT.SINGLE | SWT.BORDER);
		host.setText(hostname);
		GridData gd = new GridData();
		gd.widthHint = 200;
		host.setLayoutData(gd);

		// =========================================
		l = new Label(cSettings, SWT.NONE);
		l.setText("Number of Cameras");
		l.setLayoutData(new GridData());

		final Text numCamera = new Text(cSettings, SWT.SINGLE | SWT.BORDER);
		numCamera.setText(Integer.toString(numberOfCameras));
		gd = new GridData();
		gd.widthHint = 50;
		numCamera.setLayoutData(gd);

		// =========================================
		l = new Label(cSettings, SWT.NONE);
		l.setText("First Port");
		l.setLayoutData(new GridData());

		final Text firstPort = new Text(cSettings, SWT.SINGLE | SWT.BORDER);
		firstPort.setText(Integer.toString(startPort));
		gd = new GridData();
		gd.widthHint = 200;
		firstPort.setLayoutData(gd);

		renderMode = new Button(cSettings, SWT.CHECK);
		renderMode.setLayoutData(new GridData());
		renderMode.setText("Render OpenGL");
		renderMode.addSelectionListener(new SelectionListener() {

			public void widgetDefaultSelected(SelectionEvent arg0) {
			}

			public void widgetSelected(SelectionEvent arg0) {

				System.out.println(renderMode.getSelection());
				renderModeOpenGL = renderMode.getSelection();

				Control[] c = widget.getChildren();
				for (Control a : c)
					a.dispose();

				init();

				renderMode.setSelection(renderModeOpenGL);
			}
		});

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

	private void disconnectTelecope() {
		if (telescope_socket != null) {
			try {
				telescope_out.close();
				telescope_in.close();
				telescope_socket.close();
				printMsg("Closed Telescope Connection");
			} catch (IOException e) {
				e.printStackTrace();

			}
		}
	}

	@Override
	public void dispose() {
		System.out.println("Disposing of CameraWidget");

		super.dispose();

		// make sure we close cleanly
		closeConnection();

	}

	private void handleConnectionRequest(TabItem selected) {
		printMsg("Selected Camera Tab: " + selected.getText());

		try {
			// Don't disconnect if this is the current camera
			if (currentCamera.equals(selected.getText()))
				return;

			// Disconnect if we are connected to a stream
			closeConnection();

			currentCamera = selected.getText();

			// Force the widget to shrink.
			if (currentTab != null) {
				GridData g = (GridData) canvas.getLayoutData();
				g.minimumHeight = 0;
				g.minimumWidth = 0;
				canvas.setLayoutData(g);

			}

			// Just the settings page. Don't connect.
			if (currentCamera.equals("Settings")) {
				folder.getShell().pack();
				return;
			}

			printMsg("---- Connecting ----");
			currentTab = selected;

			socket = new Socket(hostname, startPort + (Integer) selected.getData());
			out = socket.getOutputStream();
			in = socket.getInputStream();

			printMsg("Connected");

			// Cancel all settings changes
			sendPanTilt = false;
			sendSettings = false;
			sendDoFocus = false;
			sendFocusVal = true;
			resetPanTilt = false;
			resetFocus = false;

			// Start a new frame grabber
			// getDisplay().asyncExec(new grabFramesTask());
			grabThread = new grabFramesTask();
			new Thread(grabThread).start();

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
	}

	private void init() {
		// Add a tabFolder
		folder = new TabFolder(this, SWT.NONE);
		folder.setLayoutData(new GridData(SWT.CENTER, SWT.CENTER, true, true));

		TabItem settings = new TabItem(folder, SWT.NONE);
		settings.setText("Settings");
		Composite cSettings = createSettingsPage();
		settings.setControl(cSettings);

		folder.addSelectionListener(new SelectionListener() {

			public void widgetDefaultSelected(SelectionEvent arg0) {
			}

			public void widgetSelected(SelectionEvent arg0) {
				handleConnectionRequest((TabItem) arg0.item);

			}
		});

		initTabs();

		if (renderModeOpenGL)
			setupOpenGLCanvas();
		else
			setupCanvas();

		if (telescopeMode) {
			addTelescopeMoveCode(canvas);
		} else {
			addNormalMoveCode(canvas);
		}

		folder.pack();

	}

	private void initTabs() {

		for (int i = folder.getItemCount() - 1; i < numberOfCameras; i++) {

			TabItem camTab = new TabItem(folder, SWT.NONE);

			int n = (startPort + i) - 30100;
			if (n < names.length)
				camTab.setText(names[n]);
			else
				camTab.setText("noName");
			camTab.setData(new Integer(i));

			// -------------------------------------------
			// Create a composite
			Composite comp = new Composite(folder, SWT.NONE);
			comp.setLayout(new GridLayout(5, false));
			camTab.setControl(comp);

			addStandardControls(comp);
			addSliders(comp);
			if (telescopeMode)
				addTelescopeControls(comp);

		}
	}

	private byte[] intToByteArray(final int integer) {
		int byteNum = 4;
		byte[] byteArray = new byte[4];

		for (int n = 0; n < byteNum; n++)
			byteArray[n] = (byte) (integer >>> (n * 8));

		return (byteArray);
	}

	public void printMsg(final String s) {
		System.out.println(s);

		try {
			this.getDisplay().asyncExec(new Runnable() {
				@Override
				public void run() {
					if (outputMethod != null)

						try {
							outputMethod.invoke(viewClass, s);
						} catch (IllegalArgumentException e) {
							System.out.println("Caught Exception for printmsg. Taking care of it");
							outputMethod = null;
						} catch (IllegalAccessException e) {
							System.out.println("Caught Exception for printmsg. Taking care of it");
							outputMethod = null;
						} catch (InvocationTargetException e) {
							System.out.println("Caught Exception for printmsg. Taking care of it");
							outputMethod = null;
						}

				}
			});
		} catch (SWTException e) {
			if (e.getMessage().equals("Widget is disposed"))// TODO
				// Auto-generated
				// catch block
				System.exit(1);
			else
				throw e;
		}
	}

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

	private void redoLayout() {
		GridData g = (GridData) canvas.getLayoutData();
		if (canvas.getSize().y != height) {
			g.minimumHeight = height;
			g.minimumWidth = width;
			canvas.setLayoutData(g);

			System.out.println(width + " " + height);

			canvas.getParent().pack();
			folder.getShell().pack();
			folder.getShell().layout();

		}
	}

	public void setCamTabName(String arg, int i) {
		folder.getItem(i).setText(arg);
	}

	public void setErrorOut(Method meth, View view) {
		outputMethod = meth;
		viewClass = view;
	}

	public void setTab(String camera) {
		int item = Integer.parseInt(camera);
		folder.setSelection(item);
		handleConnectionRequest(folder.getItem(item));
	}

	private void setupCanvas() {
		canvas = new Canvas(this, SWT.NONE);
		canvas.setLayoutData(new GridData(SWT.CENTER, SWT.CENTER, true, true, 6, 1));
		this.setData(canvas);

	}

	private void setupOpenGLCanvas() {
		// ==================================================
		GLData data = new GLData();
		data.doubleBuffer = true;

		canvas = new GLCanvas(this, SWT.BORDER, data);
		GridData g = new GridData(SWT.CENTER, SWT.CENTER, true, true, 6, 1);
		canvas.setLayoutData(g);

		this.setData(canvas);

		((GLCanvas) canvas).setCurrent();
		try {
			GLContext.useContext(canvas);
		} catch (LWJGLException e) {
			e.printStackTrace();
		}

		canvas.addListener(SWT.Resize, new Listener() {
			public void handleEvent(Event event) {

				System.out.println("OpenGL Resizing");
				Rectangle bounds = canvas.getBounds();
				System.out.println(bounds.height + " " + bounds.width);
				float fAspect = (float) bounds.width / (float) bounds.height;

				((GLCanvas) canvas).setCurrent();

				try {
					GLContext.useContext(canvas);
				} catch (LWJGLException e) {
					e.printStackTrace();
				}

				GL11.glViewport(0, 0, bounds.width, bounds.height);
				GL11.glMatrixMode(GL11.GL_PROJECTION);
				GL11.glLoadIdentity();

				GLU.gluPerspective(45.0f, fAspect, 0.5f, 400.0f);
				GL11.glMatrixMode(GL11.GL_MODELVIEW);
				GL11.glLoadIdentity();

			}
		});

		textid = IntBuffer.allocate(1);

		GL11.glClearColor((float) ((double) folder.getBackground().getRed() / 255.0), (float) ((double) folder.getBackground().getGreen() / 255.0), (float) ((double) folder.getBackground().getBlue() / 255.0), 1.0f);

		GL11.glColor3f(1.0f, 0.0f, 0.0f);
		GL11.glHint(GL11.GL_PERSPECTIVE_CORRECTION_HINT, GL11.GL_NICEST);
		GL11.glClearDepth(1.0);
		GL11.glEnable(GL11.GL_DEPTH_TEST);
		GL11.glEnable(GL11.GL_TEXTURE_2D);
		GL11.glGenTextures(textid);
		GL11.glBindTexture(GL11.GL_TEXTURE_2D, textid.get(0));
		GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MIN_FILTER, GL11.GL_LINEAR);
		GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MAG_FILTER, GL11.GL_LINEAR);

		canvas.addMouseWheelListener(new MouseWheelListener() {
			@Override
			public void mouseScrolled(MouseEvent e) {

				zoom_dist += e.count / 10.0;
				if (zoom_dist > 0)
					zoom_dist = -0.1f;

			}
		});

		canvas.addMouseListener(new MouseListener() {
			public void mouseDoubleClick(MouseEvent e) {
			}

			public void mouseDown(MouseEvent e) {
				if (e.button == 3) {
					zoom_dist = -7.3f;
					return;
				}
			}

			public void mouseUp(MouseEvent e) {
			}

		});
	}
}