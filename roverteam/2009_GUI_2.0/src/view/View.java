package view;
/**
 * Control Station GUI Client for Mars Rover Project @ York
 * Author: Stanley Diji
 * Date: currently being updated
 * Version: 1.1
 * dijistanley@gmail.com
 */

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.lang.reflect.Method;

import org.eclipse.swt.SWT;
import org.eclipse.swt.custom.StyledText;
import org.eclipse.swt.events.MouseAdapter;
import org.eclipse.swt.events.MouseEvent;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.TabFolder;
import org.eclipse.swt.widgets.TabItem;
import org.eclipse.swt.widgets.Text;

import processControl.processControlTabs;
//import roverControl.RoverMoveThread;
import roverControl.wirelessWidget;
import tcp.TCPClient;
import york.rover.camera.cameraWidget;
//import armControl.ArmThread;
import controls.Control;
import controls.Controls;
import controls.stereoControl;

public class View implements ActionListener 
{
	// //TODO: Add values ip and port
	private Button assign_ipButton;
	public Text b3_speed;
	public Text b2_speed;
	public Text b8_speed;
	public Text b1_speed;
	public Text b0_speed;
	private Text ip_text;
	private Label controlStat;
	private StyledText exceptionViewer;
	public final String FRONT_CAMERA = "1";
	public final String BACK_CAMERA = "2";
	public final String LEFT_CAMERA = "3";
	public final String RIGHT_CAMERA = "4";
	public final String ARM_CAMERA = "5";
	private final int INTERVAL = 100;
	private final int ARM_INTERVAL = 100;
	private final double DEADZONE = 0.05;
	private final double ARM_DEADZONE = 0.5;

//	public RoverMoveThread Accelerate;
//	public ArmThread Arm;

	// TODO: set port value for arm and rover movement
	public final int MOVE_PORT = 30001, ARM_PORT = 30010;
	// TODO: set rover ip
	public String ROVERIP = "192.168.3.1";//"nanosat.esse.yorku.ca";
	public final int POWERPORT = 0;
	public final int POWER_ID = 4;

	public final int PC1_ID = 1, ON = 0, OFF = 1;
	public final int PC2_ID = 2;
	public final int PC3_ID = 3;

	public StackTraceElement ste;

	private static Shell shell;

	private cameraWidget cameraWidget;

	private boolean color = true;

	private Button controlMapButton;

	public processControlTabs processController;
	
	
	private final int MAX_ARM_SPEED = 10;
	private final int MIN_ARM_SPEED = 0;
	
	public View view = this;
	private wirelessWidget wireless1;
	private wirelessWidget wireless2;
	
	public double b0speed, b1speed, b2speed, b3speed, b8speed;
	
	
	/**
	 * Create contents of the application window
	 * 
	 * @param parent
	 */
	protected Composite createContents(final Shell parent) {

		Composite container = new Composite(parent, SWT.NONE);
		container.setLayout(new GridLayout(2, false));
		container.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));

		if (color)
			container.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_TITLE_BACKGROUND));

		TabFolder tabFolder = new TabFolder(container, SWT.NONE);

		GridData g = new GridData(SWT.FILL, SWT.FILL, false, true, 1, 4);
		g.widthHint = 422;
		g.heightHint = 266;
		tabFolder.setLayoutData(g);
		tabFolder.addSelectionListener(new SelectionListener() {
			@Override
			public void widgetSelected(SelectionEvent arg0) {
				if (shell != null) {
					shell.pack();
					shell.layout();
				}
			}

			public void widgetDefaultSelected(SelectionEvent arg0) {
			}
		});

		if (color)
			tabFolder.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_GREEN));

		createCameraComposite(container);
		createWLanSection(container);
		createExceptionSection(container);
		createBottomBar(container);

		// createPowerTab(tabFolder);
		createControlTab(tabFolder);
		createProcessTab(tabFolder);
		createStereoTab(tabFolder);
		return container;
	}

	private void createControlTab(TabFolder tabFolder) {
		/***********************************************************************************/
		TabItem controlTabItem = new TabItem(tabFolder, SWT.NONE);
		controlTabItem.setText("Control");

		Composite comp = new Composite(tabFolder, SWT.NONE);
		comp.setLayout(new GridLayout(1, false));
		comp.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, false));
		controlTabItem.setControl(comp);

		Group roverControlGroup = new Group(comp, SWT.NONE);
		roverControlGroup.setLayout(new GridLayout(3, false));
		final GridData gd_roverControlGroup = new GridData(SWT.FILL, SWT.FILL, false, false);
		gd_roverControlGroup.widthHint = 393;
		gd_roverControlGroup.heightHint = 81;
		roverControlGroup.setLayoutData(gd_roverControlGroup);
		roverControlGroup.setText("Rover Control");

		Button roverRadio = new Button(roverControlGroup, SWT.RADIO);
		roverRadio.setLayoutData(new GridData(SWT.LEFT, SWT.TOP, false, false));
		roverRadio.setToolTipText("Click to control rover movement with joystick");
		roverRadio.setText("[R]over");

//		try {
//			Accelerate = new RoverMoveThread(this.ROVERIP, MOVE_PORT, this);
//		} catch (Exception e1) {
//			throwException(e1);
//		}

//		roverRadio.addMouseListener(new MouseAdapter() {
//			public void mouseDown(final MouseEvent e) {
//				try {
//					// if (Arm. != null)
//					Arm.stopPolling();
//					Accelerate = new RoverMoveThread(ROVERIP, MOVE_PORT, view);
//					controlStat.setText("Current Control -> Rover Movement");
//					Accelerate.setDeadZone(DEADZONE);
//					Accelerate.setPollInterval(INTERVAL);
//					Accelerate.updateDeadZone();
//					Accelerate.startPolling();
//				} catch (Exception exception) {
//					throwException(exception);
//				}
//			}
//		});
		
//				// ---------------------------------------------------------------------
				Button armRadio = new Button(roverControlGroup, SWT.RADIO);
				final GridData gd_armRadio = new GridData(SWT.LEFT, SWT.BEGINNING, false, false);
				gd_armRadio.widthHint = 66;
				armRadio.setLayoutData(gd_armRadio);
				armRadio.setToolTipText("Click to control Arm with joystick");
				armRadio.setText("[A]rm");
//				armRadio.addMouseListener(new MouseAdapter() {
//					public void mouseDown(final MouseEvent e) {
//						try {
//							// if (Accelerate != null)
//							Accelerate.stopPolling();
//							Arm = new ArmThread(ROVERIP, ARM_PORT, view);
//							controlStat.setText("Current Control -> Arm Movement");
//							Arm.setDeadZone(ARM_DEADZONE);
//							Arm.setPollInterval(ARM_INTERVAL);
//							Arm.updateDeadZone();
//							Arm.startPolling();
//
//							// System.out.println(armRadio.toString());
//						} catch (Exception exception) {
//							throwException(exception);
//							// throwException(exception.getStackTrace());
//						}
//					}
//				});
		new Label(roverControlGroup, SWT.NONE);
//		try {
//			Arm = new ArmThread(this.ROVERIP, ARM_PORT, this);
//		} catch (Exception e1) {
//			throwException(e1);
//		}
		/*****************************************************************************/

		final Group armSpeedGroup = new Group(comp, SWT.NONE);
		armSpeedGroup.setText("Arm Speed");
		armSpeedGroup.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_DARK_GREEN));
		armSpeedGroup.setLayoutData(new GridData(394, 111));
		final GridLayout gridLayout = new GridLayout();
		gridLayout.numColumns = 12;
		armSpeedGroup.setLayout(gridLayout);
		new Label(armSpeedGroup, SWT.NONE);

		final Label b0Label = new Label(armSpeedGroup, SWT.NONE);
		b0Label.setLayoutData(new GridData(SWT.LEFT, SWT.BOTTOM, false, false));
		b0Label.setText("byte[0]");
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);

		final Label b1Label = new Label(armSpeedGroup, SWT.NONE);
		b1Label.setLayoutData(new GridData(SWT.LEFT, SWT.BOTTOM, false, false));
		b1Label.setText("byte[1]");
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);

		final Label b2Label = new Label(armSpeedGroup, SWT.NONE);
		b2Label.setLayoutData(new GridData(SWT.LEFT, SWT.BOTTOM, false, false));
		b2Label.setText("byte[2]");
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);

		final Label b0Label_1 = new Label(armSpeedGroup, SWT.NONE);
		b0Label_1.setLayoutData(new GridData(SWT.LEFT, SWT.BOTTOM, false, false));
		b0Label_1.setText("byte[3]");
		new Label(armSpeedGroup, SWT.NONE);

		final Button b0Minus = new Button(armSpeedGroup, SWT.NONE);
		b0Minus.addSelectionListener(new SelectionAdapter() {
			public void widgetSelected(final SelectionEvent e) {
				double text = Double.parseDouble(b0_speed.getText()) - 2;
				text = text >= 0.0? text : 0.0;
				b0_speed.setText(text + "");
				b0speed = text;
//				Arm.updateSpeed(text);
			}
		});
		b0Minus.setLayoutData(new GridData(19, 18));
		b0Minus.setText("-");

		b0_speed = new Text(armSpeedGroup, SWT.BORDER);
		b0_speed.setText("0.0");

		final GridData gd_b0_speed = new GridData(SWT.FILL, SWT.CENTER, false, false);
		gd_b0_speed.widthHint = 26;
		b0_speed.setLayoutData(gd_b0_speed);

		final Button b0Plus = new Button(armSpeedGroup, SWT.NONE);
		b0Plus.addSelectionListener(new SelectionAdapter() {
			public void widgetSelected(final SelectionEvent e) {
				double text = Double.parseDouble(b0_speed.getText()) + 2;
				text = text <= MAX_ARM_SPEED? text : MAX_ARM_SPEED;
				b0_speed.setText(text + "");
				b0speed = text;
//				Arm.updateSpeed(text);
			}
		});
		b0Plus.setLayoutData(new GridData(20, 19));
		b0Plus.setText("+");

		final Button b1Minus = new Button(armSpeedGroup, SWT.NONE);
		b1Minus.addSelectionListener(new SelectionAdapter() {
			public void widgetSelected(final SelectionEvent e) {
				double text = Double.parseDouble(b1_speed.getText()) - 2;
				text = text >= 0.0? text : 0.0;
				b1_speed.setText(text + "");
				b1speed = text;
//				Arm.updateSpeed(text);
			}
		});
		b1Minus.setLayoutData(new GridData(19, 18));
		b1Minus.setText("-");

		b1_speed = new Text(armSpeedGroup, SWT.BORDER);
		b1_speed.setText("0.0");
		final GridData gd_b1_speed = new GridData(SWT.FILL, SWT.CENTER, false, false);
		gd_b1_speed.widthHint = 26;
		b1_speed.setLayoutData(gd_b1_speed);

		final Button b1Plus = new Button(armSpeedGroup, SWT.NONE);
		b1Plus.addSelectionListener(new SelectionAdapter() {
			public void widgetSelected(final SelectionEvent e) {
				double text = Double.parseDouble(b1_speed.getText()) + 2;
				text = text <= MAX_ARM_SPEED? text : MAX_ARM_SPEED;
				b1_speed.setText(text + "");
				b1speed = text;
			}
		});
		b1Plus.setLayoutData(new GridData(20, 19));
		b1Plus.setText("+");

		final Button b2Minus = new Button(armSpeedGroup, SWT.NONE);
		b2Minus.addSelectionListener(new SelectionAdapter() {
			public void widgetSelected(final SelectionEvent e) {
				double text = Double.parseDouble(b2_speed.getText()) - 2;
				text = text >= 0.0? text : 0.0;
				b2_speed.setText(text + "");
				b2speed = text;
//				Arm.updateSpeed(text);
			}
		});
		b2Minus.setLayoutData(new GridData(19, 18));
		b2Minus.setText("-");

		b2_speed = new Text(armSpeedGroup, SWT.BORDER);
		b2_speed.setText("0.0");
		final GridData gd_b2_speed = new GridData(SWT.FILL, SWT.CENTER, false, false);
		gd_b2_speed.widthHint = 26;
		b2_speed.setLayoutData(gd_b2_speed);

		final Button b2Plus = new Button(armSpeedGroup, SWT.NONE);
		b2Plus.addSelectionListener(new SelectionAdapter() {
			public void widgetSelected(final SelectionEvent e) {
				double text = Double.parseDouble(b2_speed.getText()) + 2;
				text = text <= MAX_ARM_SPEED? text : MAX_ARM_SPEED;
				b2_speed.setText(text + "");
				b2speed = text;
//				Arm.updateSpeed(text);
			}
		});
		b2Plus.setLayoutData(new GridData(20, 19));
		b2Plus.setText("+");

		final Button b3Minus = new Button(armSpeedGroup, SWT.NONE);
		b3Minus.addSelectionListener(new SelectionAdapter() {
			public void widgetSelected(final SelectionEvent e) {
				double text = Double.parseDouble(b3_speed.getText()) - 2;
				text = text >= MIN_ARM_SPEED? text : MIN_ARM_SPEED;
				b3_speed.setText(text + "");
				b3speed = text;
//				Arm.updateSpeed(text);
			}
		});
		b3Minus.setLayoutData(new GridData(19, 18));
		b3Minus.setText("-");

		b3_speed = new Text(armSpeedGroup, SWT.BORDER);
		b3_speed.setText("0.0");
		final GridData gd_b3_speed = new GridData(SWT.FILL, SWT.CENTER, false, false);
		gd_b3_speed.widthHint = 26;
		b3_speed.setLayoutData(gd_b3_speed);

		final Button b3Plus = new Button(armSpeedGroup, SWT.NONE);
		b3Plus.addSelectionListener(new SelectionAdapter() {
			public void widgetSelected(final SelectionEvent e) {
				double text = Double.parseDouble(b3_speed.getText()) + 2;
				text = text <= MAX_ARM_SPEED? text : MAX_ARM_SPEED;
				b3_speed.setText(text + "");
				b3speed = text;
//				Arm.updateSpeed(text);
			}
		});
		b3Plus.setLayoutData(new GridData(20, 19));
		b3Plus.setText("+");
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);

		final Label b8Label = new Label(armSpeedGroup, SWT.NONE);
		b8Label.setLayoutData(new GridData(SWT.LEFT, SWT.BOTTOM, false, false));
		b8Label.setText("byte[8]");

		final Button b8Minus = new Button(armSpeedGroup, SWT.NONE);
		b8Minus.addSelectionListener(new SelectionAdapter() {
			public void widgetSelected(final SelectionEvent e) {
				double text = Double.parseDouble(b8_speed.getText()) - 2;
				text = text >= MIN_ARM_SPEED? text : MIN_ARM_SPEED;
				b8_speed.setText(text + "");
				b8speed = text;
//				Arm.updateSpeed(text);
			}
		});
		b8Minus.setLayoutData(new GridData(20, 19));
		b8Minus.setText("-");

		b8_speed = new Text(armSpeedGroup, SWT.BORDER);
		b8_speed.setText("0.0");
		final GridData gd_b8_speed = new GridData(SWT.FILL, SWT.CENTER, false, false);
		gd_b8_speed.widthHint = 26;
		b8_speed.setLayoutData(gd_b8_speed);

		final Button b8Plus = new Button(armSpeedGroup, SWT.NONE);
		b8Plus.addSelectionListener(new SelectionAdapter() {
			public void widgetSelected(final SelectionEvent e) {
				double text = Double.parseDouble(b8_speed.getText()) + 2;
				text = text <= MAX_ARM_SPEED? text : MAX_ARM_SPEED;
				b8_speed.setText(text + "");
				b8speed = text;
//				Arm.updateSpeed(text);
			}
		});
		b8Plus.setLayoutData(new GridData(19, 18));
		b8Plus.setText("+");
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);
		new Label(armSpeedGroup, SWT.NONE);
		final Controls control = new Controls();

		if (color) {
			comp.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_DARK_GREEN));
			roverControlGroup.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_DARK_GREEN));
			roverRadio.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_DARK_GREEN));
			armRadio.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_DARK_GREEN));
		}
		
				ip_text = new Text(roverControlGroup, SWT.BORDER);
				ip_text.setLayoutData(new GridData(SWT.FILL, SWT.CENTER, false, false));
		
				assign_ipButton = new Button(roverControlGroup, SWT.NONE);
				assign_ipButton.setText("Assign_IP");
		
				controlMapButton = new Button(roverControlGroup, SWT.NONE);
				controlMapButton.setLayoutData(new GridData(SWT.LEFT, SWT.TOP, false, false));
				controlMapButton.setText("Button Mapping");
	}

	private void createWLanSection(Composite comp) {
		
		Composite wlan = new Composite(comp, SWT.NONE);
		wlan.setLayoutData(new GridData(421, 152));
		wlan.setLayout(new GridLayout(2, false));

		// =========================================
		wireless1 = new wirelessWidget(wlan, SWT.NONE, "192.168.0.1");
		wireless1.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));

		wireless2 = new wirelessWidget(wlan, SWT.NONE, "192.168.1.1");
		wireless2.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));

	}

	private void createExceptionSection(Composite comp) {

		Label exceptionsLabel = new Label(comp, SWT.NONE);
		exceptionsLabel.setText("Messages / Exceptions:");
		exceptionsLabel.setLayoutData(new GridData(SWT.FILL, SWT.FILL, false, false));

		exceptionViewer = new StyledText(comp, SWT.FULL_SELECTION | SWT.V_SCROLL | SWT.H_SCROLL | SWT.BORDER);
		exceptionViewer.setText("Exceptions Thrown here ...");
		exceptionViewer.setEditable(false);
		GridData g = new GridData(SWT.FILL, SWT.FILL, false, false, 1, 1);
		g.heightHint = 150;
		g.widthHint = 400;

		exceptionViewer.setLayoutData(g);

		if (color) {
			exceptionViewer.setForeground(Display.getCurrent().getSystemColor(SWT.COLOR_RED));
			exceptionsLabel.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_TITLE_BACKGROUND));
		}
	}

	private void createProcessTab(TabFolder tabFolder) {
		final TabItem systemTabItem = new TabItem(tabFolder, SWT.NONE);
		systemTabItem.setText("ProcessControl");
		processController = new processControlTabs(tabFolder, SWT.BORDER);
		processController.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, true, true));
		systemTabItem.setControl(processController);
	}

	private void createStereoTab(TabFolder tabFolder) {
		final TabItem systemTabItem = new TabItem(tabFolder, SWT.NONE);
		systemTabItem.setText("StereoView");

		Composite c = new Composite(tabFolder, SWT.NONE);
		c.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, true, true));
		c.setLayout(new GridLayout());

		Label l = new Label(c, SWT.NONE);
		l.setText("For Debug only.\n Simply shows the Stereo cameras and\n    allows changing of settings");
		l.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false));
		Button b = new Button(c, SWT.PUSH);
		b.setText("Show StereoView");
		b.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false));

		b.addSelectionListener(new SelectionListener() {
			public void widgetDefaultSelected(SelectionEvent arg0) {
			}

			@Override
			public void widgetSelected(SelectionEvent arg0) {
				new stereoControl().open();

			}
		});

		systemTabItem.setControl(c);
	}

//	private void createPowerTab(TabFolder tabFolder) {
//
//		final TabItem powerTabItem = new TabItem(tabFolder, SWT.NONE);
//		powerTabItem.setText("Power");
//
//		final Composite composite = new Composite(tabFolder, SWT.NONE);
//		composite.setLayout(new GridLayout(3, false));
//		composite.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, true, true));
//		powerTabItem.setControl(composite);
//
//		Button powerButton = new Button(composite, SWT.TOGGLE);
//		powerButton.setText("Power Button");
//		powerButton.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false));
//		powerButton.addListener(SWT.Selection, new Listener() {
//			public void handleEvent(Event event) {
//				try {
//					count++;
//					if ((count % 2) == 0) {
//						AllON();
//						// throwException("Power ON");
//					} else {
//						ALLOFF();
//						// throwException("Power OFF");
//					}
//
//				} catch (Exception e) {
//					throwException(e);
//					// throwException(e.getStackTrace());
//				}
//			}
//		});
//
//		new Label(composite, SWT.NONE).setLayoutData(new GridData());
//		new Label(composite, SWT.NONE).setLayoutData(new GridData());
//
//		final Label pcStatus1 = new Label(composite, SWT.BORDER);
//		pcStatus1.setText("PC 1 Status - Not Working");
//		GridData g = new GridData(SWT.LEFT, SWT.CENTER, false, false);
//		g.widthHint = 300;
//		pcStatus1.setLayoutData(g);
//
//		pcStatus1.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_YELLOW));
//		pcStatus1.setForeground(Display.getCurrent().getSystemColor(SWT.COLOR_RED));
//
//		final Button pc1ON = new Button(composite, SWT.NONE);
//		pc1ON.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false));
//		pc1ON.setText("ON");
//		pc1ON.addListener(SWT.Selection, new Listener() {
//			public void handleEvent(Event event) {
//				try {
//					boolean stat = power(PC1_ID, ON);
//					pcStatus1.setText((stat == true) ? "PC 1 Status - Working" : "PC 1 Status - Not Working");
//					pcStatus1.setForeground((stat == true) ? Display.getCurrent().getSystemColor(SWT.COLOR_GREEN)
//							: Display.getCurrent().getSystemColor(SWT.COLOR_RED));
//
//				} catch (Exception e) {
//					throwException(e);
//				}
//
//			}
//
//		});
//
//		final Button pc1OFF = new Button(composite, SWT.NONE);
//		pc1OFF.setText("OFF");
//		pc1OFF.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false));
//		pc1OFF.addListener(SWT.Selection, new Listener() {
//			public void handleEvent(Event event) {
//				try {
//					boolean stat = power(PC1_ID, OFF);
//					pcStatus1.setText((stat == true) ? "PC 1 Status - Not Working" : "PC 1 Status - Working");
//					pcStatus1.setForeground((stat == true) ? Display.getCurrent().getSystemColor(SWT.COLOR_RED)
//							: Display.getCurrent().getSystemColor(SWT.COLOR_GREEN));
//				} catch (Exception e) {
//					throwException(e);
//					// throwException(e.getStackTrace());
//				}
//
//			}
//
//		});
//
//		final Label pcStatus2 = new Label(composite, SWT.BORDER);
//		pcStatus2.setText("PC 2 Status - Not Working");
//		g = new GridData(SWT.LEFT, SWT.CENTER, false, false);
//		g.widthHint = 300;
//		pcStatus2.setLayoutData(g);
//		pcStatus2.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_YELLOW));
//		pcStatus2.setForeground(Display.getCurrent().getSystemColor(SWT.COLOR_RED));
//
//		Button pc2ON = new Button(composite, SWT.NONE);
//		pc2ON.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false));
//		pc2ON.setText("ON");
//		pc2ON.addListener(SWT.Selection, new Listener() {
//			public void handleEvent(Event event) {
//				try {
//					boolean stat = power(PC2_ID, ON);
//					pcStatus2.setText((stat == true) ? "PC 2 Status - Working" : "PC 2 Status - Not Working");
//					pcStatus2.setForeground((stat == true) ? Display.getCurrent().getSystemColor(SWT.COLOR_GREEN)
//							: Display.getCurrent().getSystemColor(SWT.COLOR_RED));
//				} catch (Exception e) {
//					throwException(e);
//
//				}
//			}
//		});
//
//		final Button pc2OFF = new Button(composite, SWT.NONE);
//		pc2OFF.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false));
//		pc2OFF.setText("OFF");
//		pc2OFF.addListener(SWT.Selection, new Listener() {
//			public void handleEvent(Event event) {
//				try {
//					boolean stat = power(PC2_ID, OFF);
//					pcStatus2.setText((stat == true) ? "PC 2 Status - Not Working" : "PC 2 Status - Working");
//					pcStatus1.setForeground((stat == true) ? Display.getCurrent().getSystemColor(SWT.COLOR_RED)
//							: Display.getCurrent().getSystemColor(SWT.COLOR_GREEN));
//				} catch (Exception e) {
//					throwException(e);
//					// throwException(e.getStackTrace());
//				}
//			}
//		});
//
//		if (color) {
//			composite.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_DARK_GREEN));
//
//		}
//	}

	private void createBottomBar(Composite comp) {

		Composite bar = new Composite(comp, SWT.NONE);
		bar.setLayoutData(new GridData(SWT.FILL, SWT.FILL, false, true, 2, 1));
		bar.setLayout(new GridLayout(5, false));

		Button exitButton = new Button(bar, SWT.NONE);
		exitButton.setText("EXIT");
		exitButton.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, true));
		exitButton.addListener(SWT.Selection, new Listener() {
			public void handleEvent(Event event) {
				try {
					shutDown();
				} catch (Exception e) {
					throwException(e);
				}
			}
		});

		Label roverStatus = new Label(bar, SWT.BORDER);
		roverStatus.setText("Rover OFF");
		GridData g = new GridData(SWT.LEFT, SWT.LEFT, false, true);
		g.widthHint = 100;
		roverStatus.setLayoutData(g);

		Label pc1Status = new Label(bar, SWT.BORDER);
		pc1Status.setText("PC1 OFF");
		g = new GridData(SWT.LEFT, SWT.LEFT, false, true);
		g.widthHint = 100;
		pc1Status.setLayoutData(g);

		Label pc2Status = new Label(bar, SWT.BORDER);
		pc2Status.setText("PC2 OFF");
		g = new GridData(SWT.LEFT, SWT.LEFT, false, true);
		g.widthHint = 100;
		pc2Status.setLayoutData(g);

		controlStat = new Label(bar, SWT.BORDER);
		controlStat.setText("Current Control ->");
		g = new GridData(SWT.FILL, SWT.LEFT, true, true);
		controlStat.setLayoutData(g);

		if (color) {
			roverStatus.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_TITLE_BACKGROUND));
			roverStatus.setForeground(Display.getCurrent().getSystemColor(SWT.COLOR_RED));
			pc1Status.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_TITLE_BACKGROUND));
			pc1Status.setForeground(Display.getCurrent().getSystemColor(SWT.COLOR_RED));
			pc2Status.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_TITLE_BACKGROUND));
			pc2Status.setForeground(Display.getCurrent().getSystemColor(SWT.COLOR_RED));
			controlStat.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_TITLE_BACKGROUND));
			controlStat.setForeground(Display.getCurrent().getSystemColor(SWT.COLOR_RED));
		}
	}

	private void createCameraComposite(Composite container) {

		cameraWidget = new cameraWidget(container, SWT.NONE);
		cameraWidget.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, true, true, 1, 7));

		if (color)
			cameraWidget.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_TITLE_BACKGROUND));

		try {
			Class partypes[] = new Class[1];
			partypes[0] = String.class;
			Method meth = getClass().getMethod("throwException", partypes);
			cameraWidget.setErrorOut(meth, this);

		} catch (SecurityException e2) {
			e2.printStackTrace();
		} catch (NoSuchMethodException e2) {
			e2.printStackTrace();
		}
	}

	private Shell createShell() {

		Shell sShell = new Shell(SWT.CLOSE | SWT.MIN | SWT.MAX | SWT.RESIZE | SWT.APPLICATION_MODAL);
		sShell.setSize(557, 469);
		sShell.setText("Plot Window");

		GridData gd = new GridData(SWT.FILL, SWT.FILL, true, true);
		gd.horizontalAlignment = SWT.CENTER;
		gd.verticalAlignment = SWT.CENTER;
		sShell.setLayoutData(gd);

		sShell.setLayout(new GridLayout(1, false));
		Composite cam = createContents(sShell);

		// Set layout info for the root widget
		gd = new GridData(SWT.FILL, SWT.FILL, true, true);
		gd.horizontalAlignment = SWT.CENTER;
		gd.verticalAlignment = SWT.CENTER;
		cam.setLayoutData(gd);

		sShell.layout();

		return sShell;
	}



	public void actionPerformed(ActionEvent ae) 
	{		
		if(ae.getSource() == this.controlMapButton)
		{
			Control c = new Control();
		}
	}

	
	public void setText(String text) {
		exceptionViewer.setText(text);
	}

	public String getIp_text() {
		return exceptionViewer.getText();
	}

	public void throwException(String exception) {
		exceptionViewer.append("\n" + exception);
		exceptionViewer.setSelection(exceptionViewer.getCharCount() - 1);

	}

	public void throwException(Exception e) {
		String v = "";

		for (int i = 0; i < e.getStackTrace().length; i++) {
			v += e.getStackTrace()[i].toString() + "\n";
		}
		// exceptionViewer.setText("\n\n" + e.toString() +
		// "\nStackTrace:***********\n" + v);
		exceptionViewer.append("\n\n" + e.toString() + "\nStackTrace:***********\n" + v);
		exceptionViewer.setSelection(exceptionViewer.getCharCount() - 1);
	}

	public void throwException(StackTraceElement[] ste) {
		String v = "";

		for (int i = 0; i < ste.length; i++) {
			v += ste[i].toString() + "\n";
		}
		exceptionViewer.append("\n" + v);
		exceptionViewer.setSelection(exceptionViewer.getCharCount() - 1);

	}

	private void shutDown() {
		// Kill off the camera and processcontrollers.
//		this.Arm = null;
//		this.Accelerate = null;
		cameraWidget.setTab("0");
		processController.dispose();
		shell.dispose();
		System.exit(0);

	}

	public boolean power(int pc_ID, int status) throws Exception {
		boolean response;
		TCPClient power = new TCPClient(ROVERIP, POWERPORT);
		power.write(pc_ID, status);
		byte respond = power.read();

		response = ((int) respond == 1) ? true : false;

		return response;
	}

	private void initialize() 
	{
//		try {
////			Arm = new ArmThread(ROVERIP, ARM_PORT, this);
//			Accelerate = new RoverMoveThread(ROVERIP, MOVE_PORT, this);
//		} catch (Exception e1) {
//			throwException(e1);
//		}
	}

	public Display getDisplay() {
		if( shell != null)
			return shell.getDisplay();
		else 
			return null;
	}

	/**
	 * Launch the application
	 * 
	 * @param args
	 */
	public static void main(String args[]) {

		Display display = Display.getDefault();
		View w = new View();
		if (System.getProperty("os.name").equals("Linux"))
			w.color = false;
	
		
		shell = w.createShell();
		shell.setText("YURT Control Station");
		shell.pack();
		shell.open();

		while (!shell.isDisposed()) {
			if (!display.readAndDispatch())
				display.sleep();

		}
		System.out.println("Dying");

		w.processController.dispose();
		w.cameraWidget.dispose();
		w.wireless1.dispose();
		w.wireless2.dispose();
		shell.dispose();

	}

}
