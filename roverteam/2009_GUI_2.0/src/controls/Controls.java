package controls;

import org.eclipse.swt.SWT;

import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Shell;

public class Controls {

	protected Shell shell;

	/**
	 * Launch the application
	 * @param args
	 */
	/*public static void main(String[] args) {
		try {
			Control window = new Control();
			window.open();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}*/

	/**
	 * Open the window
	 */
	public void open() {
		final Display display = Display.getDefault();
		createContents();
		shell.open();
		shell.layout();
		while (!shell.isDisposed()) {
			if (!display.readAndDispatch())
				display.sleep();
		}
	}

	/**
	 * Create contents of the window
	 */
	protected void createContents() {
		shell = new Shell();
		
		shell.setLayout(new GridLayout(1, true));
		shell.setText("Rover Controls Help");

		final Composite composite = new Composite(shell, SWT.NONE);
		composite.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		composite.setLayout(new GridLayout(1, true));

		final Label label = new Label(composite, SWT.NONE);
		label.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		label.setText(" Rover Movement Control \r\n\n" +
				  " Boost Speed		: Throttle Button up \r\n" +
				  " Reduce Speed		: Throttle Button Down \r\n" +
				  " Forward			: Joystick - Up + 3\r\n" +
				  " Reverse			: Joystick - Down + 2\r\n" +
				  " Left-Foward		: Joystick - Left + 3\r\n" +
				  " Right - Foward		: Joystick - Right + 3\r\n" +
				  " Left-Reverse		: Joystick - Left + 2\r\n" +
				  " Right - Reverse		: Joystick - Right + 2\r\n" +
				  " 360 about any axis	: pull Joystick backwards and bend \n\t\t\t  left or right to direction" +
				  "\r\n"+
				  "   Arm Control \r\n" +
				  " UP - Down :: Button 3 - Button 2 \r\n" +
				  " LEFT - RIGHT :: Button 4 - Button 5 \r\n" +
				  " FORWARD - BACKWARD :: Button 6 - Button 7 \r\n" +
				  " TILT UP - TILT DOWN :: Button 8 - Button 9 \r\n" +
				  " \r\n");
		
		final Button exitButton = new Button(composite, SWT.NONE);
		exitButton.setText("EXIT");
		exitButton.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false));
		exitButton.addListener(SWT.Selection, new Listener()
		{
			public void handleEvent(Event e)
			{
				shell.close();
			}
		});
		if(false){
			shell.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_TITLE_INACTIVE_BACKGROUND_GRADIENT));
			composite.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_TITLE_BACKGROUND));
			label.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_TITLE_BACKGROUND));
		}
		shell.pack();
		shell.layout();
	}

}
