package controls;

import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Shell;

import york.rover.camera.StereoWidget;

public class stereoControl {

	protected Shell shell;

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
		shell.setText("Stereo Control");

		Composite composite = new Composite(shell, SWT.NONE);
		composite.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		composite.setLayout(new GridLayout(1, true));

		StereoWidget label = new StereoWidget(composite, SWT.NONE);
		label.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));

		Button exitButton = new Button(composite, SWT.NONE);
		exitButton.setText("EXIT");
		exitButton.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false));
		exitButton.addListener(SWT.Selection, new Listener()
		{
			public void handleEvent(Event e)
			{
				shell.close();
			}
		});
		
		shell.pack();
		shell.layout();
	}

}

