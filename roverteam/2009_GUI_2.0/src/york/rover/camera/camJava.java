package york.rover.camera;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Canvas;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Shell;

import york.rover.camera.cameraWidget;

public class camJava {
	static Display display;
	static Canvas mapCanvas;
	static cameraWidget cam;
	public static void main(String[] args){
		display = Display.getDefault();
		
		Shell shell = createShell();
		
		shell.pack();
		shell.open();
		while (!shell.isDisposed()) {
			if (!display.readAndDispatch())
				display.sleep();

		}

		cam.dispose();
		
		shell.dispose();
		
	}
		
	private static Shell createShell() {

		Shell sShell = new Shell(SWT.CLOSE | SWT.MIN | SWT.MODELESS);
		sShell.setText("Plot Window");
		
		GridData gd = new GridData(SWT.FILL, SWT.FILL, true, true);
		gd.horizontalAlignment = SWT.CENTER;
		gd.verticalAlignment = SWT.CENTER;
		sShell.setLayoutData(gd);
		
		sShell.setLayout(new GridLayout(1, false));
		cam = new cameraWidget(sShell, SWT.NONE,30100, 1);
		
		// Set layout info for the root widget
		gd = new GridData(SWT.FILL, SWT.FILL, true, true);
		gd.horizontalAlignment = SWT.CENTER;
		gd.verticalAlignment = SWT.CENTER;
		cam.setLayoutData(gd);
		
		
		sShell.layout();
		
		return sShell;
	}
}
