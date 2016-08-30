package york.rover.camera;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Canvas;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Shell;

import york.rover.camera.cameraWidget;

public class multiView {
	static Display display;
	static Canvas mapCanvas;
	static cameraWidget[] cam;
	static cameraWidget cam1, cam2, cam3;
	public static void main(String[] args){
		display = Display.getDefault();
		
		Shell shell = createShell();
		
		shell.pack();
		shell.open();
		while (!shell.isDisposed()) {
			if (!display.readAndDispatch())
				display.sleep();

		}

		for(int i = 0; i<6; i++)
			cam[i].dispose();

		
		shell.dispose();
		
	}
		
	private static Shell createShell() {

		Shell sShell = new Shell(SWT.CLOSE | SWT.MIN | SWT.MODELESS);
		sShell.setText("Plot Window");
		
		GridData gd = new GridData(SWT.FILL, SWT.FILL, true, true);
		gd.horizontalAlignment = SWT.CENTER;
		gd.verticalAlignment = SWT.CENTER;
		sShell.setLayoutData(gd);
		
		sShell.setLayout(new GridLayout(3, false));
	
		cam = new cameraWidget[6];

		for(int i = 0; i < 6; i++){
			cam[i] = new cameraWidget(sShell, SWT.NONE,30100 + i, 1);
			cam[i].setLayoutData(new GridData(SWT.CENTER, SWT.CENTER, true, true));
			cam[i].forcedSize = true;
		
		}
	
		sShell.layout();
		
		return sShell;
	}
}
