package compassTilt;

import javax.swing.BoxLayout;
import javax.swing.JPanel;

public class CompassTiltPanel extends JPanel {
	private CompassClient compass;
	public TiltPanel RTPanel;
	public CompassPanel CP;
	
	private Thread updateThread;
	private String addr;
	private int port;
	
	public CompassTiltPanel(String addr, int port, boolean bTest) {
		RTPanel = new TiltPanel(TiltPanel.DEF_WIDTH, TiltPanel.DEF_HEIGHT, bTest);
		CP = new CompassPanel(15, 40, bTest);
		
		this.setLayout( new BoxLayout(this, BoxLayout.X_AXIS));
		this.add(RTPanel);
		this.add(CP);
		this.addr = addr;
		this.port = port;
		if( !bTest ) {
			updateThread = new UpdateThread();
			updateThread.start();
		}
	}
	
	public void update() { 
		
		System.out.println("Calling update");
		
		if (!compass.update()){
			return;
		}
		CP.sHeading = compass.heading + "";
		
		int headingMod = ((compass.heading + 22) / 45) % 8;
		//System.out.println(headingMod);
		if (headingMod == 0) CP.sDegree = "N";
		else if (headingMod == 1) CP.sDegree = "NE";
		else if (headingMod == 2) CP.sDegree = "E";
		else if (headingMod == 3) CP.sDegree = "SE";
		else if (headingMod == 4) CP.sDegree = "S";
		else if (headingMod == 5) CP.sDegree = "SW";
		else if (headingMod == 6) CP.sDegree = "W";
		else if (headingMod == 7) CP.sDegree = "NW";
		CP.headingTArea.setText(CP.sHeading);
		CP.degreeTArea.setText(CP.sDegree);
		CP.changeHeading(headingMod);
		
		RTPanel.setHTilt(compass.tiltx);
		RTPanel.setVTilt(compass.tilty);
	}
	
	public class UpdateThread extends Thread {
		 public void run() {
			 compass = new CompassClient(addr, port);
			 while(true) {
				 update();
				 try {
						Thread.sleep(100);
					} catch (InterruptedException e) { e.printStackTrace(); }
			 }
			
		 }
	}//updateThread
}//CompassTiltPanel
