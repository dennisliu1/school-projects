package compassTilt;

import javax.swing.BoxLayout;
import javax.swing.JPanel;

public class CopyOfCompassTiltPanel extends JPanel {
	private CompassClient compass;
	public TiltPanel RTPanel;
	public CompassPanel CP;
	
	public CopyOfCompassTiltPanel(String addr, int port, boolean bTest) {
		RTPanel = new TiltPanel(TiltPanel.DEF_WIDTH, TiltPanel.DEF_HEIGHT, bTest);
		CP = new CompassPanel(15, 40, bTest);
		
		this.setLayout( new BoxLayout(this, BoxLayout.X_AXIS));
		this.add(RTPanel);
		this.add(CP);
		
		if( !bTest ) {
			compass = new CompassClient(addr, port);
			Thread compassUpdate = new Thread() {
				public void run() {
					update();
					try {
						Thread.sleep(100);
					} catch (InterruptedException e) { e.printStackTrace(); }
				}//run method
			};//thread construct
			compassUpdate.start();
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
		
		RTPanel.setHTilt(compass.tiltx);
		RTPanel.setVTilt(compass.tilty);
	}
}
