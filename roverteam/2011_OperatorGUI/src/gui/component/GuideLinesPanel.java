package gui.component;



import java.awt.Color;
import java.awt.Graphics;

public class GuideLinesPanel extends GUIComponent {
	public GuideLinesPanel(int x, int y, int width, int height, float transparency) {
		super(x, y, width, height, transparency);
	}
	
	public void paintBuffer(Graphics g) {
		super.paintComponent(g);
		//X:530 Y:463
//	X:685 Y:568
//	X:307 Y:474
//	X:3 Y:588
		g.setColor(Color.red);
		g.drawLine(530,463, 685,568);
		g.drawLine(307,474, 3,588);
	}
}//GuideLinesPanel class
