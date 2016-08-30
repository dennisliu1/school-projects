package driveArmControl;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;

import javax.swing.JPanel;

public class LightPanel extends JPanel {
	public int width, height;
	public Color onColor, offColor, midColor, currColor;
	private int currState;
	
	public LightPanel(int width, int height, Color onColor, Color midColor, Color offColor) {
		this.width = width;
		this.height = height;
		this.onColor = onColor;
		this.midColor = midColor;
		this.offColor = offColor;
		this.currColor = offColor;
		this.setSize(new Dimension(width, height));
	}
	public LightPanel(int width, int height) {
		this(width, height, Color.red, Color.green, Color.yellow);
	}
	public  boolean setColor(int num) {
		if(num == 0) currColor = offColor;
		else if(num == 1) currColor = midColor;
		else if(num == 2) currColor = onColor;
		else return false;
		repaint();
		return true;
	}
	
	public void update(Graphics g) {
		g.setColor(currColor);
		for(int i = 0; i < width; i++ ) {
			g.drawLine(0, i, height, i);
		}
	}
	@Override
	public void paint(Graphics g) {
		update(g);
	}
}//LightPanel class
