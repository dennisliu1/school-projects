package gui.testing;



import gui.component.GUIComponent;

import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.image.BufferedImage;
import javax.swing.JLabel;

/**
 * @author Daanish
 */
public class WarningSystem extends GUIComponent {
	private GUIComponent warnPanel = this;
	private boolean warningConds[];
	public UpdateWarnFunction updateWarnings;
	private Image img;
	private float transparency;
	
	public WarningSystem(int x1,int y1,int x2,int y2, float transparency, boolean[] warningConds, Image img) {
		//Main Warning Panel
		super(x1, y1, x2, y2, transparency);
		this.warningConds = warningConds;
		
		this.img = img;
		this.updateWarnings = new UpdateWarnFunction(warningConds);
		this.transparency = transparency;
	}
	@Override
	public void paintBuffer(Graphics g) {
		g.drawImage(img, 0,0, null);
	}
	//Update Function
	private class UpdateWarnFunction extends UpdateFunction {
		private boolean[] inputBooleans;
		private boolean oldInput;
		
		public UpdateWarnFunction(boolean[] inputBooleans) {
			this.inputBooleans = inputBooleans;
			oldInput = inputBooleans[0];
		}
		
		@Override
		public boolean checkValues() {
			if(oldInput != inputBooleans[0]) {
				oldInput = inputBooleans[0];
				return true;
			}
			return false;
		}
		@Override
		public void doUpdate() {
			if(warnPanel.getTransparency() == 0) warnPanel.setTransparency(transparency);
			else warnPanel.setTransparency(0f);
		}
	}
}		

