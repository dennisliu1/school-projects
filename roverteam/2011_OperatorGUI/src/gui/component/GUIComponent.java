package gui.component;

import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.image.VolatileImage;
import java.util.ArrayList;
import javax.swing.JPanel;

/**
 * Base Component for the GUI. All classes to be used by the GUI must extend
 * this class.
 * 
 * @version 1.0
 * @author Dennis Liu, YURT 2012
 */
public class GUIComponent extends JPanel {
	//-------------------------------------		Variables			-------------------------------------//
	private float transparency;
	public int x,y,width, height;
	
	private int bufferWidth;
	private int bufferHeight;
	private Image bufferImage;
	private Graphics bufferGraphics;
	//-----------------------w--------------		CONSTRUCTOR		-------------------------------------//
	/**
	 * Constructs a JPanel with an update thread running in the background. Add
	 * functions to it by using addUpdateFunction(). Note that thread needs to be
	 * started manually. It does not start on construction. 
	 * 
	 * *several things to note: 
	 * - Layout is defaulted to null.
	 * - preferred size is set to width/height.
	 * - bounds is set to parameters.
	 * - opacity is turned on.
	 * 
	 * @param x x position of the top left of the panel. 
	 * @param y y position of the top left of the panel. 
	 * @param width width of the panel.
	 * @param height height of the panel.
	 */
	public GUIComponent(int x, int y, int width, int height, float transparency) {
		super();//construct panel
		this.x = x;
		this.y = y;
		this.width = width;
		this.height = height;
		this.transparency = transparency;
		this.setLayout(null);//ensure no layout 
		this.setPreferredSize(new Dimension(width, height));
		this.setBounds(x,y, width, height);
		this.setOpaque(false);
	}
	
//-----------------------w--------------		PAINT FUNCTIONS		-------------------------------------//
	public void update(Graphics g) {
		paint(g);
	}
  public void paint( Graphics g ) {
		//    checks the buffersize with the current panelsize
		//    or initialises the image with the first paint
		if(bufferWidth!=getSize().width || 
			bufferHeight!=getSize().height || 
			bufferImage==null || bufferGraphics==null)
				resetBuffer();
		if(bufferGraphics!=null) {
			//this clears the offscreen image, not the onscreen one
			bufferGraphics.clearRect(0,0,bufferWidth,bufferHeight);

			//calls the paintbuffer method with the offscreen graphics as a param
			paintBuffer(bufferGraphics);
			
			//we finally paint the offscreen image onto the onscreen image
			Graphics2D g2 = (Graphics2D) g;
			g2.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, transparency));
			super.paint(g);
			g2.drawImage(bufferImage,0,0,this);
			g2.dispose();
		}
  }//paintComponent()
	public void paintBuffer(Graphics g) {
			//in classes extended from this one, add something to paint here!
			//always remember, g is the offscreen graphics
	}
	private void resetBuffer() {
		// always keep track of the image size
		bufferWidth=getSize().width;
		bufferHeight=getSize().height;

		//    clean up the previous image
		if(bufferGraphics!=null) {
				bufferGraphics.dispose();
				bufferGraphics=null;
		}
		if(bufferImage!=null) {
				bufferImage.flush();
				bufferImage=null;
		}
		System.gc();

		//    create the new image with the size of the panel
		bufferImage=createImage(bufferWidth,bufferHeight);
		bufferGraphics=bufferImage.getGraphics();
	}
	public void updateDisplay() {
		repaint();
	}
	//---------------------------------		    Accessors   		-------------------------------------//
	public int getX() {
		return x;
	}
	public void setX(int x) {
		this.x = x;
		this.setBounds(x,y, width, height);
	}
	public int getY() {
		return y;
	}
	public void setY(int y) {
		this.y = y;
		this.setBounds(x,y, width, height);
	}
	public int getWidth() {
		return width;
	}
	public void setWidth(int width) {
		this.width = width;
		this.setBounds(x,y, width, height);
	}
	public int getHeight() {
		return height;
	}
	public void setHeight(int height) {
		this.height = height;
		this.setBounds(x,y, width, height);
	}
	public float getTransparency() {
		return transparency;
	}
	public void setTransparency(float transparency) {
		this.transparency = transparency;
	}
	//---------------------------------		Thread Functions		-------------------------------------//
	//------------------------------- 		Thread Functions		-------------------------------------//
	//-------------------------------------		Thread		-------------------------------------//
}//class
