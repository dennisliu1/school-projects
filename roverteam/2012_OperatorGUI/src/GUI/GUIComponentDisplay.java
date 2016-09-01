/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package GUI;

import java.awt.*;
import javax.swing.JPanel;

/**
 * Base Component for the GUI. All classes to be used by the GUI must extend
 * this class.
 * 
 * @version 1.0
 * @author Dennis Liu, YURT 2012
 */
public class GUIComponentDisplay extends JPanel {
	//-------------------------------------		Variables			-------------------------------------//
	private float transparency;
	int width, height;
	
	private int bufferWidth;
	private int bufferHeight;
	private Image bufferImage;
	private Graphics bufferGraphics;
	//-----------------------w--------------		Constructor		-------------------------------------//
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
	public GUIComponentDisplay(int x, int y, int width, int height, float transparency) {
		super();//construct panel
		this.transparency = transparency;
		this.setLayout(null);//ensure no layout 
		this.setPreferredSize(new Dimension(width, height));
		this.setBounds(x,y, width, height);
		this.setOpaque(false);
		
		this.width = width;
		this.height = height;
	}
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

			//calls the paintbuffer method with 
			//the offscreen graphics as a param
			paintBuffer(bufferGraphics);
			

			//we finaly paint the offscreen image onto the onscreen image
			Graphics2D g2 = (Graphics2D) g;
			g2.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, transparency));
			//super.paint(g2);
			g2.drawImage(bufferImage,0,0,this);
			g2.dispose();
			//g.drawImage(bufferImage,0,0,this);
		}
  }//paintComponent()
	public void paintBuffer(Graphics g) {
			//in classes extended from this one, add something to paint here!
			//always remember, g is the offscreen graphics
	}
	private void resetBuffer(){
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
	
}//class
