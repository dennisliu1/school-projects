package GUI;

// Created by Dachuan Huang
// Rover Lag-meter display
// Version 3.0

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.io.IOException;
import java.util.Scanner;
import javax.swing.JFrame;

public class LagMeter extends GUIComponent {
	  private int[] latency; 	// latency array
	  private int bound1;		// custom defined bound for signal strength
	  private int bound2;		// custom defined bound for signal strength
	  private int bound3;		// custom defined bound for signal strength
	  private int bound4;		// custom defined bound for signal strength
	  public LagMeterUpdateFunction muf;		// a sub class to use Updatefunction
	  public static final int _width = 80;	// width of the GUI
	  public static final int _height = 60;	// height of the GUI
		public static final Color DEFAULT_BAR_COLOR = Color.red;
		public static final Color DEFAULT_FONT_COLOR = Color.blue;
		public static final Color DEFAULT_BOX_COLOR = Color.black;
		private LagMeter self = this;
		private Graphics2D g2;
//---------------------------             CONSTRUCTORS             ---------------------------//
	  /** Constructor for class invoking */
	  public LagMeter(int x, int y, int width, int height, float transparency, int[] latency) {
		  super(x,y,width,height,transparency);	// pass to GUIComponent to construct its parent class
		  this.latency = latency;
		  initVars();
		}
		public LagMeter(int x, int y, float transparency, int[] latency) {
			this(x,y, _width, _height, transparency, latency);
		}
		public LagMeter(int x, int y, int width, int height,float transparency) {
			super(x,y, width,height, transparency);
			this.latency = new int[1];
			initVars();
		}
		public LagMeter(int x, int y, float transparency) {
			this(x,y, _width, _height, transparency);
		}
		private void initVars() {
		  muf = new LagMeterUpdateFunction(latency);
		}
//---------------------------               FUNCTIONS               ---------------------------//
		public void setTelemetry(int[] latency) {
			this.latency = latency;
		}
	  /** Allow bounds to be set and adjusted */
	  public void setLMBounds(int bound1, int bound2, int bound3, int bound4) {
		  this.bound1 = bound1;
		  this.bound2 = bound2;
		  this.bound3 = bound3;
		  this.bound4 = bound4;
	  }
	  
	  /** Allow constant update for latency rate */
	  public void updateDisplay() {
		  this.repaint();
	  }
	  	  
	  /** Custom paint method to draw lag meters */
	  public void paintBuffer(Graphics g) {
			g2 = (Graphics2D)g;
		  //super.paint(g);
			super.paintComponent(g);
		  g2.setColor(DEFAULT_BAR_COLOR);
		  //x = 0; y = 0;	// setting x and y to 0 to reposition
		    
		  /** Draw the default boundaries of bounds */
		  //g.drawRect(0, 40, 15, 10);
		  //g.drawRect(20, 30, 15, 20);
		  //g.drawRect(40, 20, 15, 30);
		  //g.drawRect(60, 10, 15, 40);
		  
		  /** Draw bars according to the given latency */
		  g2.fillRect(5, 40, 15, 10);
		  if (this.latency[0] <= bound4) {
				g2.fillRect(21, 30, 15, 20);
		  } else g2.drawRect(21, 30, 15, 20);
			
		  if (this.latency[0] <= bound3) {
				g2.fillRect(37, 20, 15, 30);
		  }else g2.drawRect(37, 20, 15, 30);
			
		  if (this.latency[0] <= bound2) {
				g2.fillRect(53, 10, 15, 40);
		  } else g2.drawRect(53, 10, 15, 40);
			
		  g2.setColor(DEFAULT_FONT_COLOR);
		  g2.drawString(Integer.toString(this.latency[0]), 12,17);
			
			g2.setColor(DEFAULT_BOX_COLOR);
			g2.drawRect(0, 0, width-1, height-1);
		  //this.repaint();
	  }
	  
	  /** MyUpdateFunction implements two abstract class from parent */
	  class LagMeterUpdateFunction extends UpdateFunction {
		  private int[] old, latency;

		  /** default constructor */
		  public LagMeterUpdateFunction(int[] latency) {
		  	this.latency = latency;
		  	this.old = new int[1];
		  	this.old[0] = latency[0];
		  }
		  
		  /** Note: parent has function: update() that uses the two methods below */
		  
		  /** Check to see if there's a new value update */
		  public boolean checkValues() {
			  if (old[0] != latency[0]) {
				  return true;
			  }
			  return false;
		  }
		  
		  /** Update the current value to the new value */
		  public void doUpdate() {
		  	this.old[0] = latency[0];
				self.updateDisplay();
		  }	   	  
	  }
		
		
		
		/** The main method */
	  public static void main(String[] a) throws IOException {
	    
	    JFrame window = new JFrame();
	    window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	    //window.setBounds(30, 30, 200, 200);
			
	    int[] latency = {350};	// Latency default value
	    
	    int input_int = 0;	// used for testing value
	    
	    /** ============ On-call functions ===============
	    /** This object can be created by passing x,y coordinates and an array
	    /** of latency */
	    /** create LagMeter with given parameters */
	    LagMeter lm = new LagMeter(0,0, _width, _height, 1f, latency);
	    
	    /** ============ On-call functions ===============
	    /** To change the default bounds, use this function to set new bounds
	    /** set bounds for LagMeter, default bounds: 0-100 101-350 351-500 500+ */
	    lm.setLMBounds(0, 100, 350, 500);
	    
	    window.getContentPane().add(lm);
			window.pack();
	    window.setVisible(true);
	 	
    	    /** ======================== Testing =============================
    	    /** To change latency display, simply type a value (Numeric) on 
    	    /** the command line, the LagMeter will auto-updates its status */
	    while (true){
	    	Scanner scanner = new Scanner(System.in);
	    	try{
	    		input_int = scanner.nextInt();	// get new value from input stream
	    		latency[0] = input_int;			// assign value to latency
	    	}
	    	catch(Exception e){			// catch non numeric exceptions
	    		System.out.println("Enter numeric values only!");
	    	}
	    	
	    	/* call update function to check for new values */
	    	lm.muf.update();			
	    }  
	  }
}//LagMeter class
