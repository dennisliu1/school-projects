/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package GUI;

import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;
import java.util.Random;
import javax.swing.JFrame;
import javax.swing.JPanel;

/**
 * 
 * @author dennis
 */
public class RoverPicPanel extends GUIComponent {
	public static final int DEFAULT_WIDTH = 140;
	public static final int DEFAULT_HEIGHT = 129;
	public static final Color DEFAULT_BOX_COLOR = Color.black;
	public static final Color DEFAULT_WHEEL_COLOR = Color.black;
	public static final Color DEFAULT_ROVER_COLOR = Color.black;
	public static final Color DEFAULT_FORWARD_COLOR = Color.orange;
	public static final Color DEFAULT_REVERSE_COLOR = Color.pink;
	
	private ArrayList<double[]> speeds;
	public UpdateSpeed updateSpeed;

	private int pX, pY, wheelDistWidth,wheelDistHeight;
	private int wheelWidth, wheelHeight, a,b;
//---------------------------             CONSTRUCTORS             ---------------------------//
	/**
	 * creates a 
	 */
	public RoverPicPanel(int x, int y, float transparency) {
		this(x,y,DEFAULT_WIDTH, DEFAULT_HEIGHT,transparency);
	}
	public RoverPicPanel(int x, int y, int width, int height, float transparency) {
		super(x,y,width,height,transparency);
		speeds = new ArrayList<double[]>();
		for(int i = 0; i < 4; i++) speeds.add(new double[1]); //creates an empty set of numbers
		initVars();
	}
	public RoverPicPanel(int x, int y, float transparency, double[]speed1, double[] speed2, double[] speed3, double[] speed4) {
		this(x,y,DEFAULT_WIDTH, DEFAULT_HEIGHT,transparency, speed1,speed2,speed3,speed4);
	}
	public RoverPicPanel(int x, int y, int width, int height, float transparency, double[]speed1, double[] speed2, double[] speed3, double[] speed4) {
		super(x,y,width,height,transparency);
		speeds = new ArrayList<double[]>();
		setTelemetry(speed1,speed2,speed3,speed3);
		initVars();
	}
	public RoverPicPanel(int x, int y, float transparency, ArrayList<double[]> speeds) {
		this(x,y,DEFAULT_WIDTH, DEFAULT_HEIGHT,transparency, speeds);
	}
	public RoverPicPanel(int x, int y, int width, int height, float transparency, ArrayList<double[]> speeds) {
		super(x,y,width,height,transparency);
		this.speeds = speeds;
		initVars();
	}
	private void initVars() {
		pX = 10; pY = 10;
		wheelDistWidth = 50; wheelDistHeight = 10;
		wheelWidth = 15; wheelHeight = 50;
		a = pX; b = pY;
		updateSpeed = new UpdateSpeed(speeds);
	}
//---------------------------               FUNCTIONS               ---------------------------//
	/**
	 * sets the telemetry data to be used. Should be an Arraylist with 4 double[1].
	 */
	public void setTelemetry(ArrayList<double[]> speeds) {
		this.speeds = speeds;
	}
	public void setTelemetry(double[]speed1, double[] speed2, double[] speed3, double[] speed4) {
		speeds.set(0,speed1);
		speeds.set(1,speed2);
		speeds.set(2,speed3);
		speeds.set(3,speed4);
	}

	@Override
	public void paintBuffer( Graphics g ) {
		super.paintComponent(g);
		g.setColor(DEFAULT_BOX_COLOR);
		g.drawRect(0, 0, width-1, height-1);
		
		//draw wheels
		for(int i = 0; i < 4; i++) {
			if(i == 0) {
				a = pX; b = pY;
			} else if(i == 1) {
				a = pX; b = pY+wheelDistHeight+wheelHeight;
			} else if(i == 2) {
				a = pX+wheelDistWidth+wheelWidth; b = pY;
			} else {
				a = pX+wheelDistWidth+wheelWidth; b = pY+wheelDistHeight+wheelHeight;
			}
			g.setColor(DEFAULT_WHEEL_COLOR);
			g.drawRect(a,b, wheelWidth, wheelHeight);
			if(speeds.get(i)[0] >= 0) {
				g.setColor(DEFAULT_FORWARD_COLOR);
				g.fillRect(a+1,(int)(b+wheelHeight*(1.0-(speeds.get(i)[0]/127.0))), wheelWidth-2, (int)((wheelHeight)*(speeds.get(i)[0]/127.0)));
			} else {
				g.setColor(DEFAULT_REVERSE_COLOR);
				g.fillRect(a+1,(int)(b+wheelHeight*(1.0-(speeds.get(i)[0]/-127.0))), wheelWidth-2, (int)((wheelHeight)*(speeds.get(i)[0]/-127.0)));
			}
		}
		//draw text
		g.setColor(DEFAULT_WHEEL_COLOR);
		for(int i = 0; i < 4; i++) {
			if(i == 0) {
				a = pX+wheelWidth+3; b = pY+wheelHeight/2;
			} else if(i == 1) {
				a = pX+wheelWidth+3; b = pY+wheelHeight/2+wheelDistHeight+wheelHeight;
			} else if(i == 2) {
				a = pX+wheelWidth+3+wheelDistWidth+wheelWidth; b = pY+wheelHeight/2;
			} else {
				a = pX+wheelWidth+3+wheelDistWidth+wheelWidth; b = pY+wheelHeight/2+wheelDistHeight+wheelHeight;
			}
			g.drawString(""+speeds.get(i)[0], a,b);
		}//for loop
	}//paintBuffer method

	/**
	 * Redraws the GUIComponent.
	 */
	public void updateDisplay() {
		this.repaint();
	}
	
	public class UpdateSpeed extends UpdateFunction {
		private ArrayList<double[]> speeds, temp;
		
		public UpdateSpeed(ArrayList<double[]> speeds) {
			this.speeds = speeds;
			this.temp = new ArrayList<double[]>();
			for(int i = 0; i < speeds.size(); i++)
				temp.add(speeds.get(i));
		}
		/**
		 * change the pointer of the telemetry to a new set of data.
		 */
		public void setTelemetry(ArrayList<double[]> speeds) {
			this.speeds = speeds;
		}
		
		public boolean checkValues() 
		{
			for(int i = 0; i < speeds.size(); i++)
				if(temp.get(i) != speeds.get(i)) return true;
			return false;
		}
	
		public void doUpdate() {
			for(int i = 0; i < speeds.size(); i++) {
				temp.set(i, speeds.get(i));
			}
			updateDisplay();
		}
	} // end of update speed class 
//---------------------------             TESTING MAIN              ---------------------------//
	public static void main(String[] args) {
			boolean v1 = true; 
			int A= 75;
			int B=100;
			double[] speed1 = new double[]{0};
			double[] speed2 = new double[]{0};
			double[] speed3 = new double[]{0};
			double[] speed4 = new double[]{0};
   		//-------------FRAME---------------
   		
   		JFrame frame = new JFrame(); 
   		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
   		frame.setTitle("Speed - test");
   		//frame.setSize(800, 800);
   		
   		//-----------Main panel------------
			RoverPicPanel constructorCall = new RoverPicPanel(A,B,1f, speed1,speed2,speed3,speed4);
   		frame.add(constructorCall);
			frame.pack();
   		frame.setVisible(true);
			int dir = -1;
   		do
   		{
   			if(speed1[0] >= 120 || speed1[0] <= -120) {
					try { Thread.sleep(1000);
					} catch (InterruptedException e) { }
					speed1[0] = 0;
					dir *= -1;
				}
				else speed1[0] += 10*dir;
				speed2[0] = speed1[0];
				speed3[0] = speed2[0];
				speed4[0] = speed3[0];
				constructorCall.updateDisplay();
				//System.out.printf("%d %d %d\n", (int)speeds[0], (int)bucket[0], (int)conveyer[0]);
				try { Thread.sleep(1000);
   			} catch (InterruptedException e) { }
   		} while (v1);
   	}//testing main
}//RoverPicPanel class
