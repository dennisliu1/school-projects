/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package gui.testing;

import gui.component.GUIComponent;

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
public class LunabotPicPanel extends GUIComponent {
	public static final int DEFAULT_WIDTH = 200;
	public static final int DEFAULT_HEIGHT = 90;
	public static final Color DEFAULT_BOX_COLOR = Color.black;
	public static final Color DEFAULT_WHEEL_COLOR = Color.black;
	public static final Color DEFAULT_ROVER_COLOR = Color.black;
	public static final Color DEFAULT_FORWARD_COLOR = Color.orange;
	public static final Color DEFAULT_REVERSE_COLOR = Color.pink;
	
	ArrayList<double[]> speeds;
	public UpdateSpeed updateSpeed;
	private int s;
//---------------------------             CONSTRUCTORS             ---------------------------//
	public LunabotPicPanel(int x, int y, float transparency, double[] bucket, double[] conveyer) {
		this(x,y,DEFAULT_WIDTH, DEFAULT_HEIGHT,transparency, bucket, conveyer);
	}
	public LunabotPicPanel(int x, int y, int width, int height, float transparency, double[] bucket, double[] conveyer) {
		super(x,y,width,height,transparency);
		speeds = new ArrayList<double[]>();
		speeds.add(bucket);
		speeds.add(conveyer);
		initVars();
	}
	public LunabotPicPanel(int x, int y, float transparency, ArrayList<double[]> speeds) {
		this(x,y,DEFAULT_WIDTH, DEFAULT_HEIGHT,transparency, speeds);
	}
	public LunabotPicPanel(int x, int y, int width, int height, float transparency, ArrayList<double[]> speeds) {
		super(x,y,width,height,transparency);
		this.speeds = speeds;
		initVars();
	}
	public void initVars() {
		updateSpeed = new UpdateSpeed(speeds);
		s = 40;
	}
//---------------------------               FUNCTIONS               ---------------------------//
	@Override
	public void paintBuffer( Graphics g ) {
		super.paintComponent(g);
		g.setColor(DEFAULT_BOX_COLOR);
		g.drawRect(0, 0, width-1, height-1);
		for(int z = 0; z < speeds.size(); z++) {
			if(speeds.get(z)[0] > 0) {
				g.setColor(new Color(255,0,0));
				for(int i = 0; i < (int)(100*(speeds.get(z)[0]/127.0)); i += 10) {
					g.fillRect( 10+(i/10)*18, 20+(z*s), 15, 20 );
				}
				g.setColor(Color.BLACK);
				for(int i = (int) (100*(speeds.get(z)[0]/127.0)); i < 100; i += 10)
					g.fillRect( 10+(i/10)*18, 20+(z*s), 15, 20 );
			} else {
				g.setColor(new Color(0,0,255));
				for(int i = 0; i > (int)(100*(speeds.get(z)[0]/127.0)); i -= 10) {
					g.fillRect( 10+(-i/10)*18, 20+(z*s), 15, 20 );
				}
				g.setColor(Color.BLACK);
				for(int i = (int)(100*(speeds.get(z)[0]/127.0)); i > -100; i -= 10)
					g.fillRect( 10+(-i/10)*18, 20+(z*s), 15, 20 );
			}
			if(z == 0) g.drawString("bucket: "+speeds.get(z)[0], width/4, 14+(z*s));
			else g.drawString("conveyor: "+speeds.get(z)[0], width/4, 14+(z*s));
		}
	}
	public void updateDisplay() {
		this.repaint();
	}
	
	public class UpdateSpeed extends UpdateFunction {
		ArrayList<double[]> speeds, temp;
		//the values are different and the screen gets updated  
		
		public UpdateSpeed(ArrayList<double[]> speeds) 
		{
			this.speeds = speeds;
			this.temp = new ArrayList<double[]>();
			for(int i = 0; i < speeds.size(); i++)
				temp.add(speeds.get(i));
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
			double[] bucket = new double[1];
			double[] conveyer = new double[1];
   		//-------------FRAME---------------
   		
   		JFrame frame = new JFrame(); 
   		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
   		frame.setTitle("Speed - test");
   		//frame.setSize(800, 800);
   		
   		//-----------Main panel------------
			LunabotPicPanel constructorCall = new LunabotPicPanel(A,B,1f, bucket, conveyer);
   		frame.add(constructorCall);
			frame.pack();
   		frame.setVisible(true);
			int dir = -1;
   		do
   		{
   			if(bucket[0] >= 120 || bucket[0] <= -120) {
					bucket[0] = 0;
					conveyer[0] = 0;
					dir *= -1;
				}
				bucket[0] += 10*dir;
				conveyer[0] += 10*dir;
				constructorCall.updateDisplay();
				//System.out.printf("%d %d %d\n", (int)speeds[0], (int)bucket[0], (int)conveyer[0]);
				try { Thread.sleep(100);
   			} catch (InterruptedException e) { }
   		} while (v1);
   		//v1 is a variable which is always true, so the loop never ends. 
   	}//testing main
}
