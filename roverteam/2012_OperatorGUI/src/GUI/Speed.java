package GUI;

import java.awt.Color;
import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.Random;

import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

public class Speed extends GUIComponent {
	public static final int DEFAULT_WIDTH = 200;
	public static final int DEFAULT_HEIGHT = 40;
	public static final Color DEFAULT_BOX_COLOR = Color.black;
	
	public UpdateSpeed uSpeed;
	public final int MAXAveragespeed  = 100;
	private ArrayList<double[]> speeds;
	private double Averagespeed;
	private Color[] colorRect, colorReverse;
//---------------------------             CONSTRUCTORS             ---------------------------//
	private Speed(int x, int y, float transparency) {
		this(x,y,DEFAULT_WIDTH, DEFAULT_HEIGHT,transparency);
	}
	private Speed(int x, int y, int width, int height, float transparency) {
		super(x,y,width,height,transparency);
		initVars();
	}
	public Speed(int x, int y, float transparency, double[]speed1, double[]speed2,double[] speed3, double[] speed4) {
		this(x,y,DEFAULT_WIDTH, DEFAULT_HEIGHT,transparency, speed1,speed2,speed3,speed4);
	}
	public Speed(int x, int y, int width, int height, float transparency, double[]speed1, double[]speed2,double[] speed3, double[] speed4) {
		super(x,y,DEFAULT_WIDTH, DEFAULT_HEIGHT,transparency);
		setTelemetry(speed1, speed2, speed3, speed4);
		initVars();
	}
	public Speed(int x, int y, float transparency, ArrayList<double[]> speeds) {
		this(x,y,DEFAULT_WIDTH, DEFAULT_HEIGHT,transparency, speeds);
	}
	public Speed(int x, int y, int width, int height, float transparency, ArrayList<double[]> speeds) {
		super(x,y,width,height,transparency);
		setTelemetry(speeds);
		initVars();
	}
	private void initVars() {
		colorRect = new Color[11];
		int color = 250;
		for(int i = colorRect.length-1; i >= 0; i--) {
			colorRect[i] = new Color(color,0,0);
			color -= 10;
		}
		colorReverse = new Color[11];
		color = 250;
		for(int i = colorReverse.length-1; i >= 0; i--) {
			colorReverse[i] = new Color(0,0,color);
			color -= 10;
		}
		//colorReverse
		uSpeed = new UpdateSpeed(speeds);
	}
//---------------------------              FUNCTIONS              ---------------------------//
	public void setTelemetry(double[]speed1, double[]speed2,double[] speed3, double[] speed4) {
		speeds = new ArrayList<double[]>();
		speeds.add(speed1);
		speeds.add(speed2);
		speeds.add(speed3);
		speeds.add(speed4);
	}
	public void setTelemetry(ArrayList<double[]> speeds) {
		this.speeds = speeds;
	}
	public void updateDisplay(double AverageSpeed) {
		this.Averagespeed = AverageSpeed;
		this.repaint();
	}
	@Override
	public void paintBuffer( Graphics g ) {
		super.paintComponent(g);
		g.setColor(DEFAULT_BOX_COLOR);
		g.drawRect(0, 0, width-1, height-1);
		int y = 15;
		if(Averagespeed > 0) {
			for(int i = 0; i < Averagespeed; i += 10) {
				g.setColor(colorRect[i/10]);
				g.fillRect( 10+(i/10)*18, y, 15, 20 );
			}
			g.setColor(Color.BLACK);
			for(int i = (int) Averagespeed; i < 100; i += 10)
				g.fillRect( 10+(i/10)*18, y, 15, 20 );
		} else {
			for(int i = 0; i > Averagespeed; i -= 10) {
				g.setColor(colorReverse[-i/10]);
				g.fillRect( 10+(-i/10)*18, y, 15, 20 );
			}
			g.setColor(Color.BLACK);
			for(int i = (int) Averagespeed; i > -100; i -= 10)
				g.fillRect( 10+(-i/10)*18, y, 15, 20 );
		}
		g.drawString(""+Averagespeed, width/2-y, 14);
	}

	public class UpdateSpeed extends UpdateFunction {
		ArrayList<double[]> speeds, temp;
		double AverageSpeed;
		double prevSpeed = 0; // set the speed to 0, that way, right off the bat, 
		//the values are different and the screen gets updated  
		
		public UpdateSpeed(ArrayList<double[]> speeds) 
		{
			this.speeds = speeds;
			this.temp = new ArrayList<double[]>();
			for(int i = 0; i < speeds.size(); i++)
				temp.add(speeds.get(i));
			this.AverageSpeed = (speeds.get(0)[0]+speeds.get(1)[0]+speeds.get(2)[0]+speeds.get(3)[0])/4*127; //sets the speed 
			//System.out.println("average is "+ AverageSpeed);
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
			this.AverageSpeed = (speeds.get(0)[0]+speeds.get(1)[0]+speeds.get(2)[0]+speeds.get(3)[0])/4; //sets the speed 
			updateDisplay(Averagespeed);
		}
	} // end of update speed class 
//----------------------------        TEST        ------------------------------//
   	public static void main(String[] args) {
			boolean v1 = true; 
			int A= 75;
			int B=100;
			int width = 600;
			int height = 600;	
			Random x = new Random(); 
   		
   	//------------1.Init variables-----------------//
   		//-------------FRAME---------------
   		
   		JFrame frame = new JFrame(); 
   		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
   		frame.setTitle("Speed - test");
   		frame.setSize(800, 800);
   		
   		//-----------Main panel------------
   		JPanel fpanel = new JPanel(); 
   		fpanel.setOpaque(true);
   		fpanel.setBackground(Color.black);
   		//frame.add(fpanel);
   		fpanel.setLayout(null);
   		double speed1[] = new double[]{0}, speed2[] = new double[]{0}, speed3[] = new double[]{0}, speed4[] = new double[]{0};
			Speed constructorCall = new Speed(A,B,width,height,1f,speed1,speed2,speed3,speed4);
   		fpanel.add(constructorCall);
   		frame.add(fpanel);
   		frame.setVisible(true);
   		do
   		{
   			if(speed1[0] >= 100) {
				try { Thread.sleep(1000);
				} catch (InterruptedException e) { }
				speed1[0] = -100;
			}
			else speed1[0] += 10;
			speed2[0] = speed1[0];
			speed3[0] = speed2[0];
			speed4[0] = speed3[0];
			System.out.println(speed1[0]+"\n");
			constructorCall.updateDisplay((speed1[0]+speed2[0]+speed3[0]+speed4[0])/4);
			try { Thread.sleep(100);
   			} catch (InterruptedException e) { }
   		} while (true);
   		//v1 is a variable which is always true, so the loop never ends. 
   	}//testing main
}//Speed class
