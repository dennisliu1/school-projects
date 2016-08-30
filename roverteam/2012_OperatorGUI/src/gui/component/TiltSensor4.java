package gui.component;





import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.Random;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

/**
@author MACIEJ LIS 
version 1.0 (beta)
email: mlisbit@gmail.com
*/
public class TiltSensor4 extends GUIComponent 
{
	private double[] Htilt;
	private double[] Vtilt;
	private int sleepTime;
	
	public static final Color TEXT_COLOR = Color.GRAY;
	public static final Color CONNECTING_LINE_COLOR = Color.orange;
	public static final Color PERPENDICULAR_COLOR = Color.darkGray;
	public static final Color CONNECTING_LINES_COLOR = Color.black;
	
	public static final Color AXIS_LINES_COLOR = Color.darkGray;
	public static final Color HORIZONTAL_LINES_COLOR = Color.red;
	public static final Color HORIZONTAL_THRESHOLD_LINES_COLOR = Color.gray;
	public static final double MAX_HEIGHT = 200;
	public static final double MAX_WIDTH = 200;
	//preffered even numbers and MAX_HEIGHT % NOVD =0
	public static final int NUMBER_OF_VERTICAL_DIVIDERS = 10 ; 
	public static final int RATE_OF_CHANGE_VERTICAL = 20;
	public static final int LENGTH_OF_TILT_LINE = (int)((MAX_WIDTH*2)/4);
	public static final boolean DRAW_PERPENDICULAR = true;
	public static final int LENGTH_OF_PERPENDICULAR = (int)((MAX_HEIGHT*3)/4);
	public static final boolean DRAW_CONNECTING_LINES = true;
	public static final boolean DEBUGGING = true; 
	public static final int VERTICALTILT_THRESHOLD = 90;
	public static final int HORIZONTALTILT_THRESHOLD = 80;

	public TiltSensor4(int x, int y, float transparency, int sleepTime,double[] Htilt, double[] Vtilt) {
		this(x,y,(int)MAX_WIDTH, (int)MAX_HEIGHT, transparency, sleepTime, Htilt, Vtilt);
	}
	public TiltSensor4(int x, int y, int width, int height, float transparency, int sleepTime,double[] Htilt, double[] Vtilt) 
	{
		super(x,y, width, height, transparency);

		this.Htilt = Htilt;
		this.Vtilt = Vtilt;
		this.sleepTime = sleepTime;
	}

	public void paintBuffer(Graphics g)  
	{  
		super.paintComponent(g);
		Graphics2D g2 = (Graphics2D) g;
		int counter = (int)(((NUMBER_OF_VERTICAL_DIVIDERS/2) - 1) * RATE_OF_CHANGE_VERTICAL);
		
		for (int i = (int)MAX_HEIGHT/NUMBER_OF_VERTICAL_DIVIDERS ; 
			i <= (int)MAX_HEIGHT; i = i + (int)(MAX_HEIGHT/NUMBER_OF_VERTICAL_DIVIDERS))
		{	
			if ( Math.abs(Vtilt[0]) >= VERTICALTILT_THRESHOLD)
				g2.setColor(HORIZONTAL_LINES_COLOR);			
			else
				g2.setColor(HORIZONTAL_THRESHOLD_LINES_COLOR);
			//System.out.println(i);
			g2.drawLine((0+Math.abs(counter)),i,((int)MAX_WIDTH-Math.abs(counter)),i);
			counter = counter - RATE_OF_CHANGE_VERTICAL;
		}

		//((LENGTH_OF_TILT_LINE/2)*Math.cos(Htilt[0]) + MAX_WIDTH/2) for x2
		g2.setColor(CONNECTING_LINE_COLOR);
		//#######DRAWS ROTATING MAIN LINE
		g2.setStroke(new BasicStroke(2));
		g2.drawLine((int)(MAX_WIDTH/2 - (LENGTH_OF_TILT_LINE/2)*Math.cos(Math.toRadians(Htilt[0]))),
			(int)(MAX_HEIGHT/2 - (LENGTH_OF_TILT_LINE/2)*Math.sin(Math.toRadians(Htilt[0]))), 
			(int)((LENGTH_OF_TILT_LINE/2)*Math.cos(Math.toRadians(Htilt[0])) + MAX_WIDTH/2),
			(int)((LENGTH_OF_TILT_LINE/2)*Math.sin(Math.toRadians(Htilt[0])) + MAX_HEIGHT/2));
		if (DEBUGGING) {
			g2.setColor(TEXT_COLOR);
			String s = "H: " + Htilt[0] + 
				" V: " + Vtilt[0];
			g2.drawChars(s.toCharArray(), 0, s.length(), 0,14);
		}
		
		g2.setColor(AXIS_LINES_COLOR); //color of main lines.
		g2.drawLine(0, (int)(MAX_HEIGHT/2), (int)MAX_WIDTH, (int)(MAX_HEIGHT/2)); // horizontal line across center.
		g2.drawLine((int)MAX_WIDTH/2, 0, (int)MAX_WIDTH/2, (int)(MAX_HEIGHT)); //vertical line down center.
		
		if (DRAW_PERPENDICULAR) {//draws middle line in the triangle
			g2.setColor(PERPENDICULAR_COLOR);
			g2.drawLine((int)MAX_WIDTH/2, (int)MAX_HEIGHT/2, 
				(int)(MAX_WIDTH/2 - Vtilt[0]*Math.sin(Math.toRadians(Htilt[0]))),
				(int)(MAX_HEIGHT/2 + Vtilt[0]*Math.cos(Math.toRadians(Htilt[0]))));
		}

		if (DRAW_CONNECTING_LINES) {//draw v shape of the triangle vector
			g2.setColor(CONNECTING_LINES_COLOR);
			g2.drawLine((int)(MAX_WIDTH/2 - Vtilt[0]*Math.sin(Math.toRadians(Htilt[0]))), 
				(int)(MAX_HEIGHT/2 + Vtilt[0]*Math.cos(Math.toRadians(Htilt[0]))),
				(int)((LENGTH_OF_TILT_LINE/2)*Math.cos(Math.toRadians(Htilt[0])) + MAX_WIDTH/2),
				(int)((LENGTH_OF_TILT_LINE/2)*Math.sin(Math.toRadians(Htilt[0])) + MAX_HEIGHT/2));

			g2.drawLine((int)(MAX_WIDTH/2 - Vtilt[0]*Math.sin(Math.toRadians(Htilt[0]))), 
				(int)(MAX_HEIGHT/2 + Vtilt[0]*Math.cos(Math.toRadians(Htilt[0]))),
				(int)(MAX_WIDTH/2 - (LENGTH_OF_TILT_LINE/2)*Math.cos(Math.toRadians(Htilt[0]))),
				(int)(MAX_HEIGHT/2 - (LENGTH_OF_TILT_LINE/2)*Math.sin(Math.toRadians(Htilt[0]))));
		}
	}

	public static void main(String [] args) {
        	boolean v1 = true;
        	Random rand = new Random(System.nanoTime());
  
       	 	
        	JFrame frame = new JFrame(); 
        	frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        	frame.setTitle("TiltSensor");
        	frame.setSize((int)MAX_WIDTH*2, (int)MAX_HEIGHT*2);
    
		//########TEST TILT FRAME IN MAIN FRAME############
		
		JPanel fpanel = new JPanel();
		fpanel.setBackground(Color.black);
		fpanel.setOpaque(false);
		frame.add(fpanel);
		fpanel.setLayout(null);
		
		
		double[] horizontalTilt = new double[1];
		double[] verticalTilt = new double[1];
		
		horizontalTilt[0] = 0;
		verticalTilt[0] = 0;

		TiltSensor4 test1 = new TiltSensor4(0,0,1f, 100,horizontalTilt, verticalTilt);
		fpanel.add(test1);
	
		frame.add(fpanel);
		frame.setVisible(true);

		do {
   			//horizontalTilt[0] = rand.nextDouble()*100;
			//verticalTilt[0] = - rand.nextDouble()*100;
			if (horizontalTilt[0] >= 0 && horizontalTilt[0] < 120) 
				horizontalTilt[0]++;
			else if (horizontalTilt[0] >= 120)
				horizontalTilt[0]-=125;
			else if ((horizontalTilt[0] <= 0) && (horizontalTilt[0] >= -120)) 
				horizontalTilt[0]--;
			else 
				horizontalTilt[0]+=121;
	
			if (verticalTilt[0] >= 0 && verticalTilt[0] < 100) 
				verticalTilt[0]+=2; 
			else 
				verticalTilt[0] -=50;
			test1.updateDisplay();
			try { Thread.sleep(250); } catch (InterruptedException e) {e.printStackTrace();}
    	} while (v1=true);


	}

}
