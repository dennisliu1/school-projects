package GUI;



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
	
	public static final Color BACKGROUND_COLOR = Color.yellow;
	public static final Color TEXT_COLOR = Color.GREEN;
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
	

	public UpdateTiltFunction tiltMeterUpdateFunction;
	private JLabel TiltLabel;

	public TiltSensor4(int x, int y, float transparency, int sleepTime,double[] Htilt, double[] Vtilt) {
		this(x,y,(int)MAX_WIDTH, (int)MAX_HEIGHT, transparency, sleepTime, Htilt, Vtilt);
	}
	public TiltSensor4(int x, int y, int width, int height, float transparency, int sleepTime,double[] Htilt, double[] Vtilt) 
	{
		super(x,y, width, height, transparency);

		this.Htilt = Htilt;
		this.Vtilt = Vtilt;
		this.sleepTime = sleepTime;
		
		TiltLabel = new JLabel();
		updateDisplay(this.Htilt[0], this.Vtilt[0]);

		this.setLayout(new FlowLayout());
		this.add(TiltLabel);
		
		this.setBackground(BACKGROUND_COLOR);
		tiltMeterUpdateFunction = new UpdateTiltFunction(this.Htilt, this.Vtilt);
		tiltMeterUpdateFunction.setSleepTime(sleepTime);
		//this.addUpdateThreadFunction(tiltMeterUpdateFunction);
		//this.startUpdate();

	}

	public void updateDisplay(double H, double V)
	{
		//TiltLabel.setText(H + ", " + V);
		repaint();
		//TiltLabel.add();
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
				g2.setColor(Color.black);			
			else
				g2.setColor(Color.black);
			//System.out.println(i);
			g2.drawLine((0+Math.abs(counter)),i,((int)MAX_WIDTH-Math.abs(counter)),i);
			counter = counter - RATE_OF_CHANGE_VERTICAL;
		}

		//((LENGTH_OF_TILT_LINE/2)*Math.cos(Htilt[0]) + MAX_WIDTH/2) for x2
		g2.setColor(Color.green);
		//#######DRAWS ROTATING MAIN LINE
		g2.setStroke(new BasicStroke(2));
		g2.drawLine((int)(MAX_WIDTH/2 - (LENGTH_OF_TILT_LINE/2)*Math.cos(Math.toRadians(Htilt[0]))),
			(int)(MAX_HEIGHT/2 - (LENGTH_OF_TILT_LINE/2)*Math.sin(Math.toRadians(Htilt[0]))), 
			(int)((LENGTH_OF_TILT_LINE/2)*Math.cos(Math.toRadians(Htilt[0])) + MAX_WIDTH/2),
			(int)((LENGTH_OF_TILT_LINE/2)*Math.sin(Math.toRadians(Htilt[0])) + MAX_HEIGHT/2));
		if (DEBUGGING == true)
		{
			String s = "HTilt: " + Htilt[0] + 
				" VTilt: " + Vtilt[0] +
				" Position @ Main: (" + 
				((int)(MAX_WIDTH/2 - (LENGTH_OF_TILT_LINE/2)*Math.cos(Math.toRadians(Htilt[0]))) + ", " +
				(int)(MAX_HEIGHT/2 - (LENGTH_OF_TILT_LINE/2)*Math.sin(Math.toRadians(Htilt[0]))) + ", " +
				(int)((LENGTH_OF_TILT_LINE/2)*Math.cos(Math.toRadians(Htilt[0])) + MAX_WIDTH/2) + ", " +
				(int)((LENGTH_OF_TILT_LINE/2)*Math.sin(Math.toRadians(Htilt[0])) + MAX_HEIGHT/2)) + ")";
			g2.drawChars(s.toCharArray(), 0, s.length(), 0,14);
				
				
		}
		
		g2.setColor(Color.BLACK); //color of main lines.
		g2.drawLine(0, (int)(MAX_HEIGHT/2), (int)MAX_WIDTH, (int)(MAX_HEIGHT/2)); // horizontal line across center.
		g2.drawLine((int)MAX_WIDTH/2, 0, (int)MAX_WIDTH/2, (int)(MAX_HEIGHT)); //vertical line down center.
		
		if (DRAW_PERPENDICULAR == true)
		{
			g2.setColor(Color.WHITE);
			g2.drawLine((int)MAX_WIDTH/2, (int)MAX_HEIGHT/2, 
				(int)(MAX_WIDTH/2 - Vtilt[0]*Math.sin(Math.toRadians(Htilt[0]))),
				(int)(MAX_HEIGHT/2 + Vtilt[0]*Math.cos(Math.toRadians(Htilt[0]))));
		}

		if (DRAW_CONNECTING_LINES == true)
		{
			g2.setColor(Color.BLUE);
			g2.drawLine((int)(MAX_WIDTH/2 - Vtilt[0]*Math.sin(Math.toRadians(Htilt[0]))), 
				(int)(MAX_HEIGHT/2 + Vtilt[0]*Math.cos(Math.toRadians(Htilt[0]))),
				(int)((LENGTH_OF_TILT_LINE/2)*Math.cos(Math.toRadians(Htilt[0])) + MAX_WIDTH/2),
				(int)((LENGTH_OF_TILT_LINE/2)*Math.sin(Math.toRadians(Htilt[0])) + MAX_HEIGHT/2));

			g2.drawLine((int)(MAX_WIDTH/2 - Vtilt[0]*Math.sin(Math.toRadians(Htilt[0]))), 
				(int)(MAX_HEIGHT/2 + Vtilt[0]*Math.cos(Math.toRadians(Htilt[0]))),
				(int)(MAX_WIDTH/2 - (LENGTH_OF_TILT_LINE/2)*Math.cos(Math.toRadians(Htilt[0]))),
				(int)(MAX_HEIGHT/2 - (LENGTH_OF_TILT_LINE/2)*Math.sin(Math.toRadians(Htilt[0]))));
				
		}
		
		
		//g.drawLine(0,0, (int)Htilt[0], (int)Vtilt[0]);
	}

	private class UpdateTiltFunction extends UpdateFunction 
	{
		private double[] Hinput;
		private double[] Vinput;
		
		private double oldH;
		private double oldV;

		public UpdateTiltFunction(double[] Hinput, double[] Vinput)
		{
			this.Hinput = Hinput; 
			this.oldH = this.Hinput[0];
			
			this.Vinput  = Vinput; 
			this.oldV = this.Vinput[0];
		}

		@Override 
		public boolean checkValues()
		{
			if ((oldH == Hinput[0]) && (oldV == Vinput[0]))
			{
			 	return false;
			}
			return true;	
		}

		public void doUpdate()
		{
			oldH = Hinput[0];
			oldV = Vinput[0];
			
			updateDisplay(oldH, oldV); //why do i have to update the old one?
		} //doUpdate 
	}// UpdateTiltFunction 

	public static void main(String [] args)
    	{
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
			test1.updateDisplay(horizontalTilt[0], verticalTilt[0]);
			try { Thread.sleep(250); } catch (InterruptedException e) {e.printStackTrace();}
    	} while (v1=true);


	}

}
