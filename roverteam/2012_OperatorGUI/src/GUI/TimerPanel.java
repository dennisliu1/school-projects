package GUI;

import java.awt.Color;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.Graphics;
import java.util.Date;
import java.util.Random;
import javax.swing.JFrame;
import javax.swing.JLabel;

/**
 * Creates a Panel that displays the current time. Use this as an example of how to program a component.
 * The updating variable (timer) is passed by reference using an array. 
 * 
 * Please note the comments and the style of the program. Everything here is commented unless obvious, and
 * Javadoc is used for all classes/variables. You don't need to comment trivial things, or the main, unless it is 
 * something important or special. Your code should be easy to read and variables should be meaningful.
 * 
 * Running this under an IDE with color is recommended for easier reading.
 * 
 * @version 1.0
 * @author Dennis Liu, YURT 2012
 */
public class TimerPanel extends GUIComponent {
	/** default width for the panel */public static final int DEFAULT_WIDTH = 150;
	/** default height for the panel */public static final int DEFAULT_HEIGHT = 40;
	//-------------------------------------		Variables			-------------------------------------//
	/** variable to use to update the panel */			private long[] time;
	/** Convertor of the format of the date to string */	private Date date;
	public UpdateTimeFunction updateTimer;
	private Font font = new Font("Arial", Font.PLAIN, 32);
	//-------------------------------------		Constructor		-------------------------------------//
	/**
	 * Main Constructor. Creates a JPanel that updates the display with the timer passed.
	 * @param x x coordinate of the panel for use by JLayeredPane
	 * @param y y coordinate of the panel for use by JLayeredPane
	 * @param width  preferred width of the panel
	 * @param height preferred height of the panel
	 * @param sharedTime a number(Long) that is to be displayed. The array should be of length 1. 
	 * @throws Exception if timer is not of length 1, creation fails.
	 */
	public TimerPanel(int x, int y, int width, int height, float transparency, long[] sharedTime) {
		super(x,y,width,height,transparency);
			//If time data is piped to panel, use it. Else, make a placeholder for now.
			if(sharedTime == null) this.time = new long[1];
			else this.time = sharedTime;
			initVars();
		
	}
	/**
	 * Construct the Timer Panel with default size.
	 * @param x x coordinate of the panel for use by JLayeredPane
	 * @param y y coordinate of the panel for use by JLayeredPane
	 * @param timer a number(Long) that is to be displayed. The array should be of length 1. 
	 * @throws Exception if timer is not of length 1, creation fails.
	 */
	public TimerPanel(int x, int y, float transparency, long[] timer) {
		this(x,y, DEFAULT_WIDTH,DEFAULT_HEIGHT, transparency, timer);
	}
	public TimerPanel(int x, int y, int width, int height, float transparency) {
		super(x,y, width, height, transparency);
		this.time = new long[1];
	}
	public TimerPanel(int x, int y, float transparency) {
		this(x,y, DEFAULT_WIDTH,DEFAULT_HEIGHT, transparency);
	}
	private void initVars() {
		date= new Date();
		updateDisplay();//default values for the display
		//----------------		Panel Construction			---------------------//
		this.setLayout(new FlowLayout());
		this.setBackground(Color.green);
		updateTimer = new UpdateTimeFunction(this.time);
	}
	//-------------------------------------		Functions		-------------------------------------//
	/**
	 * set the time data. Simply points to the new data.
	 */
	public void setTelemetry(long[] sharedTime) {
		this.time = sharedTime;
	}
	public void paintBuffer(Graphics g) {
		String s = (int)(time[0]/3600)+":"+(int)((time[0]%3600)/60)+":"+time[0]%60;
		g.setFont(font);
		g.drawChars(s.toCharArray(), 0, s.length(), 5, font.getSize());
		g.drawRect(0, 0, width-1, height-1);
	}
	/**
	 * Update Panel's display with new time.
	 * @param newTime 
	 */
	public void updateDisplay() {
		this.repaint();
	}
	//-------------------------------------		Threading			-------------------------------------//
	/**
	 * Function to run in the update thread. Note several things:
	 * - the monitored variable is passed by reference, and the previous time is
	 * kept.
	 * - checkValues() works when the values do not match; you should have something
	 * like this as well.
	 * - doUpdate() is where you update the display (updateDisplay should run here)
	 */
	public class UpdateTimeFunction extends UpdateFunction {
		private long[] input;
		private long oldTime;

		public UpdateTimeFunction(long[] input) {
			this.input = input;
			this.oldTime = this.input[0];//default current time display
		}

		@Override
		public boolean checkValues() {
			return (oldTime != input[0]);
			//return true;
		}

		@Override
		public void doUpdate() {
			oldTime = input[0];//update old time
			updateDisplay();//update timer display
		}
	}
	//-------------------------------------		Testing			-------------------------------------//
	public static void main(String[]a) {
		long[] time = new long[1];//create the shared variable to work with
		TestThread t = new TestThread(time);//thread that updates the variable
		t.start();
		
		//display panel on screen
		JFrame frame = new JFrame("Testing Timer Panel");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setLayout(null);
                TimerPanel panel = new TimerPanel(0,0, 1f,time);
                frame.add(panel);
		frame.setSize(200,100);
		//frame.pack();
		frame.setVisible(true);//this should always be last method to run.
	}
	/**
	 * updates the variable with the current time. Try changing the long and watch the 
	 * label display something different! =)
	 */
	private static class TestThread extends Thread {
		boolean isRunning = true;
		boolean pause = false;
		long[] input;
		Random rand;
		public TestThread(long[] input) {
			this.input = input;
			input[0] = 3610;
		}
		@Override
		public void run() {
		while(isRunning) {
			while(pause) { try{Thread.sleep(1); } catch(Exception e) {} }//lock thread
			//DO CHANGE HERE
			System.out.println(input[0]);
			input[0]-= 1;
			try{Thread.sleep(1000); } catch(Exception e) {}//try changing wait time to update time differently
		}}
	}//UpdateThread class
}