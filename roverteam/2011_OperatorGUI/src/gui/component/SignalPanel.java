package gui.component;



import java.awt.Color;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.Graphics;
import java.util.Date;
import java.util.Random;
import javax.swing.JFrame;
import javax.swing.JLabel;

/**
 * 
 * 
 * @version 1.0
 * @author Dennis Liu, YURT 2012
 */
public class SignalPanel extends GUIComponent {
	/** default width for the panel */public static final int DEFAULT_WIDTH = 200;
	/** default height for the panel */public static final int DEFAULT_HEIGHT = 40;
	//-------------------------------------		Variables			-------------------------------------//
	/** variable to use to update the panel */			private String[] signal;
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
	public SignalPanel(int x, int y, int width, int height, float transparency, String[] signal) {
		super(x,y,width,height,transparency);
		this.signal = signal;
		initVars();
	}
	/**
	 * Construct the Timer Panel with default size.
	 * @param x x coordinate of the panel for use by JLayeredPane
	 * @param y y coordinate of the panel for use by JLayeredPane
	 * @param timer a number(Long) that is to be displayed. The array should be of length 1. 
	 * @throws Exception if timer is not of length 1, creation fails.
	 */
	public SignalPanel(int x, int y, float transparency, String[] signal) {
		this(x,y, DEFAULT_WIDTH,DEFAULT_HEIGHT, transparency, signal);
	}
	public SignalPanel(int x, int y, int width, int height, float transparency) {
		super(x,y, width, height, transparency);
		this.signal = new String[1];
	}
	public SignalPanel(int x, int y, float transparency) {
		this(x,y, DEFAULT_WIDTH,DEFAULT_HEIGHT, transparency);
	}
	private void initVars() {
		updateDisplay();//default values for the display
	}
	//-------------------------------------		Functions		-------------------------------------//
	public String[] getTelemetry() {
		return signal;
	}
	/**
	 * set the time data. Simply points to the new data.
	 */
	public void setTelemetry(String[] signal) {
		this.signal = signal;
	}
	public void paintBuffer(Graphics g) {
		String s = ("Sig:"+signal[0]);
		g.setFont(font);
		g.drawChars(s.toCharArray(), 0, s.length(), 5, font.getSize());
		g.drawRect(0, 0, width-1, height-1);
	}
	//-------------------------------------		Testing			-------------------------------------//
	public static void main(String[]a) {
		String[] signal = new String[]{""};//create the shared variable to work with
		
		//display panel on screen
		JFrame frame = new JFrame("Testing Timer Panel");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setLayout(null);
			SignalPanel panel = new SignalPanel(0,0, 1f,signal);
			frame.add(panel);
			TestThread t = new TestThread(signal,panel);//thread that updates the variable
			t.start();
		frame.setSize(300,100);
		//frame.pack();
		frame.setVisible(true);//this should always be last method to run.
	}
	/**
	 * updates the variable with the current time. Try changing the long and watch the 
	 * label display something different! =)
	 */
	public static class TestThread extends Thread {
		boolean isRunning = true;
		boolean pause = false;
		String[] input;
		int a,b;
		Random rand;
		SignalPanel panel;
		
		public TestThread(String[] signal,SignalPanel panel) {
			this.input = signal;
			a = 6;
			b = 12;
			input[0] = a+"/"+b;
			this.panel = panel;
		}
		@Override
		public void run() {
		while(isRunning) {
			while(pause) { try{Thread.sleep(1); } catch(Exception e) {} }//lock thread
			//DO CHANGE HERE
			System.out.println(input[0]);
			if(a < 80 && b < 80) {
				a += 5;
				b += 6;
			} else {
				a = 6;
				b = 12;
			}
			input[0] = a+"/"+b;
			panel.updateDisplay();
			try{Thread.sleep(1000); } catch(Exception e) {}//try changing wait time to update time differently
		}}
	}//UpdateThread class
}