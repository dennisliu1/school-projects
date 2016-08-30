package gui.component;



import java.awt.*;
import java.util.Scanner;
import javax.swing.JFrame;

public class Compass extends GUIComponent{
 public static final int DEFAULT_WIDTH = 500;    // panel width
 public static final int DEFAULT_HEIGHT = 70;   // panel height
 public static final int DEFAULT_MARGIN = 5;//UNUSED
 public static final int DEFAULT_VERTI_MARGIN = 25;
 public static final int DEFAULT_TICK_HEIGHT = 15, DEFAULT_LARGE_TICK_HEIGHT = 20;
 public static final int DEFAULT_DIST_SMALL_TICKS_NUM = 90;
 public static final int DEFAULT_DIST_SMALL_TICKS = (DEFAULT_WIDTH - 2*DEFAULT_MARGIN) / DEFAULT_DIST_SMALL_TICKS_NUM;
 public static final int DEFAULT_DIST_LARGE_TICKS = 15;//UNUSED
 public static final int DEFAULT_SMALL_TICKS_DIV = 9;
 public static final int DEFAULT_LARGE_TICKS_DIV = 45;//DEGREES PER TICK
 public static final Color DEFAULT_COMPASS_TICK_COLOR = Color.black;
 public static final Color DEFAULT_COMPASS_NUM_COLOR = Color.black;
 public static final Color DEFAULT_COMPASS_POINTER_COLOR = Color.darkGray;
 public static final Color DEFAULT_BACKGROUND_COLOR = Color.lightGray;
 public static final String[] DEFAULT_COMPASS_STRINGS = new String[]{"N","NE","E","SE","S","SW","W","NW"};
 //public static final int DISTLINE = DEFAULT_WIDTH / 14;    // distance between each line
 private long[] rotate;    // current degree
 //for paint to use //
 private int sleepTime;
 private double middle;
 private int startLine, endLine;
 private Font font = new Font("Arial", Font.PLAIN, 20);
 private int offset;
//---------------------------             CONSTRUCTORS             ---------------------------//
	public Compass(int x, int y, int width, int height, float transparency, long[] degree) {
	  super(x, y, width, height, transparency);
	  if(degree == null) this.rotate = new long[1];
	  else this.rotate = degree;
	  if(degree[0] > 360 || degree[0] < 0) this.rotate = new long[0];
		else this.rotate = degree;
		initVars();
	}
	public Compass(int x, int y, float transparency, long[] degree) {
		this(x,y, DEFAULT_WIDTH,DEFAULT_HEIGHT,transparency, degree);
	}
	public Compass(int x, int y, int width, int height, float transparency) {
		super(x, y, width, height, transparency);
		this.rotate = new long[1];
		initVars();
	}
	public Compass(int x, int y, float transparency) {
		this(x,y, DEFAULT_WIDTH,DEFAULT_HEIGHT,transparency);
	}
	private void initVars() {
		startLine = DEFAULT_MARGIN + DEFAULT_DIST_SMALL_TICKS*((DEFAULT_DIST_SMALL_TICKS_NUM/2)+DEFAULT_DIST_SMALL_TICKS_NUM%2);
		endLine = this.getHeight() - (this.getHeight() / 4);
		offset = 0;
	}
//---------------------------             FUNCTIONS             ---------------------------//
	public int getOffset() {
		return offset;
	}
	public void setOffset(int offset) {
		this.offset = offset;
	}
	public void setTelemetry(long[] degree) {
		this.rotate = degree;
	}
	public void paintBuffer(Graphics g) {
		Graphics2D g2 = (Graphics2D)g;
		g2.setFont(font);    // setting font and size of numbers
		super.paintComponent(g);			    // clears screen before painting
		g2.setColor(DEFAULT_COMPASS_TICK_COLOR);
		g2.drawRect(0, 0, width-1, height-1);
		middle = (double) ((int) rotate[0]+offset - ((this.getWidth() / DEFAULT_DIST_SMALL_TICKS) / 2));//tick amount on screen
		// left most number (used to determine the middle number)
		for(int i = 0; i <= this.getWidth(); i = i + DEFAULT_DIST_SMALL_TICKS) {
			if(middle < 0) middle = 360 + middle;// if degree is ever negative than add 360 (makes sure the current degree is correct)
			else if(middle > 359) middle = 0; 
				// if current degree is greater than 359 than set current degree to 0
				// determines if number is multiple of 5
			if(middle%DEFAULT_SMALL_TICKS_DIV == 0) {
				g2.setColor(DEFAULT_COMPASS_TICK_COLOR);
				g2.drawLine(i, this.getHeight()-DEFAULT_VERTI_MARGIN, i, this.getHeight() - DEFAULT_TICK_HEIGHT-DEFAULT_VERTI_MARGIN);
			}
			if(middle%DEFAULT_LARGE_TICKS_DIV == 0) {
				g2.setColor(DEFAULT_COMPASS_TICK_COLOR);
				g2.drawLine(i, this.getHeight()-DEFAULT_VERTI_MARGIN, i, this.getHeight() - DEFAULT_LARGE_TICK_HEIGHT-DEFAULT_VERTI_MARGIN);
				g2.setColor(DEFAULT_COMPASS_NUM_COLOR);
				g2.drawString(DEFAULT_COMPASS_STRINGS[(int)middle/DEFAULT_LARGE_TICKS_DIV], i - 10, this.getHeight() - DEFAULT_LARGE_TICK_HEIGHT-DEFAULT_VERTI_MARGIN-3);  // draws number
			}
			middle++;
		}

		g2.setFont(new Font("Arial", Font.PLAIN, 20));
		g2.setColor(DEFAULT_COMPASS_POINTER_COLOR); // draws triangle and sets its color to red
//		g2.drawLine(startLine - 30, this.getHeight(), 
//								startLine, endLine);
//		g2.drawLine(startLine + 30, this.getHeight(), 
//								startLine, endLine);
		g2.drawLine(width/2, height, width/2, height-15);
		g2.drawString(""+rotate[0], width/2+10, height-4);
		g2.setColor(Color.BLACK);
	}
//------------------------------        TEST         ---------------------------//
	public static void main(String[] args){
		long[] d = new long[1];
		JFrame f = new JFrame();
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setLayout(null);
		f.setSize(800, 450);
		Compass p = new Compass(0, 0,1f, d);
			TestThread t = new TestThread(d, p);
			t.start();
		f.getContentPane().add(p);
		f.setVisible(true);
	}
	private static class TestThread extends Thread {
		Scanner inp = new Scanner(System.in);
		boolean isRunning = true;
		boolean pause = false;
		long[] input;
		Compass c;
		public TestThread(long[] input, Compass c) {
			this.input = input;
			this.c = c;
		}
		public void run() {
			while(true) {
				if(this.input[0] == 360) this.input[0] = 0;
				else this.input[0] += 1;
				c.updateDisplay();
				System.out.printf("%d\n", this.input[0]);
				try { Thread.sleep(100); } catch (InterruptedException e) { }
				if(this.input[0] % 15 == 0)
					try { Thread.sleep(1000); } catch (InterruptedException e) { }
			}
		}
	}
}//Compass class
