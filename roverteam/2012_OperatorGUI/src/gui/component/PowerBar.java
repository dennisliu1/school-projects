package gui.component;




import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.util.Random;
import javax.swing.JFrame;
import javax.swing.JPanel;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author dennis
 */
public class PowerBar extends GUIComponent {
	public static final int DEFAULT_BATTERY_WIDTH = 100;
	public static final int DEFAULT_BATTERY_HEIGHT = 40;
	public static final int DEFAULT_BATTERY_MARGIN = 5;
	public static final int DEFAULT_BAR_WIDTH = 80;
	public static final int DEFAULT_BAR_HEIGHT = 30;
	public static final int DEFAULT_BATTERY_NUB_HEIGHT = 20;
	public static final int DEFAULT_BATTERY_NUB_WIDTH = 10;
	public static final Color DEFAULT_BOX_COLOR = Color.BLACK;
	public static final Color DEFAULT_BATTERY_COLOR = Color.green;
	public static final Color DEFAULT_FONT_COLOR = Color.black;
	public static final Color DEFAULT_BACKGROUND_COLOR = Color.gray;
	
	private JPanel batteryLevelPanel;
	private double[] power;
//---------------------------             CONSTRUCTORS             ---------------------------//
	public PowerBar(int x,int y,float transparency,double[] power) {
		this(x,y,DEFAULT_BATTERY_WIDTH,DEFAULT_BATTERY_HEIGHT,transparency,power);
	}
	public PowerBar(int x, int y, int width, int height,float transparency, double[] power) {
		super(x,y,width,height,transparency);
		this.power = power;
	}
	public PowerBar(int x,int y,float transparency) {
		this(x,y,DEFAULT_BATTERY_WIDTH,DEFAULT_BATTERY_HEIGHT,transparency);
	}
	public PowerBar(int x, int y, int width, int height,float transparency) {
		super(x,y,width,height,transparency);
		this.power = new double[1];
	}
//---------------------------               FUNCTIONS               ---------------------------//
	public void setTelemetry(double[] power) {
		this.power = power;
	}
	@Override
	public void paintBuffer(Graphics g) {
		//super.paint(g);
		super.paintComponents(g);
		g.setColor(DEFAULT_BACKGROUND_COLOR);//draw background
		g.fillRect(0, 0, width-1, height-1);
		g.setColor(DEFAULT_BOX_COLOR);
		g.drawRect(0, 0, width-1, height-1);
		g.drawRect(DEFAULT_BATTERY_MARGIN-1, DEFAULT_BATTERY_MARGIN-1, DEFAULT_BAR_WIDTH+1, DEFAULT_BAR_HEIGHT+1);
		g.fillRect(DEFAULT_BATTERY_MARGIN+DEFAULT_BAR_WIDTH, this.getHeight()/2-DEFAULT_BATTERY_NUB_HEIGHT/2, 
						DEFAULT_BATTERY_MARGIN, DEFAULT_BATTERY_NUB_HEIGHT);
//		g.setColor(DEFAULT_BATTERY_COLOR);//draw battery power
//		g.fillRect(DEFAULT_BATTERY_MARGIN, DEFAULT_BATTERY_MARGIN, 
//						(int)(power[0]*(DEFAULT_BAR_WIDTH)), DEFAULT_BAR_HEIGHT);
		g.setColor(DEFAULT_BATTERY_COLOR);//draw battery power
		g.fillRect(DEFAULT_BATTERY_MARGIN, DEFAULT_BATTERY_MARGIN, DEFAULT_BAR_WIDTH, DEFAULT_BAR_HEIGHT);
		g.setColor(DEFAULT_FONT_COLOR);
		g.setFont(new Font("arial", Font.PLAIN, 20));
		String s = Double.toString(power[0]);
		if(s.length() > 6) s = s.substring(0, 6);
		g.drawString(s, (DEFAULT_BAR_WIDTH/6)-DEFAULT_BATTERY_MARGIN, (DEFAULT_BAR_HEIGHT/2)+2*DEFAULT_BATTERY_MARGIN);
	}
//---------------------------               TESTING METHODS               ---------------------------//
	public static void main(String[] args) {
		double[] d = new double[1];
		JFrame f = new JFrame();
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setLayout(null);
		f.setSize(800, 450);
		PowerBar p = new PowerBar(0, 0,1f, d);
		f.getContentPane().add(p);
		f.setVisible(true);
		TestThread t = new TestThread(d,p);
		t.start();
	}
	
	private static class TestThread extends Thread {
		boolean isRunning = true;
		boolean pause = false;
		double[] power;
		PowerBar p;
		
		Random rand;
		public TestThread(double[] power, PowerBar p) {
			this.power = power;
			this.p = p;
		}
		@Override
		public void run() {
		while(isRunning) {
			while(pause) { try{Thread.sleep(1); } catch(Exception e) {} }//lock thread
			//DO CHANGE HERE
			if(power[0] <= 11) power[0] = 13;
			else power[0] -= 0.1;
			p.updateDisplay();
			//System.out.println(input[0]);
			try{Thread.sleep(1000); } catch(Exception e) {}//try changing wait time to update time differently
		}}
	}//UpdateThread class
}//PowerBar class
