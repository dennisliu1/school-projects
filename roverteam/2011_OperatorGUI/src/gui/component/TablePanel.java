/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package gui.component;


import java.awt.*;
import java.util.ArrayList;
import java.util.Random;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JTable;

/**
 *
 * @author dennis
 */
public class TablePanel extends GUIComponent {
	private TablePanel self = this;
	public static final int DEFAULT_WIDTH = 150;
	public static final int DEFAULT_HEIGHT_PER_CELL = 25;
	public static final Font DEFAULT_FONT = new Font("Arial", Font.PLAIN, 14);
	public static final Color DEFAULT_BOX_COLOR = Color.black;
	public static final Color DEFAULT_FONT_COLOR = Color.black;
	public static final Color DEFAULT_CONNECTED_COLOR = Color.green;
	public static final Color DEFAULT_DISCONNECTED_COLOR = Color.gray;
	private ArrayList<String> components;
	private ArrayList<double[]> statuses, voltage;
	
	public TablePanel(int x, int y, float transparency, ArrayList<String> data, ArrayList<double[]> statuses, ArrayList<double[]> voltage) {
		this(x,y,DEFAULT_WIDTH,0,transparency, data,statuses,voltage);
	}
	public TablePanel(int x, int y, int width, int height, float transparency, ArrayList<String> data, ArrayList<double[]> statuses, ArrayList<double[]> voltage) {
		super(x,y,width,height,transparency);
		this.components = data;
		this.statuses = statuses;
		this.voltage = voltage;
		height += data.size()*DEFAULT_HEIGHT_PER_CELL;
		this.setHeight(height);
		updateDisplay();
	}
	
	public void paintBuffer(Graphics g) {
		g.setFont(DEFAULT_FONT);
		for(int i = 0; i < components.size()*2; i += 2) {
			if(statuses.get(i/2)[0] > 0) g.setColor(DEFAULT_CONNECTED_COLOR);//draw cell color 
			else g.setColor(DEFAULT_DISCONNECTED_COLOR);
			g.fillRect(0, (i/2)*DEFAULT_HEIGHT_PER_CELL, width-1, DEFAULT_HEIGHT_PER_CELL);
			g.setColor(DEFAULT_FONT_COLOR);//draw component name
			g.drawString(components.get(i/2)+" "+voltage.get(i)[0] + " " + voltage.get(i+1)[0], 2, 5+(i/2)*DEFAULT_HEIGHT_PER_CELL+DEFAULT_FONT.getSize());
			g.setColor(DEFAULT_BOX_COLOR);//draw box around cell
			g.drawRect(0, (i/2)*DEFAULT_HEIGHT_PER_CELL, width, DEFAULT_HEIGHT_PER_CELL-1);
			//System.out.printf("%d:%s\n",i,voltage.get(i)[0]);
		}
		g.setColor(DEFAULT_BOX_COLOR);
		g.drawRect(0, 0, width-1, height-1);
	}
	
	
	public static void main(String[]a) {
		ArrayList<String> components = new ArrayList<String>();
		ArrayList<double[]> statuses = new ArrayList<double[]>();
		ArrayList<double[]> voltage = new ArrayList<double[]>();
		for(int i = 0; i < 1; i++) {
			components.add("a"+i);
			statuses.add(new double[]{0});
			voltage.add(new double[]{0});
			voltage.add(new double[]{0});
		}
		
		//display panel on screen
		JFrame frame = new JFrame("Testing Timer Panel");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setLayout(null);
			TablePanel panel = new TablePanel(0,25, 1f,components, statuses,voltage);
			frame.add(panel);
			TestThread t = new TestThread(components, statuses,voltage,panel);//thread that updates the variable
			t.start();
		frame.setSize(200,200);
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
		ArrayList<String> components;
		ArrayList<double[]> statuses, voltage;
		Random rand;
		TablePanel panel;
		
		public TestThread(ArrayList<String> data, ArrayList<double[]> statuses, ArrayList<double[]> voltage,TablePanel panel) {
			this.components = data;
			this.statuses = statuses;
			this.voltage = voltage;
			this.panel = panel;
			
			this.statuses.get(0)[0] = 5;
			this.voltage.get(0)[0] = 5;
			this.voltage.get(1)[0] = 5;
		}
		@Override
		public void run() {
		while(isRunning) {
			while(pause) { try{Thread.sleep(1); } catch(Exception e) {} }//lock thread
			//DO CHANGE HERE
			//System.out.println(input[0]);
			if(this.voltage.get(0)[0] == 0) {
				this.statuses.get(0)[0] = 5;
				this.voltage.get(0)[0] = 5;
			} else {
				this.statuses.get(0)[0] -= 1;
				this.voltage.get(0)[0] -= 1;
			}
			
			panel.updateDisplay();
			try{Thread.sleep(1000); } catch(Exception e) {}//try changing wait time to update time differently
		}}
	}//UpdateThread class
}//class
