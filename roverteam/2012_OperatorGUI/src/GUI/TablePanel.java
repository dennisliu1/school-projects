/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package GUI;

import java.awt.*;
import java.util.ArrayList;
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
	public static final Color DEFAULT_BACKGROUND_COLOR = Color.gray;
	public static final Color DEFAULT_CONNECTED_COLOR = Color.green;
	public static final Color DEFAULT_DISCONNECTED_COLOR = Color.red;
	private ArrayList<String> components;
	private ArrayList<double[]> statuses, voltage;
	public TableUpdateFunction updater;
	
	public TablePanel(int x, int y, float transparency, ArrayList<String> data, ArrayList<double[]> statuses, ArrayList<double[]> voltage) {
		this(x,y,DEFAULT_WIDTH,0,transparency, data,statuses,voltage);
	}
	public TablePanel(int x, int y, int width, int height, float transparency, ArrayList<String> data, ArrayList<double[]> statuses, ArrayList<double[]> voltage) {
		super(x,y,width,height,transparency);
		this.components = data;
		this.statuses = statuses;
		this.voltage = voltage;
		height += data.size()*DEFAULT_HEIGHT_PER_CELL;
		this.setBounds(x,y, width, height);
		updater = new TableUpdateFunction();
	}
	
	@Override
	public void paintBuffer(Graphics g) {
		Graphics2D g2 = (Graphics2D) g;
		g2.setFont(DEFAULT_FONT);
		for(int i = 0; i < components.size(); i++) {
			if(statuses.get(i)[0] > 0) g2.setColor(DEFAULT_CONNECTED_COLOR);//draw cell color 
			else g2.setColor(DEFAULT_DISCONNECTED_COLOR);
			g2.fillRect(0, i*DEFAULT_HEIGHT_PER_CELL, width-1, (i+1)*DEFAULT_HEIGHT_PER_CELL);
			g2.setColor(DEFAULT_FONT_COLOR);//draw component name
			g2.drawString(components.get(i)+" "+voltage.get(i)[0], 2, i*DEFAULT_HEIGHT_PER_CELL+DEFAULT_FONT.getSize());
			g2.setColor(DEFAULT_BOX_COLOR);//draw box around cell
			g2.drawRect(0, i*DEFAULT_HEIGHT_PER_CELL, width, (i+1)*DEFAULT_HEIGHT_PER_CELL-1);
			//System.out.printf("%d:%s\n",i,voltage.get(i)[0]);
		}
		g2.setColor(DEFAULT_BOX_COLOR);
		g2.drawRect(0, 0, width-1, height-1);
	}
	
	public class TableUpdateFunction extends UpdateFunction {
		@Override
		public boolean checkValues() { return true; }
		@Override
		public void doUpdate() { self.repaint(); }
	}
}//class
