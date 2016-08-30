package compassTilt;


import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import javax.swing.JFrame;
import javax.swing.JPanel;

public class CompassPaint extends JPanel {
	final static Color BG_COLOR = Color.black, ARROW_COLOR = Color.white, CIRCLE_COLOR = Color.green;
	final static int DEF_RADIUS = 65, DEF_CROSSHAIR = 10, DEF_ARROWLEN = 15;
	
	int radius = 65, center = 0, crosshair = 10, arrowLgthExt = 15, arrowHeadLength = 20, offset = arrowLgthExt,
		width, height;
	int angle = 0;
	
	public CompassPaint(int offset, int radius) {
		this.radius = radius;
		this.offset = offset;
		width = radius*2+offset+arrowLgthExt;
		height = radius*2+offset+arrowLgthExt;
		this.crosshair = radius*2/10;
		this.arrowLgthExt = radius/2 - radius/8;
		this.arrowHeadLength = radius/2 - radius/8;
		
		this.setBackground(Color.white);
		this.setPreferredSize(new Dimension(width, height));
		//setSize(200, 250);
	}

	public void update( Graphics g ) {
		//clear screen
		g.setColor(BG_COLOR);
		for(int i = 0; i < width; i++ ) {
			g.drawLine(0, i, height, i);
		}
		
		g.setColor( CIRCLE_COLOR );  
	    g.drawOval(offset, offset, 2*radius, 2*radius);
	    //get defaults
	    //int crosshair = 10;
	    center = offset + radius;
	    int coef = radius - crosshair/2;
	    for (double i = 0; i <= 2*Math.PI; i+=(Math.PI/4)){ //draw circle and crosshairs
	    	g.drawLine((int)(coef*Math.cos(i))+center, center - (int)(coef*Math.sin(i)), (int)((coef+crosshair)*Math.cos(i))+center, center - (int)((coef+crosshair)*Math.sin(i)));
	    }
	    
	    //draw arrow
		g.setColor( ARROW_COLOR );
	    int arrowLength = radius+arrowLgthExt;
	    double angle2 = ((360-angle+90)*Math.PI)/180; //convert into radian
	    int x = center+(int)(Math.cos(angle2)*arrowLength);
	    int y = center-(int)(Math.sin(angle2)*arrowLength);
	    g.drawLine(center, center, x, y);
	    //draw arrowhead
	    	//int arrowHeadLength = 20;
	    double complementAngle = angle2 - Math.PI/4;
	    g.drawLine(x, y, x + (int)(Math.sin(complementAngle)*arrowHeadLength), y + (int)(Math.cos(complementAngle)*arrowHeadLength));
	    g.drawLine(x, y, x - (int)(Math.cos(complementAngle)*arrowHeadLength), y + (int)(Math.sin(complementAngle)*arrowHeadLength));
	    g.setColor( BG_COLOR );
	}
	
	public void paint( Graphics g ) {
		update(g);
	}
	
	/*
	public static void main(String[] a) {
	    JFrame f = new JFrame();
	    f.addWindowListener(new WindowAdapter() {
	      public void windowClosing(WindowEvent e) {
	        System.exit(0);
	      }
	    });
	    f.setContentPane(new CompassGUI(10, 20));
	    f.pack();
	    f.setVisible(true);
	  }
	  */
}